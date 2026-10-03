"""The burst's whole-waveform fit, ported from osc-ident `exp::wavefit`.

WHAT IT FITS

`osc ident burst` records the shunt at one conversion per ~1.08 us around a
duty step from rest, with the driven terminal interleaved. One model of the
PWM period, stepped every 0.25 us, is read through the shunt amplifier and
fitted to the shunt samples of all the from-rest captures at once:

    ON:  L di/dt = V_on - (Rds + Rsh) i - V0 - R i - Ke w
    OFF: L di/dt =      - 2 Rds i       - V0 - R i - Ke w
         J dw/dt = Ke i
    shunt = winding current while ON, 0 while OFF, through a first-order
            lag tau_a, sampled delta late

Five parameters: R, L, V0, tau_a, delta. Ke and J are priors. The loss is
soft L1 with a knee of 8 counts.

WHY A PORT

The Rust fit is the authority, and this module follows it line for line so
it can be cross-checked against the numbers `osc ident` printed for the same
captures. What it adds is the one thing the Rust fit has no switch for:
holding V0 (or any other parameter) at a value the caller names, so that R is
read with the R-V0 trade-off taken out. `Run.fit(fixed=...)` and
`Run.fit_like_ident(fixed)` do that.

SPEED

The simulation is a sequential loop. It runs across "lanes" at once: every
capture, and for the Jacobian every capture again with one parameter nudged,
so one numpy pass over the ~2100 time steps evaluates the whole Jacobian.
"""

from dataclasses import dataclass, field

import gzip
import numpy as np
import pandas as pd
from scipy.optimize import least_squares

# --- constants, each from the Rust source it mirrors ---
SAMPLE_US = 52.0 / 48.0         # burst.rs: one conversion, 26 ADCCLK at HCLK/2
HCLK_MHZ = 48.0                 # burst.rs
CHAN_VMOTOR_A, CHAN_VMOTOR_B = 1, 2
DT_US = 0.25                    # wavefit.rs: model step
KE_PRIOR = 1.334e-3             # V s/rad at the motor shaft, bench MG90 prior
J_PRIOR = 6.9e-9                # kg m2, prior
RDS_ON_OHM = 0.140              # winding.rs: per bridge FET, assumed
F_SCALE = 8.0                   # soft-L1 knee, counts
LEAD_US = 5.0                   # model starts this far before the first rise
EDGE_MARGIN_US = 2.0            # fitted samples stay this far inside the grid
MIN_EDGES = 6
NAMES = ("r", "l_mh", "v0", "tau_a", "delta")
P0 = np.array([4.7, 0.8, 0.3, 1.5, 0.0])
P_LO = np.array([1.0, 0.2, -1.0, 0.3, -3.0])
P_HI = np.array([12.0, 3.0, 2.0, 6.0, 3.0])
P_SCALE = np.array([1.0, 0.2, 0.2, 0.5, 0.5])
LOW_R_FRAC = 0.85               # a capture under this fraction of the median R is set aside


@dataclass(frozen=True)
class Scales:
    """ident's `Scales` for one board: what one count is worth."""
    amps_per_count: float
    v_term_per_count: float
    v_rail_per_count: float
    adc_lsb_v: float
    shunt_ohm: float

    @classmethod
    def of(cls, board):
        return cls(board.a_per_count, board.v_term_per_count,
                   board.v_adc_per_count * board.vbus_ratio, board.v_adc_per_count,
                   board.shunt_ohm)

    def terminal_volts(self, tap, vb):
        # one terminal's absolute volts from its tap: the divider returns to the bias node vb
        return self.v_term_per_count * (tap - vb) + self.adc_lsb_v * vb


@dataclass
class Capture:
    """One burst-N.csv: the meta row and the raw interleaved codes."""
    index: int
    meta: dict
    samples: np.ndarray

    @classmethod
    def read(cls, path, index):
        op = gzip.open if str(path).endswith(".gz") else open
        with op(path, "rt") as fh:
            df = pd.read_csv(fh)
        meta = {k: df[k].iloc[0] for k in df.columns[2:]}
        meta = {k: (int(v) if pd.notna(v) else None) for k, v in meta.items()}
        return cls(index, meta, df["code"].to_numpy(float))

    @property
    def frame_len(self):
        return self.meta["frame_len"]

    @property
    def from_rest(self):
        return self.meta["step_q15"] != 0 and self.meta["pre_q15"] == 0 and not self.meta["seated"]

    def slot(self, bit):
        c = self.meta["chans"]
        return 1 + bin(c & (bit - 1)).count("1") if c & bit else None

    def stream(self, slot):
        return self.samples[slot::self.frame_len]


def _edge_at(vt, t_of, i, lo, hi, rising):
    # the sub-conversion instant an edge crossed half its swing between samples i and i+1
    span = hi - lo
    fa, fb = (vt[i] - lo) / span, (vt[i + 1] - lo) / span
    if not rising:
        fa, fb = 1 - fa, 1 - fb
    if fa > 0.02:
        return t_of(i) + (0.5 - fa) * SAMPLE_US, 0.0
    if fb < 0.98:
        return t_of(i + 1) + (0.5 - fb) * SAMPLE_US, 0.0
    return 0.5 * (t_of(i) + t_of(i + 1)), 0.5 * (t_of(i + 1) - t_of(i))


def _locate(at):
    # one edge's place on the PWM grid from its instances; a caught edge outranks intervals
    caught = [a for a, s in at if s == 0.0]
    if caught:
        return sum(caught) / len(caught)
    lo = max(a - s for a, s in at)
    hi = min(a + s for a, s in at)
    return 0.5 * (lo + hi) if lo <= hi else sum(a for a, _ in at) / len(at)


def _find_edges(vt, t_of, step):
    n = len(vt)
    mx, off = vt.max(), np.median(vt[step:])
    thr = 0.5 * (mx + off)
    hi = vt > thr
    frm = max(step - 2, 0)
    rises = [i for i in range(frm, n - 1) if not hi[i] and hi[i + 1]]
    falls = [i for i in range(frm, n - 1) if hi[i] and not hi[i + 1]]
    out = []
    for r in rises:
        f = next((x for x in falls if x > r), None)
        if f is None:
            break
        off_end = next((x for x in rises if x > f), n - 1)
        on, offs = vt[r + 2:f], vt[f + 3:off_end]
        if len(on) == 0 or len(offs) < 3:
            continue
        level, lo = np.median(on), np.median(offs)
        prev = np.median(vt[r - 8:r]) if r > 8 else lo
        rise, rs = _edge_at(vt, t_of, r, prev, level, True)
        fall, fs = _edge_at(vt, t_of, f, lo, level, False)
        out.append(dict(rise=rise, fall=fall, hi=level, rise_span=rs, fall_span=fs))
    return out


@dataclass
class Raw:
    index: int
    duty_q15: int
    forward: bool
    pos: int
    rail_v: float
    edges: list
    pre_tap: float
    shunt_t: np.ndarray
    shunt_y: np.ndarray


def split(cap, sc):
    fwd = cap.meta["step_q15"] >= 0
    slot = cap.slot(CHAN_VMOTOR_A if fwd else CHAN_VMOTOR_B)
    if slot is None:
        raise ValueError("no driven-terminal channel")
    fl = cap.frame_len
    sh, vt = cap.stream(0), cap.stream(slot)
    n = len(sh)
    step = cap.meta["step_index"] // fl
    if step < 12 or step + 12 > n:
        raise ValueError("the step sits at an end of the capture")
    pre_tap, bias = np.median(vt[:step - 4]), np.median(sh[:step - 4])
    edges = _find_edges(vt, lambda k: (k * fl + slot) * SAMPLE_US, step)
    if len(edges) < MIN_EDGES:
        raise ValueError("too few ON windows on the driven terminal")
    return Raw(cap.index, abs(cap.meta["step_q15"]), fwd, cap.meta["pos"],
               cap.meta["vbus_raw"] * sc.v_rail_per_count, edges, pre_tap,
               np.arange(n) * fl * SAMPLE_US, sh - bias)


def _place(raw, period):
    # the capture's PWM grid from its whole windows: (centre phase, ON width)
    def at(key):
        return [(e[key] - (j + 1) * period, e[key + "_span"]) for j, e in enumerate(raw.edges[1:])]
    rise, fall = _locate(at("rise")), _locate(at("fall"))
    return 0.5 * (rise + fall), fall - rise


@dataclass
class Prepared:
    index: int
    duty: float
    t0: float
    on: np.ndarray
    v: np.ndarray
    windows: list
    ts: np.ndarray
    y: np.ndarray


def prepare(raw, width, period, sc):
    e = raw.edges
    phi = _place(raw, period)[0]
    volts = lambda hi: sc.terminal_volts(hi, raw.pre_tap)
    wins = [(e[0]["rise"], e[0]["fall"], volts(e[0]["hi"]))]
    for k in range(1, len(e) + 2):
        c = phi + k * period
        hi = e[k]["hi"] if k < len(e) else e[-1]["hi"]
        wins.append((c - width / 2, c + width / 2, volts(hi)))
    t0 = e[0]["rise"] - LEAD_US
    t_last = raw.shunt_t[-1]
    nst = max(int(np.floor((t_last + 3.0 - t0) / DT_US)), 1)
    on, v = np.zeros(nst), np.zeros(nst)
    windows = []
    tk = t0 + DT_US * np.arange(nst)
    for a, b, vv in wins:
        cov = np.clip((np.minimum(tk + DT_US, b) - np.maximum(tk, a)) / DT_US, 0, 1)
        on += cov
        v += cov * vv
        mid = round((0.5 * (a + b) - t0) / DT_US)
        if 0 <= mid < nst:
            windows.append((mid, vv))
    grid_end = t0 + DT_US * (nst - 1)
    keep = (raw.shunt_t > t0 + EDGE_MARGIN_US) & (raw.shunt_t < grid_end - EDGE_MARGIN_US)
    return Prepared(raw.index, raw.duty_q15 / 32767, t0, on, v, windows,
                    raw.shunt_t[keep], raw.shunt_y[keep])


@dataclass(frozen=True)
class Model:
    rds: float
    rsh: float
    ke: float
    j: float
    g: float                    # shunt counts per amp


def simulate(caps, m, P, want_current=False):
    """The shunt as the amplifier shows it, for many lanes at once.

    caps: list of Prepared, one per lane; P: (lanes, 5) parameters per lane.
    Returns ys (lanes, steps) and, when asked, the winding current."""
    nl = len(caps)
    ns = max(len(c.on) for c in caps)
    on = np.zeros((nl, ns))
    vk = np.zeros((nl, ns))
    for j, c in enumerate(caps):
        on[j, :len(c.on)] = c.on
        vk[j, :len(c.v)] = c.v
    r, l_mh, v0, tau_a = P[:, 0], P[:, 1], P[:, 2], P[:, 3]
    dt = DT_US * 1e-6
    over_l = dt / (l_mh * 1e-3)
    lag = np.where(tau_a > DT_US, DT_US / np.maximum(tau_a, 1e-9), 1.0)
    spin = m.ke / m.j * dt if m.j > 0 else 0.0
    on_r, off_r = m.rds + m.rsh, 2 * m.rds
    # the per-step resistance the current sees, ON share and OFF share
    rloop = on * on_r + (1 - on) * off_r
    i = np.zeros(nl)
    w = np.zeros(nl)
    y = np.zeros(nl)
    ys = np.empty((nl, ns))
    cur = np.empty((nl, ns)) if want_current else None
    for k in range(ns):
        o = on[:, k]
        drop = np.where((i > 1e-4) | (o > 0), v0, 0.0)
        vd = vk[:, k] - rloop[:, k] * i - drop - r * i - m.ke * w
        i = np.maximum(i + over_l * vd, 0.0)
        w = w + spin * i
        y = y + lag * (o * i - y)
        ys[:, k] = y
        if want_current:
            cur[:, k] = i
    return ys, cur


def _residuals(caps, m, P, ys):
    out = []
    for j, c in enumerate(caps):
        x = (c.ts + P[j, 4] - c.t0) / DT_US - 1.0
        n = len(c.on)
        out.append(m.g * np.interp(x, np.arange(n), ys[j, :n]) - c.y)
    return np.concatenate(out)


@dataclass
class Fit:
    p: dict
    sd: dict
    rms: float
    captures: int
    cost: float


def pooled(caps, m, start, free):
    """Soft-L1 least squares over `caps`, one parameter set for all of them.

    start: length-5 parameter vector; free: indices that move. scipy's
    trust-region solver stands in for ident's own Levenberg-Marquardt; both
    minimise the same soft-L1 cost, and section 2 of notebook 14 checks they
    land on the same numbers."""
    free = list(free)
    p_start = np.asarray(start, float)

    def full(q):
        p = p_start.copy()
        p[free] = q
        return p

    def f(q):
        P = np.tile(full(q), (len(caps), 1))
        ys, _ = simulate(caps, m, P)
        return _residuals(caps, m, P, ys)

    def jac(q):
        base = full(q)
        lanes, Ps = [], []
        hs = []
        for idx in [None] + free:
            p = base.copy()
            if idx is not None:
                h = 1e-6 * max(P_SCALE[idx], abs(p[idx]))
                if p[idx] + h > P_HI[idx]:
                    h = -h
                p[idx] += h
                hs.append(h)
            lanes += caps
            Ps += [p] * len(caps)
        P = np.array(Ps)
        ys, _ = simulate(lanes, m, P)
        nc = len(caps)
        r0 = _residuals(caps, m, P[:nc], ys[:nc])
        cols = []
        for jj in range(len(free)):
            sl = slice((jj + 1) * nc, (jj + 2) * nc)
            cols.append((_residuals(caps, m, P[sl], ys[sl]) - r0) / hs[jj])
        return np.column_stack(cols)

    lo, hi = P_LO[free], P_HI[free]
    q0 = np.clip(p_start[free], lo, hi)
    s = least_squares(f, q0, jac=jac, bounds=(lo, hi), loss="soft_l1", f_scale=F_SCALE,
                      x_scale=P_SCALE[free], method="trf", max_nfev=200)
    r = s.fun
    J = s.jac
    dof = max(len(r) - len(free), 1)
    try:
        cov = np.linalg.inv(J.T @ J) * (r @ r) / dof
        sd = np.sqrt(np.clip(np.diag(cov), 0, None))
    except np.linalg.LinAlgError:
        sd = np.full(len(free), np.nan)
    p = full(s.x)
    sdd = {NAMES[i]: np.nan for i in range(5)}
    for i, v in zip(free, sd):
        sdd[NAMES[i]] = v
    return Fit(dict(zip(NAMES, p)), sdd, float(np.sqrt(np.mean(r ** 2))), len(caps), float(s.cost))


@dataclass
class Run:
    """One run's from-rest captures, split and put on the model grid."""
    raws: list
    prepared: list
    period: float
    on_loss: float
    rail_v: float
    model: Model
    sc: Scales
    notes: list = field(default_factory=list)

    @classmethod
    def build(cls, caps, sc, rds=RDS_ON_OHM, ke=KE_PRIOR, j=J_PRIOR):
        raws, notes = [], []
        rest = [c for c in caps if c.from_rest]
        for c in rest:
            try:
                raws.append(split(c, sc))
            except ValueError as e:
                notes.append(f"capture {c.index}: {e}; not fitted")
        period = 2.0 * rest[0].meta["pwm_arr"] / HCLK_MHZ
        duties = sorted({r.duty_q15 for r in raws})
        widths = {d: np.mean([_place(r, period)[1] for r in raws if r.duty_q15 == d]) for d in duties}
        on_loss = float(np.median([d / 32767 * period - w for d, w in widths.items()]))
        prepared = [prepare(r, widths[r.duty_q15], period, sc) for r in raws]
        model = Model(rds, sc.shunt_ohm, ke, j, 1.0 / sc.amps_per_count)
        return cls(raws, prepared, period, on_loss, float(np.median([r.rail_v for r in raws])),
                   model, sc, notes)

    def fit(self, fixed=None, start=None, keep=None):
        """Pooled fit. `fixed` maps parameter names to values held there.
        `keep` limits the fit to these capture indices."""
        fixed = fixed or {}
        p = np.array(P0 if start is None else start, float)
        for k, v in fixed.items():
            p[NAMES.index(k)] = v
        free = [i for i, n in enumerate(NAMES) if n not in fixed]
        caps = [c for c in self.prepared if keep is None or c.index in keep]
        return pooled(caps, self.model, p, free)

    def alone(self, with_fit):
        """Every capture's own R and L, the rest pinned at `with_fit`."""
        base = np.array([with_fit.p[n] for n in NAMES])
        out = []
        for c in self.prepared:
            f = pooled([c], self.model, base, [0, 1])
            out.append({"index": c.index, "duty": c.duty, "r": f.p["r"], "l_mh": f.p["l_mh"], "rms": f.rms})
        return pd.DataFrame(out)

    def fit_like_ident(self, fixed=None):
        """ident's sequence: pooled, each capture alone, set aside the low R,
        pooled again over the rest."""
        first = self.fit(fixed)
        each = self.alone(first)
        med = each.r.median()
        each["set_aside"] = each.r < LOW_R_FRAC * med
        keep = set(each.index[~each.set_aside])
        start = [first.p[n] for n in NAMES]
        fit = self.fit(fixed, start=start, keep=keep)
        return fit, each

    def duty_volts(self, p, i_a, v_on_open, z_on):
        # duty x rail whose settled winding current is i_a, ident's WaveRun::duty_volts
        v_on = v_on_open - z_on * i_a
        d_on = (p["v0"] + (p["r"] + 2 * self.model.rds) * i_a) / (v_on + (self.model.rds - self.model.rsh) * i_a)
        return (d_on + self.on_loss / self.period) * self.rail_v

    def on_line(self, p, keep=None):
        """The driven terminal's ON level against the winding current, (open, sag ohms)."""
        caps = [c for c in self.prepared if keep is None or c.index in keep]
        P = np.tile([p[n] for n in NAMES], (len(caps), 1))
        _, cur = simulate(caps, self.model, P, want_current=True)
        pts = np.array([(cur[j, k], v) for j, c in enumerate(caps) for k, v in c.windows])
        b, a = np.polyfit(pts[:, 0], pts[:, 1], 1)
        return a, -b

    def v_over_i(self, p, i_a, keep=None):
        a, z = self.on_line(p, keep)
        return self.duty_volts(p, i_a, a, z) / i_a
