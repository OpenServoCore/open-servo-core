"""Steady-state shunt bursts: `r4s` point holds on a stalled motor or a
static load, where the drive already sat at its duty when the burst launched
(pre_q15 == step_q15), so every period in the burst is the same period.

A burst free-runs the ADC, one conversion every 52 timer ticks, so its samples
walk through the 2400-tick PWM period and fold back onto it. `start_cnt` and
`start_dir` stamp the counter at the first sample, and every sample's phase
follows from them. Two layouts occur:

  chans 0,  frame_len 1   every sample is the shunt
  chans 11, frame_len 4   shunt, tap A, shunt, tap B: the shunt every other sample

Each burst csv carries its stamp in the first row (k, code, then the meta
columns). These captures have NO meta.json and no `sense` block, so the board
cannot be identified from the data and `check_meta` cannot run on them. The
dataset.toml names the board instead and says why. Everything here is raw ADC
codes over each burst's own OFF-phase zero: no settle gain, no counts per amp,
so a gain change on the board (Rg swapped) changes the counts but not the
percentages these functions return.

  read(path)                  (meta dict, codes) of one burst csv or csv.gz
  tau_on(meta, k)             ticks from the crest at which sample k closes
  burst(path)                 one burst reduced: zero, crest, late-ON line, residuals
  phase_delta(meta, codes)    where the ON window sits against its stamp, ticks
  phase_drops(bursts)         mark the misfolded bursts of one hold
  hold_dirs(capture_dir)      the hold folders of one capture, parsed
  score(bursts)               the motor floor estimator E(w) and its floors
  boot(bursts)                burst bootstrap of the floors and E(48)

THE TIMING

The crest scan's shunt sample closes 31 ticks after the counter crest (2-clock
trigger delay plus a 13.5-clock aperture, 2 ticks a clock, `opasweep`). A
window of w drive ticks either side of the crest therefore reads the shunt
w + 31 ticks after the ON compare. `tau_on` returns time from the crest; the
ON samples of a burst sit at -w <= tau_on <= w, and tau = tau_on + w is time
since the ON compare.

THE MISFOLD CHECK

A firmware race read the counter direction after the PWM trough turn, so a few
bursts carry a stamp a whole half-period off and fold their ON window onto the
OFF phase. `phase_delta` measures where the ON samples actually sit against
where the stamp puts them (the circular mean of the stamped phase of every
sample above half the swing). A sound burst lands within a few ticks of its
hold's median; a misfolded one lands hundreds off. The tolerance is 12 ticks
for chans 0 (about 90 ON samples, robust sd 1.85 ticks over the r4 bursts) and
30 for chans 11 (about 20 ON shunt samples, robust sd 5.4, max 22), set by the
misfold root-cause analysis.
"""

import csv
import gzip
import math
import random
import re
from collections import defaultdict
from pathlib import Path

import numpy as np

from . import boards
from . import opasweep as ow

BOARD = boards.BOARDS["dev-v006-2A"]
HALF = BOARD.pwm_half_ticks                         # ARR, 1200 ticks
PERIOD = 2 * HALF                                   # 2400 ticks, 50 us
STEP = ow.STEP_TICKS                                # 52 ticks between burst samples
AP = int(ow.APERTURE_CLK * ow.TICKS_PER_ADC_CLK)    # 27: a sample closes one aperture after its phase
CREST = int(ow.sample_close_ticks(0))               # 31: the firmware's shunt sample, ticks after the crest
FIT_FROM = 136                                      # the late-ON line is fitted from tau >= 136
CENTRE = HALF - AP                                  # where a centred ON window's stamped phase sits
PHASE_TOL = {0: 12, 11: 30}
WS = (48, 56, 64, 72, 80, 88, 96)


def read(path):
    """(meta, codes): the first row's stamp as ints, every row's code as float."""
    path = Path(path)
    op = gzip.open if path.suffix == ".gz" else open
    with op(path, "rt") as fh:
        rows = list(csv.reader(fh))
    hdr, first = rows[0], rows[1]
    meta = {h: int(v) for h, v in zip(hdr[2:], first[2:]) if v != ""}
    return meta, np.array([float(r[1]) for r in rows[1:]])


def tau_on(meta, k):
    """Ticks from the crest at which sample k closes (negative = before the crest)."""
    phi0 = meta["start_cnt"] if meta["start_dir"] == 0 else PERIOD - meta["start_cnt"]
    return (((phi0 + STEP * np.asarray(k)) % PERIOD + AP - HALF + PERIOD // 2) % PERIOD) - PERIOD // 2


def burst(path):
    """One burst reduced to what the estimators need:

    w      drive ticks, |pre_q15| / 32768 x ARR
    sign   +1 positive duty, -1 negative
    vb     vbus_raw at launch (slot 5 of the scan, closes 291 ticks after the crest)
    zero   median of the OFF samples at least 200 ticks clear of the window
    zsd    their standard deviation
    crest  shunt codes over the zero within 6 ticks of tau = w + 31
    fit    (a, b, sd) of the line a + b tau through the ON samples at tau >= 136,
           None when there are fewer than 4 or any sits more than 10 counts off it
    resid  [(tau, code - zero - line)] for the ON samples before tau 136
    on     [(tau, code - zero)] for every ON sample"""
    meta, code = read(path)
    w = abs(meta["pre_q15"]) / 32768 * HALF
    k = np.arange(len(code))
    keep = np.ones(len(code), bool) if meta["frame_len"] == 1 else (k % 2 == 0)
    t = tau_on(meta, k)
    on = keep & (t >= -w) & (t <= w)
    off = keep & (np.abs(t) >= w + 200) & (np.abs(t) <= PERIOD // 2 - 100)
    z = float(np.median(code[off]))
    tau, x = t[on] + w, code[on] - z
    crest = x[np.abs(tau - (w + CREST)) <= 6]
    late = tau >= FIT_FROM
    fit, resid = None, []
    if late.sum() >= 4:
        A = np.c_[np.ones(late.sum()), tau[late]]
        (a, b), *_ = np.linalg.lstsq(A, x[late], rcond=None)
        r = x[late] - A @ np.array([a, b])
        if np.max(np.abs(r)) <= 10:
            fit = (float(a), float(b), float(np.std(r, ddof=2)) if len(r) > 2 else float("nan"))
            resid = [(float(tt), float(xx - (a + b * tt))) for tt, xx in zip(tau[~late], x[~late])]
    return dict(path=str(path), meta=meta, codes=code, w=w, sign=1 if meta["pre_q15"] > 0 else -1,
                vb=meta["vbus_raw"], zero=z, zsd=float(np.std(code[off], ddof=1)), crest=crest,
                fit=fit, resid=resid, on=list(zip(tau.tolist(), x.tolist())))


def _cmean(ph):
    a = 2 * math.pi * np.asarray(ph) / PERIOD
    s, c = np.sin(a).mean(), np.cos(a).mean()
    return (math.atan2(s, c) * PERIOD / (2 * math.pi)) % PERIOD, math.hypot(s, c)


def _wrap(x):
    return ((x + PERIOD / 2) % PERIOD) - PERIOD / 2


def _on_mask(codes):
    off = float(np.median(codes))
    swing = float(np.percentile(codes, 99)) - off
    return codes > off + max(15.0, 0.5 * swing)


def phase_delta(meta, codes):
    """Ticks between the ON window's centre and where its stamp puts it.

    Shunt samples only, from step_index + 60 on; ON = above the median by half
    the median-to-p99 swing (at least 15 counts); the centre is the circular
    mean of their stamped phases. A burst whose ON samples are too few (< 8),
    too many (> 45%) or too spread (resultant < 0.8) is measured again over
    every sample when it is chans 0, else returns None."""
    flen, ch = meta.get("frame_len", 1), meta.get("chans", 0)
    s = 1 if flen == 1 else (2 if flen in (2, 4) and ch & 0x08 else flen)
    phi0 = meta["start_cnt"] if meta["start_dir"] == 0 else PERIOD - meta["start_cnt"]
    k = np.arange(len(codes))
    sel = (k % s == 0) & (k >= meta.get("step_index", 0) + 60)
    kk, cc = k[sel], codes[sel]
    if len(cc) >= 100:
        on = _on_mask(cc)
        if on.sum() >= 8 and on.mean() <= 0.45:
            c, R = _cmean((phi0 + STEP * kk[on]) % PERIOD)
            if R >= 0.8:
                return round(_wrap(c - CENTRE), 1)
    if flen != 1:
        return None
    on = _on_mask(codes)
    if on.sum() < 8:
        return None
    c, _ = _cmean((phi0 + STEP * k[on]) % PERIOD)
    return _wrap(c - CENTRE)


def phase_drops(bursts):
    """Set b['delta'] and b['phase_bad'] on the bursts of one hold. The baseline
    is the median delta of the same hold and layout, because a slow edge moves
    the ON centroid (a chans-11 caps-in window sits ~40 ticks off, a chans-0
    caps-out one ~27). A burst with no measurable delta counts as bad."""
    by = defaultdict(list)
    for b in bursts:
        b["delta"] = phase_delta(b["meta"], b["codes"])
        by[b["meta"]["chans"]].append(b)
    for ch, grp in by.items():
        ds = [b["delta"] for b in grp if b["delta"] is not None]
        med = float(np.median(ds)) if ds else 0.0
        for b in grp:
            b["phase_bad"] = b["delta"] is None or abs(b["delta"] - med) > PHASE_TOL.get(ch, 12)
    return bursts


def zero_window(b, ref_crest=None, ref_per_vb=None):
    """The earlier drop rule, kept for the sensitivity comparison: OFF sd over
    10 counts, or a median crest under half the reference (the hold's median
    crest, or a plateau per vbus count)."""
    if b["zsd"] > 10:
        return True
    if len(b["crest"]) == 0:
        return False
    m = float(np.median(b["crest"]))
    if ref_per_vb is not None:
        return m / b["vb"] < 0.5 * ref_per_vb
    if ref_crest is not None:
        return m < 0.5 * ref_crest
    return False


HOLD_RE = re.compile(r"^(?P<rail>\d+(?:\.\d+)?)_(?P<pct>\d+(?:\.\d+)?)(?:_(?P<stop>low|high))?$")


def hold_dirs(capture_dir):
    """[(rail V, duty %, stop or None, path)] for every hold folder of a capture,
    named <rail>_<duty>[_<stop>], in name order (the order the bootstrap draws in)."""
    out = []
    for d in Path(capture_dir).iterdir():
        m = HOLD_RE.match(d.name)
        if m and d.is_dir():
            out.append((float(m["rail"]), float(m["pct"]), m["stop"], d))
    return sorted(out, key=lambda r: r[3].name)


def hold_bursts(path):
    """Every burst of one hold folder, reduced and phase-checked."""
    bs = [burst(f) for f in sorted(Path(path).glob("ident/*/burst-*.csv*"), key=str)]
    return phase_drops(bs)


def late_level(b):
    """The burst's late-ON line averaged over tau 136 to 2w in 8-tick steps."""
    a, s, _ = b["fit"]
    return float(np.mean([a + s * t for t in range(136, int(2 * b["w"]) + 1, 8)]))


def resid_at(b, tau, tol=6):
    """Mean residual within `tol` ticks of `tau`, over the line there; None
    without a line or a sample."""
    xs = [r for t, r in b["resid"] if abs(t - tau) <= tol]
    if not xs or not b["fit"]:
        return None
    a, s, _ = b["fit"]
    return float(np.mean(xs)) / (a + s * tau)


def curve(per, bw=4, lo=40, hi=136):
    """Residual lists pooled into `bw`-tick bins, keyed by bin centre."""
    d = defaultdict(list)
    for pts in per:
        for t, x in pts:
            if lo <= t < hi:
                d[int(t // bw) * bw].append(x)
    return {k + bw / 2: float(np.mean(v)) for k, v in sorted(d.items())}


def at(c, tau):
    """Linear interpolation in a curve; NaN outside it."""
    ks = sorted(c)
    for a, b in zip(ks, ks[1:]):
        if a <= tau <= b:
            return c[a] + (c[b] - c[a]) * (tau - a) / (b - a)
    return float("nan")


def score(bursts):
    """The motor floor estimator on the bursts of one session and rail (each
    with b['D'] the duty in %).

    resid(tau)  the residual curve pooled over the D 8/10/15 bursts, 4-tick bins
    R(w)        mean crest of the D 6/8/10 holds, linear in w between them and
                EXTRAPOLATED below 72 ticks
    E(w)        resid(w + 31) / (R(w) - resid(w + 31)), percent: how far the
                crest at window w sits off what the settled line would read
    floors      {bound: smallest w on a 2-tick grid from 40 with |E| <= bound
                for every w up to 100}, for bounds 2% and 1%
    late        mean late-ON level of the D 8/10/15 lines, for residuals in %"""
    g = defaultdict(list)
    for b in bursts:
        g[b["D"]].append(b)
    per = [b["resid"] for D in (8, 10, 15) for b in g.get(D, []) if b["fit"]]
    c = curve(per)
    R = {D * 12: float(np.mean([x for b in g[D] for x in b["crest"]])) for D in (6, 8, 10) if g.get(D)}
    ws = sorted(R)
    late = [b["fit"][0] + b["fit"][1] * (136 + 2 * b["w"]) / 2 for D in (8, 10, 15) for b in g.get(D, []) if b["fit"]]
    out = dict(c=c, R=R, n=len(per), late=float(np.mean(late)) if late else float("nan"))
    if len(ws) < 3:
        out.update(E=None, floors={2: None, 1: None})
        return out

    def Rw(w):
        a, b = (ws[0], ws[1]) if w <= ws[1] else (ws[1], ws[2])
        return R[a] + (R[b] - R[a]) * (w - a) / (b - a)

    def E(w):
        r = at(c, w + CREST)
        return r / (Rw(w) - r) * 100

    fl = {}
    for bound in (2, 1):
        ok = [w for w in range(40, 101, 2) if all(abs(E(x)) <= bound for x in range(w, 101, 2))]
        fl[bound] = min(ok) if ok else None
    out.update(E=E, floors=fl)
    return out


def boot(bursts, n=300, seed=7):
    """Resample bursts with replacement inside each (D, stop) hold and score
    again: 5-95% ranges of the 2% and 1% floors (102 = no floor up to 100) and
    of E(48), and E(48)'s standard deviation. None when the set has no floor."""
    random.seed(seed)
    g = defaultdict(list)
    for b in bursts:
        g[(b["D"], b["stop"])].append(b)
    f2, f1, e48 = [], [], []
    for _ in range(n):
        s = [random.choice(v) for v in g.values() for _ in v]
        r = score(s)
        if r["E"] is None:
            return None
        f2.append(r["floors"][2] or 102)
        f1.append(r["floors"][1] or 102)
        e48.append(r["E"](48))
    q = lambda v: (float(np.percentile(v, 5)), float(np.percentile(v, 95)))
    return dict(f2=q(f2), f1=q(f1), e48=q(e48), e48_sd=float(np.std(e48)))
