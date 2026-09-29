"""Position linearization table, with the semantics of ident/src/lut.rs.

The pot's local gain varies along its travel, so a straight line between the
rails mis-reports position and, worse, speed. A motor-side angle clock (the
commutation ripple, nb06 sec 5) turns that into a table: 55 points evenly
spaced in raw counts between raw_min and raw_max, each an i16 correction
against the identity ramp.

  corrected(raw_i) = raw_i + corr[i]      raw_i = raw_min + i * span / 54
  linearize(raw)   = (corrected(raw) - raw_min) / span, clamped to [0, 1]
  counts(raw)      = raw_min + span * linearize(raw)   linearized counts

All-zero corr is the identity. The rails map to themselves, so linearized
counts are still counts.

  lut = build(chunks, raw_min, raw_max)      chunks = [(pos, cycles), ...]
  st = stitch(chunks, raw_min, raw_max)      the per-count curve under it

A chunk is one time-ordered run of raw pot samples with the motor angle at
every sample, monotone in either direction. The stitch deposits each time
step's angle increment into the counts the pot occupied during that step, so
a chunk's deposits sum to its angle span and the pot's jitter never reorders
the clock (lut.rs stitch_slope). Chunks share the count axis and average per
count, and interior gaps are bridged from the median density of up to 8
covered counts either side.

fit_points anchors the covered pot endpoints affinely onto their identity
fractions and pins the rails to 0 and 1: corrections are against the chord
through the covered ends and taper to zero across uncovered insets. That
anchoring is a gauge choice, so two builds over different covered ends differ
by a tilt that is not a disagreement: compare shapes with chord_residual.

The grid table is the one the firmware applies (core pos_lut.rs): 256 intervals
of 16 raw counts over the 12-bit ADC domain, 257 i16 points at raw k * 16, the
last fixed 0, all-zero the identity. Per sample the firmware indexes with
raw >> 4 and interpolates in integer math to a Q4 word, linearized counts
times 16:

  g = fit_grid(st)                    points on the band >= MIN_RUNG_COVER rungs cross
  g.q4(raw)                           the u16 the firmware computes, bit for bit
  g.counts(raw)                       the same over 16, in counts
  g.validate(raw_min, raw_max)        None, or the firmware's reject reason

fit_grid anchors on rung coverage, never on the extreme sample (one stray rung
moves every point): points inside the band carry the chord through the band
ends, every other point is zero, so the insets and the stops map to themselves.
interp_q4 and validate mirror the firmware and are what a capture of TEL
pos_lin is checked against.

slip_metrics is ident/src/slip.rs, the per-run gear slip check.
"""

import json
from dataclasses import dataclass

import numpy as np

N_POINTS = 55
MIN_SPAN_COVER = 0.7
GAP_ANCHOR = 8


@dataclass(frozen=True)
class PosLut:
    raw_min: int
    raw_max: int
    corr: tuple

    @classmethod
    def identity(cls, raw_min, raw_max, n_points=N_POINTS):
        return cls(int(raw_min), int(raw_max), (0,) * n_points)

    @property
    def span(self):
        return self.raw_max - self.raw_min

    @property
    def points(self):
        n = len(self.corr)
        return self.raw_min + np.arange(n) * self.span / (n - 1)

    def corrected(self, raw):
        r = np.clip(np.asarray(raw, float), self.raw_min, self.raw_max)
        return np.interp(r, self.points, self.points + np.asarray(self.corr, float))

    def linearize(self, raw):
        if self.span <= 0:
            return np.zeros_like(np.asarray(raw, float))
        return np.clip((self.corrected(raw) - self.raw_min) / self.span, 0.0, 1.0)

    def counts(self, raw):
        return self.raw_min + self.span * self.linearize(raw)

    def to_json(self, **extra):
        return json.dumps({"raw_min": self.raw_min, "raw_max": self.raw_max,
                           "lut_corr": [int(c) for c in self.corr], **extra}, indent=2)

    @classmethod
    def from_json(cls, text):
        d = json.loads(text)
        return cls(int(d["raw_min"]), int(d["raw_max"]), tuple(int(c) for c in d["lut_corr"]))


@dataclass(frozen=True)
class Stitch:
    raw_min: int
    cum: np.ndarray      # angle at the lower edge of each count bin, from the first covered bin
    density: np.ndarray  # angle per count, gaps filled
    chunks: np.ndarray   # chunks covering each bin, 0 = gap or outside
    fc: int
    lc: int

    @property
    def coverage(self):
        return float((self.chunks > 0).mean())

    @property
    def lo(self):
        return self.raw_min + self.fc

    @property
    def hi(self):
        return self.raw_min + self.lc

    def angle(self, raw):
        """Stitched angle at raw counts, relative to the first covered count."""
        x = self.raw_min + np.arange(len(self.cum))
        return np.interp(raw, x, self.cum)


def _deposit(pos, cyc, raw_min, raw_max, bins):
    """One chunk's angle per count bin, and which bins it touched."""
    dphi = np.maximum(np.diff(cyc), 0.0)
    p = pos.astype(np.int64)
    a, b = np.minimum(p[:-1], p[1:]), np.maximum(p[:-1], p[1:])
    local = np.zeros(bins)
    touch = np.zeros(bins)
    # sat at one count: the whole step lands there, a rail dwell folds into the top interval
    sat = (a == b) & (a >= raw_min) & (a <= raw_max)
    idx = np.minimum(a[sat], raw_max - 1) - raw_min
    np.add.at(local, idx, dphi[sat])
    np.add.at(touch, idx, 1)
    # crossed [a, b): spread evenly over the counts traversed, clipped to the span
    cr = a != b
    s = dphi[cr] / (b[cr] - a[cr])
    lo = np.clip(a[cr], raw_min, raw_max) - raw_min
    hi = np.clip(b[cr], raw_min, raw_max) - raw_min
    d = np.zeros(bins + 1)
    t = np.zeros(bins + 1)
    np.add.at(d, lo, s)
    np.add.at(d, hi, -s)
    np.add.at(t, lo, 1)
    np.add.at(t, hi, -1)
    local += np.cumsum(d)[:bins]
    touch += np.cumsum(t)[:bins]
    return local, touch > 0


def _anchor(avg, cov, start, step, lo, hi):
    vals, b = [], start
    while len(vals) < GAP_ANCHOR and lo <= b <= hi:
        if cov[b] > 0:
            vals.append(avg[b])
        b += step
    return float(np.sort(vals)[len(vals) // 2]) if vals else 0.0


def stitch(chunks, raw_min, raw_max):
    """Per-count angle over the shared pot axis from several chunks, or None."""
    raw_min, raw_max = int(raw_min), int(raw_max)
    if raw_max <= raw_min:
        return None
    bins = raw_max - raw_min + 1
    acc, cov = np.zeros(bins), np.zeros(bins, dtype=int)
    for pos, cyc in chunks:
        if len(pos) != len(cyc) or len(pos) < 4:
            continue
        local, touched = _deposit(np.asarray(pos), np.asarray(cyc, float), raw_min, raw_max, bins)
        acc[touched] += local[touched]
        cov[touched] += 1
    covered = np.flatnonzero(cov > 0)
    if len(covered) < 2:
        return None
    fc, lc = int(covered[0]), int(covered[-1])
    avg = np.where(cov > 0, acc / np.maximum(cov, 1), 0.0)
    prev = fc
    for b in covered[1:]:
        if b > prev + 1:
            y0 = _anchor(avg, cov, prev, -1, fc, lc)
            y1 = _anchor(avg, cov, b, 1, fc, lc)
            avg[prev + 1:b] = y0 + (y1 - y0) * np.arange(1, b - prev) / (b - prev)
        prev = b
    cum = np.zeros(bins)
    cum[fc + 1:lc + 1] = np.cumsum(avg[fc + 1:lc + 1])
    cum[lc + 1:] = cum[lc]
    if cum[lc] - cum[fc] <= 0:
        return None
    return Stitch(raw_min, cum, avg, cov, fc, lc)


def fit_points(raw, frac, raw_min, raw_max, n_points=N_POINTS):
    """Corrections from (raw, rel) with rel in [0, 1] rising with raw (lut.rs fit_points).
    n_points other than 55 is for studying table size; the firmware block holds 55."""
    o = np.argsort(raw, kind="stable")
    raw, frac = np.asarray(raw, float)[o], np.asarray(frac, float)[o]
    span = raw_max - raw_min
    tf_lo, tf_hi = (raw[0] - raw_min) / span, (raw[-1] - raw_min) / span
    if tf_hi == tf_lo:
        return PosLut.identity(raw_min, raw_max, n_points)
    frac = tf_lo + frac * (tf_hi - tf_lo)
    raw = np.concatenate([raw, [raw_min, raw_max]])
    frac = np.concatenate([frac, [0.0, 1.0]])
    u, inv = np.unique(raw, return_inverse=True)
    f = np.bincount(inv, weights=frac) / np.bincount(inv)
    f = np.maximum.accumulate(f)
    points = raw_min + np.arange(n_points) * span / (n_points - 1)
    corrected = raw_min + np.interp(points, u, f) * span
    d = corrected - points
    # Rust f64::round: half away from zero, not numpy's half to even
    corr = np.clip(np.sign(d) * np.floor(np.abs(d) + 0.5), -32768, 32767).astype(int)
    return PosLut(int(raw_min), int(raw_max), tuple(int(c) for c in corr))


def build(chunks, raw_min, raw_max, min_cover=MIN_SPAN_COVER, n_points=N_POINTS):
    """The LUT from several chunks (lut.rs build_multi); identity when coverage is short."""
    return from_stitch(stitch(chunks, raw_min, raw_max), raw_min, raw_max, min_cover, n_points)


def from_stitch(st, raw_min, raw_max, min_cover=MIN_SPAN_COVER, n_points=N_POINTS):
    if st is None or st.coverage < min_cover:
        return PosLut.identity(raw_min, raw_max, n_points)
    b = np.arange(st.fc, st.lc + 1)
    total = st.cum[st.lc] - st.cum[st.fc]
    return fit_points(raw_min + b, (st.cum[b] - st.cum[st.fc]) / total, raw_min, raw_max, n_points)


def true_counts(st, raw, lo=None, hi=None):
    """Linearized counts straight from a stitch, no points: the curve a finer table converges
    to, the chord through lo and hi (the covered ends unless given) mapping to themselves."""
    lo, hi = st.lo if lo is None else lo, st.hi if hi is None else hi
    rel = (st.angle(raw) - st.angle(lo)) / (st.angle(hi) - st.angle(lo))
    return lo + rel * (hi - lo)


def chord_residual(fn, x, a, b):
    """fn(x) minus the straight line through fn(a) and fn(b): a table's shape with its
    affine part removed, so builds anchored on different covered ends compare."""
    x = np.asarray(x, float)
    ya, yb = fn(np.array([a, b], float))
    return fn(x) - (ya + (yb - ya) * (x - a) / (b - a))


ADC_BITS = 12
GRID_SHIFT = 4
GRID = 1 << GRID_SHIFT                        # raw counts per interval
INTERVALS = 1 << (ADC_BITS - GRID_SHIFT)      # 256
POINTS = INTERVALS + 1                        # point INTERVALS fixed 0
ADC_MASK = (1 << ADC_BITS) - 1
FRAC_MASK = GRID - 1
GAIN_MAX = 16                                 # local gain at or above this is corruption, not a pot
MIN_RUNG_COVER = 20
I16 = (-32768, 32767)


def index(raw):
    """pos_lut.rs index: the interval a raw sample falls in."""
    return (np.asarray(raw, np.int64) & ADC_MASK) >> GRID_SHIFT


def interp_q4(raw, c0, c1):
    """pos_lut.rs interp_q4, bit for bit: linearized counts in Q4 as the u16 the firmware
    computes from a sample and the two points around it. One multiply, no divide, and the
    same wrap as the u16 cast (a table that passes validate never reaches it)."""
    raw = np.asarray(raw, np.int64) & ADC_MASK
    c0, c1 = np.asarray(c0, np.int64), np.asarray(c1, np.int64)
    return (((raw + c0) << GRID_SHIFT) + (c1 - c0) * (raw & FRAC_MASK)) & 0xFFFF


def validate(points, raw_min, raw_max):
    """pos_lut.rs validate: None when the table passes, else the reject reason. Identity
    at and beyond the stops ("ends"), so raw_min and raw_max map to themselves and points
    0, 255 and 256 are zero; then every interval's Q4 gain d = GRID + c[k+1] - c[k] in
    1 <= d < GAIN_MAX * GRID ("shape"), so the output is monotone and stays a u16."""
    c = np.asarray(points, np.int64)
    if len(c) != POINTS or (c < I16[0]).any() or (c > I16[1]).any():
        return "shape"
    k = np.arange(POINTS)
    outside = (k <= (raw_min + GRID - 1) >> GRID_SHIFT) | (k >= raw_max >> GRID_SHIFT)
    if c[outside].any():
        return "ends"
    d = GRID + np.diff(c)
    if (d < 1).any() or (d >= GAIN_MAX * GRID).any():
        return "shape"
    return None


@dataclass(frozen=True)
class GridLut:
    """The firmware's table: POINTS corrections against the identity ramp, point k at raw
    k * GRID, the last fixed 0. All-zero is the identity, raw << 4 everywhere."""
    points: tuple

    @classmethod
    def identity(cls):
        return cls((0,) * POINTS)

    @property
    def raw(self):
        return np.arange(POINTS) * GRID

    @property
    def corr(self):
        return np.asarray(self.points, np.int64)

    def q4(self, raw):
        """The Q4 word the firmware computes for each raw sample."""
        i = index(raw)
        return interp_q4(raw, self.corr[i], self.corr[i + 1])

    def counts(self, raw):
        """Linearized counts as the firmware sees them, the Q4 word over 16."""
        return self.q4(raw) / GRID

    def validate(self, raw_min, raw_max):
        return validate(self.points, raw_min, raw_max)

    def to_json(self, raw_min, raw_max, **extra):
        """The image body osc lut write takes: points 0..INTERVALS-1, the fixed last point
        left out, with the stops the table was built against."""
        return json.dumps({"raw_min": int(raw_min), "raw_max": int(raw_max), "grid_shift": GRID_SHIFT,
                           "points": [int(c) for c in self.points[:INTERVALS]], **extra}, indent=2)

    @classmethod
    def from_json(cls, text):
        d = json.loads(text)
        if d.get("grid_shift", GRID_SHIFT) != GRID_SHIFT or len(d["points"]) != INTERVALS:
            raise ValueError("not a table on the firmware grid")
        return cls(tuple(int(c) for c in d["points"]) + (0,))


def well_covered(st, min_cover=MIN_RUNG_COVER):
    """(lo, hi): the stretch of the stitch at least min_cover chunks cross, or None."""
    well = np.flatnonzero(st.chunks >= min_cover) + st.raw_min
    return (int(well.min()), int(well.max())) if len(well) >= 2 else None


def fit_grid(st, band=None, min_cover=MIN_RUNG_COVER):
    """The grid table from a stitch. Points inside the band (the well-covered stretch unless
    given) carry the chord through the band ends, rounded half away from zero as the Rust
    builder does; every other point is zero. Identity when nothing is covered well enough."""
    if band is None and st is not None:
        band = well_covered(st, min_cover)
    if band is None:
        return GridLut.identity()
    lo, hi = band
    r = np.arange(POINTS) * GRID
    inside = (r >= lo) & (r <= hi)
    d = np.zeros(POINTS)
    d[inside] = true_counts(st, r[inside], lo, hi) - r[inside]
    corr = np.clip(np.sign(d) * np.floor(np.abs(d) + 0.5), *I16).astype(int)
    return GridLut(tuple(int(c) for c in corr))


SLIP_COV_MAX = 0.20
STUCK_FRAC = 0.40


def slip_metrics(pos, phase, segments=10, edge_exclude=1):
    """ident/src/slip.rs: counts per motor angle over equal-index segments of one
    constant-duty run, rail segments dropped. None when the run is degenerate."""
    n = len(pos)
    if n != len(phase) or segments < 3 or 2 * edge_exclude >= segments or n < 2 * segments:
        return None
    loc = []
    for i in range(segments):
        a, z = i * n // segments, (i + 1) * n // segments
        dph = phase[z - 1] - phase[a]
        loc.append(abs(float(pos[z - 1]) - float(pos[a])) / dph if dph > 1e-3 else 0.0)
    core = np.array(loc[edge_exclude:segments - edge_exclude])
    if len(core) < 3 or not (core > 0).any() or core.mean() <= 0:
        return None
    med = float(np.median(core[core > 0]))
    cov = float(core.std() / core.mean())
    stuck = int((core < STUCK_FRAC * med).sum())
    return {"slip_cov": cov, "min_over_median": float(core.min() / med), "stuck_count": stuck,
            "core_segments": len(core), "flagged": cov > SLIP_COV_MAX or stuck >= 1}
