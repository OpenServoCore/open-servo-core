"""Pot linearization LUT, with the semantics of ident/src/lut.rs.

The pot's local gain varies along its travel, so a straight line between the
rails mis-reports position and, worse, speed. A motor-side angle clock (the
commutation ripple, nb06 sec 5) turns that into a table: 55 knots evenly
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

fit_knots anchors the covered pot endpoints affinely onto their identity
fractions and pins the rails to 0 and 1: corrections are against the chord
through the covered ends and taper to zero across uncovered insets. That
anchoring is a gauge choice, so two builds over different covered ends differ
by a tilt that is not a disagreement: compare shapes with chord_residual.

slip_metrics is ident/src/slip.rs, the per-run gear slip check.
"""

import json
from dataclasses import dataclass

import numpy as np

N_KNOTS = 55
MIN_SPAN_COVER = 0.7
GAP_ANCHOR = 8


@dataclass(frozen=True)
class PotLut:
    raw_min: int
    raw_max: int
    corr: tuple

    @classmethod
    def identity(cls, raw_min, raw_max, n_knots=N_KNOTS):
        return cls(int(raw_min), int(raw_max), (0,) * n_knots)

    @property
    def span(self):
        return self.raw_max - self.raw_min

    @property
    def knots(self):
        n = len(self.corr)
        return self.raw_min + np.arange(n) * self.span / (n - 1)

    def corrected(self, raw):
        r = np.clip(np.asarray(raw, float), self.raw_min, self.raw_max)
        return np.interp(r, self.knots, self.knots + np.asarray(self.corr, float))

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


def fit_knots(raw, frac, raw_min, raw_max, n_knots=N_KNOTS):
    """Corrections from (raw, rel) with rel in [0, 1] rising with raw (lut.rs fit_knots).
    n_knots other than 55 is for studying table size; the firmware block holds 55."""
    o = np.argsort(raw, kind="stable")
    raw, frac = np.asarray(raw, float)[o], np.asarray(frac, float)[o]
    span = raw_max - raw_min
    tf_lo, tf_hi = (raw[0] - raw_min) / span, (raw[-1] - raw_min) / span
    if tf_hi == tf_lo:
        return PotLut.identity(raw_min, raw_max, n_knots)
    frac = tf_lo + frac * (tf_hi - tf_lo)
    raw = np.concatenate([raw, [raw_min, raw_max]])
    frac = np.concatenate([frac, [0.0, 1.0]])
    u, inv = np.unique(raw, return_inverse=True)
    f = np.bincount(inv, weights=frac) / np.bincount(inv)
    f = np.maximum.accumulate(f)
    knots = raw_min + np.arange(n_knots) * span / (n_knots - 1)
    corrected = raw_min + np.interp(knots, u, f) * span
    d = corrected - knots
    # Rust f64::round: half away from zero, not numpy's half to even
    corr = np.clip(np.sign(d) * np.floor(np.abs(d) + 0.5), -32768, 32767).astype(int)
    return PotLut(int(raw_min), int(raw_max), tuple(int(c) for c in corr))


def build(chunks, raw_min, raw_max, min_cover=MIN_SPAN_COVER, n_knots=N_KNOTS):
    """The LUT from several chunks (lut.rs build_multi); identity when coverage is short."""
    return from_stitch(stitch(chunks, raw_min, raw_max), raw_min, raw_max, min_cover, n_knots)


def from_stitch(st, raw_min, raw_max, min_cover=MIN_SPAN_COVER, n_knots=N_KNOTS):
    if st is None or st.coverage < min_cover:
        return PotLut.identity(raw_min, raw_max, n_knots)
    b = np.arange(st.fc, st.lc + 1)
    total = st.cum[st.lc] - st.cum[st.fc]
    return fit_knots(raw_min + b, (st.cum[b] - st.cum[st.fc]) / total, raw_min, raw_max, n_knots)


def true_counts(st, raw):
    """Linearized counts straight from a stitch, no knots: the curve a finer table converges to."""
    span = st.lc - st.fc
    rel = (st.angle(raw) - st.cum[st.fc]) / (st.cum[st.lc] - st.cum[st.fc])
    return st.lo + rel * span


def chord_residual(fn, x, a, b):
    """fn(x) minus the straight line through fn(a) and fn(b): a table's shape with its
    affine part removed, so builds anchored on different covered ends compare."""
    x = np.asarray(x, float)
    ya, yb = fn(np.array([a, b], float))
    return fn(x) - (ya + (yb - ya) * (x - a) / (b - a))


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
