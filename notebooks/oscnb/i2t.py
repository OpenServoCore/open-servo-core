"""The I2t winding estimate the firmware runs, replayed over a capture.

THE ESTIMATE

Every SLOW period (16 ms) the kernel leaks the square of the held current
into a heat state and reads the winding as the board NTC plus a rise that is
proportional to that heat:

  heat += (i^2 - heat) >> tau_shift        counts^2, an EWMA of i^2
  t_winding = t_ntc + k x heat             centi-C, k in Q0.16 centi-C per count^2

`tau_shift` is the leak as a power of two of SLOW periods (the firmware's
`limits::tau_shift` rounds `thermal_tau_ms` to the nearest octave, at most
2^12 periods, 65.5 s), and `k` is the user's rise constant: thermal
resistance from the winding to wherever the NTC sits, times the winding's
resistance, in the board's current counts.

  heat(i_counts, tau_shift)                the heat state, integer, as the kernel keeps it
  rise_cc(heat, k_q16)                     the rise, integer, as the kernel reads it (saturates at i16::MAX)
  tau_shift(tau_s)                         the octave for a time constant in s
  tau_s(shift)                             the time constant an octave runs at
  ewma(i2, tau_s, dt)                      the same leak in floats at any time constant

THE REPLAY

A capture's current polls are put on the kernel's 16 ms grid by holding the
last poll (`grid`), the heat is run over that, and `fit_k` finds the one k
that best explains a truth excess over the base at the truth's own times. A
`grade` is the signed error of the estimate against the truth: its most
negative value is how far the estimate read cold (the protection side), its
most positive how far it read warm.
"""

import numpy as np
import pandas as pd

SLOW_S = 0.016                  # the kernel's SLOW period
TAU_SHIFT_MAX = 12              # the longest leak, 2^12 periods = 65.5 s
HEAT_GUARD = 4                  # guard bits under the heat state
HEAT_MAX = (2 ** 31 - 1) >> HEAT_GUARD
Q16 = 65536
RISE_MAX = 32767                # the rise saturates at i16::MAX centi-C, as the kernel's does


def tau_shift(tau_s):
    """`limits::tau_shift`: the nearest octave of SLOW periods, 0 under two."""
    periods = int(tau_s / SLOW_S)
    if periods < 2:
        return 0
    k = periods.bit_length() - 1
    if periods - (1 << k) > (1 << k) >> 1:
        k += 1
    return min(k, TAU_SHIFT_MAX)


def tau_s(shift):
    return SLOW_S * (1 << shift)


def heat(i_counts, shift):
    """The heat state after each sample of `i_counts` (one per SLOW period),
    in counts^2, with the kernel's integer arithmetic: the square is capped,
    carried with HEAT_GUARD extra bits, and the leak is an arithmetic shift."""
    out = np.empty(len(i_counts), dtype=np.int64)
    h = 0
    for n, i in enumerate(np.asarray(i_counts, dtype=np.int64)):
        x = min(int(i) * int(i), HEAT_MAX) << HEAT_GUARD
        h += (x - h) >> shift
        out[n] = h >> HEAT_GUARD
    return out


def rise_cc(heat_counts2, k_q16):
    """The rise in centi-C the kernel reads from the heat: (heat x k) >> 16,
    saturated at RISE_MAX (the kernel's `.min(i16::MAX)`)."""
    return np.minimum((np.asarray(heat_counts2, dtype=np.int64) * int(k_q16)) >> 16, RISE_MAX)


def ewma(i2, tau, dt):
    """The leak in floats at any time constant: one pole, `dt` per sample."""
    a = dt / tau
    out = np.empty(len(i2))
    h = 0.0
    for n, x in enumerate(np.asarray(i2, float)):
        h += a * (x - h)
        out[n] = h
    return out


def grid(t, i, t0, t1, dt=SLOW_S):
    """Current polls held onto a regular grid from t0 to t1: the kernel runs
    on the held current, so the last poll stands until the next one."""
    tg = np.arange(t0, t1, dt)
    idx = np.searchsorted(np.asarray(t, float), tg, side="right") - 1
    ig = np.where(idx >= 0, np.asarray(i, float)[np.clip(idx, 0, None)], 0.0)
    return tg, ig


def fit_k(heat_at_truth, x_truth):
    """The least-squares k (centi-C per count^2) through the origin, and the
    residual sd in C."""
    h = np.asarray(heat_at_truth, float)
    x = np.asarray(x_truth, float)
    k = float(h @ x / (h @ h))
    return k, float(np.std(x - k * h)) / 100


def grade(est_c, truth_c):
    """Signed error of the estimate against the truth, in C: (min, max, rms)."""
    e = np.asarray(est_c, float) - np.asarray(truth_c, float)
    return float(e.min()), float(e.max()), float(np.sqrt(np.mean(e * e)))


def k_q16_of(k_cc_per_count2):
    return int(round(k_cc_per_count2 * Q16))


def k_physical(r_th_c_per_w, r_ohm, a_per_count):
    """k in centi-C per count^2 from a thermal resistance in C/W, the
    winding's resistance and the board's amps per count."""
    return 100 * r_th_c_per_w * r_ohm * a_per_count ** 2


def table(rows):
    return pd.DataFrame(rows)
