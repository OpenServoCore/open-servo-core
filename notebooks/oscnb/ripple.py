"""Commutation-ripple tracker: a motor-side speed and angle from the shunt current.

Each brush crossing a commutator gap steps the winding current, six times per
motor revolution on a 3-segment, 2-brush motor. Sampled once per PWM period the
chop cancels and the ripple is a clean spectral line at six times the motor's
revolutions per second. The line owes nothing to the pot, and its integrated
phase is a motor angle in ripple cycles (nb06 sec 5).

  f, X = line_spectrum(x, tick_hz)            amplitude spectrum, Hann, 8x padded
  fr, amp = settled_line(f, X)                the line of a settled window
  cyc, n0, ridge = motor_angle(seg, c_prior, tick_hz, v_per_count)
                                              cycles at every sample of a step

motor_angle is two passes: a ridge track walking backward from the settled end
of the step in 200-sample windows (10 ms at 20 kHz), then a demodulation
against the ridge phase that counts cycles exactly wherever the line is present.

Known traps, each one measured in nb06:

- A spectral peak is the settled line, not a mean. Over a step whose shaft is
  still accelerating a single peak sits at the final frequency, so dividing it
  by a mean speed reads 5.5% high (sec 5.5). Count demodulated cycles for a
  mean rate.
- The search band is per supply. 1400-5200 Hz holds the 2S lines at 30 to 45%
  duty and misses every USB line (about 470 to 1700 Hz). Off 2S, search a band
  relative to the settled line (0.7 to 1.4 times it), which still excludes 2f
  and the family below.
- The line has a family at f/2, f/3 and f/6, a once-per-revolution modulation
  that grows with speed and overtakes the line at 60% duty. settled_line
  promotes the largest peak to its 6x, 3x or 2x multiple when that is at least
  half as tall, which does not always rescue a 60% step. motor_angle seeds from
  the pot speed times a coupling prior instead, and that is why it takes one.
- The tracker needs the line above 250 Hz, about 1 wiper V/s on sg90-a.
"""

import numpy as np
import pandas as pd

from scipy.signal import get_window, detrend, butter, sosfiltfilt, medfilt

F_LO, F_HI = 200.0, 6000.0
SETTLED = 0.6


def line_spectrum(x, tick_hz, nfft_mult=8):
    """Amplitude spectrum in the units of x, coherent gain of the Hann window removed."""
    x = detrend(x)
    w = get_window("hann", len(x))
    nfft = nfft_mult * len(x)
    return np.fft.rfftfreq(nfft, 1.0 / tick_hz), np.abs(np.fft.rfft(x * w, nfft)) * 2 / w.sum()


def interp_peak(f, X, lo, hi):
    m = (f >= lo) & (f <= hi)
    Xm, fm = X[m], f[m]
    i = int(np.argmax(Xm))
    d = 0.0
    if 0 < i < len(Xm) - 1:
        a, b, c = np.log(Xm[i - 1]), np.log(Xm[i]), np.log(Xm[i + 1])
        d = 0.5 * (a - c) / (a - 2 * b + c)
    return fm[i] + d * (f[1] - f[0]), Xm[i]


def half_width(f, X, f0):
    m = (f >= 0.7 * f0) & (f <= 1.3 * f0)
    fm, Xm = f[m], X[m]
    i = int(np.argmax(Xm))
    above = Xm >= Xm[i] / 2
    lo = i
    while lo > 0 and above[lo - 1]:
        lo -= 1
    hi = i
    while hi < len(Xm) - 1 and above[hi + 1]:
        hi += 1
    return fm[hi] - fm[lo]


def settled_line(f, X, lo=F_LO, hi=F_HI):
    """(frequency, amplitude) of the ripple line in a settled window."""
    f0, a0 = interp_peak(f, X, lo, hi)
    for k in (6, 3, 2):
        if k * f0 * 1.05 <= hi:
            fk, ak = interp_peak(f, X, k * f0 * 0.95, k * f0 * 1.05)
            if ak >= 0.5 * a0:
                return fk, ak
    return f0, a0


def tone_width(n, tick_hz):
    """Resolution floor of an n-sample window: a pure tone's width through line_spectrum."""
    x = np.sin(2 * np.pi * 1000.0 * np.arange(n) * (1.0 / tick_hz))
    f, X = line_spectrum(x, tick_hz)
    return half_width(f, X, 1000.0)


def settled(seg, frac=SETTLED):
    """The last `frac` of a step."""
    n0 = int(len(seg) * (1 - frac))
    return seg.iloc[n0:]


def pot_speed(seg, tick_hz, v_per_count):
    """Least-squares slope of the wiper over the window, in wiper V/s."""
    p = seg["pos"].to_numpy() * v_per_count
    return np.polyfit(np.arange(len(p)) * (1.0 / tick_hz), p, 1)[0]


def ridge_track(x, seed, tick_hz, win=200, hop=20, tol=0.15, fmin=150.0, fmax=F_HI, nfft=8192):
    """Sliding-window peak track, walking backward from the end, seeded with the settled line.

    tol = 0.15 excludes the nearest family member (5/6 of the line) while the
    line itself moves at most 6% per hop at the steepest acceleration seen.
    """
    w = get_window("hann", win)
    f = np.fft.rfftfreq(nfft, 1.0 / tick_hz)
    band = (f >= fmin) & (f <= fmax)
    out = []
    fprev = seed
    for c in np.arange(win // 2, len(x) - win // 2, hop)[::-1]:
        X = np.abs(np.fft.rfft(detrend(x[c - win // 2:c + win // 2]) * w, nfft)) * 2 / w.sum()
        fpk, amp = interp_peak(f, X, max(fmin, fprev * (1 - tol)), min(fmax, fprev * (1 + tol)))
        out.append((c, fpk, amp, np.median(X[band])))
        fprev = fpk
    return pd.DataFrame(out[::-1], columns=["n", "f", "amp", "noise"])


def demod_cycles(x, n_ref, f_ref, tick_hz, lp_frac=0.25):
    """Ripple phase in cycles at every sample: mix down with the ridge phase,
    low-pass, unwrap the residual, add the ridge phase back."""
    n = np.arange(len(x))
    fref = np.interp(n, n_ref, f_ref)
    phref = 2 * np.pi * np.cumsum(fref) * (1.0 / tick_hz)
    z = detrend(x) * np.exp(-1j * phref)
    sos = butter(3, max(30.0, lp_frac * fref.min()), "lowpass", fs=tick_hz, output="sos")
    zb = sosfiltfilt(sos, z, padlen=min(1000, len(z) - 1))
    return (phref + np.unwrap(np.angle(zb))) / (2 * np.pi)


def motor_angle(seg, c_prior, tick_hz, v_per_count, snr_min=2.0, f_floor=250.0, prior_tol=0.3):
    """(cycles at every sample, first accepted sample, ridge) of one drive step,
    or (None, None, None) if nothing in it tracked.

    seg needs current_raw, bias and pos. c_prior is a coupling estimate in
    ripple cycles per wiper V, used only to seed off the sub-line family.
    """
    x = seg["current_raw"].to_numpy().astype(float) - seg["bias"].iloc[0]
    st = settled(seg)
    v = abs(pot_speed(st, tick_hz, v_per_count))
    f, X = line_spectrum(x[len(seg) - len(st):], tick_hz)
    seed, _ = interp_peak(f, X, max(F_LO, c_prior * v * (1 - prior_tol)), min(F_HI, c_prior * v * (1 + prior_tol)))
    r = ridge_track(x, seed, tick_hz)
    ok = ((r["amp"] / r["noise"] >= snr_min) & (r["f"] >= f_floor)).to_numpy()
    bad = np.where(~ok)[0]
    r = r.iloc[bad.max() + 1:] if len(bad) else r
    if len(r) < 2:
        return None, None, None
    cyc = demod_cycles(x, r["n"].to_numpy(), r["f"].to_numpy(), tick_hz)
    # Drops the first 100 samples of the accepted track: 5 ms at 20 kHz, fixed in samples.
    return cyc, int(r["n"].iloc[0]) + 100, r


def local_gains(trace, win_cycles=5, bin_v=0.02):
    """Slope of position against motor angle in consecutive windows of
    win_cycles, each placed at its mean position rounded to bin_v.

    trace needs p (wiper V) and theta (cycles). Windows are cut on the motor
    angle, the independent variable, so the slope is unbiased.
    """
    p, th = trace["p"].to_numpy(), trace["theta"].to_numpy()
    out = []
    for a in np.arange(th.min(), th.max() - win_cycles, win_cycles):
        m = (th >= a) & (th < a + win_cycles)
        if m.sum() >= 20:
            out.append({"bin_V": np.round(p[m].mean() / bin_v) * bin_v,
                        "mV_per_cycle": np.polyfit(th[m], p[m], 1)[0] * 1e3})
    return pd.DataFrame(out)


def step_events(x, frac=0.10, floor=10, dist=4):
    """Sample indices of commutation steps: one-sample jumps of at least
    max(floor, frac x local current), jumps within dist samples merged."""
    d1 = np.zeros_like(x)
    d1[1:] = np.diff(x)
    thr = np.maximum(floor, frac * np.abs(medfilt(x, 21)))
    ev = []
    for k in np.where(np.abs(d1) >= thr)[0]:
        if ev and k - ev[-1][0] <= dist:
            if abs(d1[k]) > abs(ev[-1][1]):
                ev[-1] = (k, d1[k])
        else:
            ev.append((k, d1[k]))
    return np.array([e[0] for e in ev], dtype=int)
