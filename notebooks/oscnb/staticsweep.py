"""Static-load sweep captures: `osc sweep --static-load` chains driven one cycle
at a time by a bench runner that logs every cycle to a run log.

A static-load sweep holds no position loop and seeks nowhere (a bare motor has
no pot). The servo takes a duty schedule in one commit under the stall permit
and streams the TEL burst at every kernel tick (20 kHz) for each segment. An
experiment folder of this kind holds:

  runlog.jsonl              one JSON object per runner event; `cycle` rows
                            carry k, rail (V set on the supply), duty (%), dir
                            (+1/-1), steps (the schedule), t_sweep0, t_off
                            (unix s at torque off), rc (0 = the sweep wrote its
                            capture), pre/post status and, from the second part
                            on, `part`
  capture-K/sweep.csv.gz    cycle K's per-tick rows: seg (1-based schedule
                            entry), duty_q15 (applied), current_raw,
                            current_trough, vmotor_a, vmotor_b, ...
  capture-K/meta.json       the sweep's meta, with the `sense` block
  capture-K/sweep.log       the sweep's console (seq holes, garble per segment)

The schedule is a comma list. `D@ms` drives D% for ms, `then:D@ms` follows it
in the same commit without a rest, and `coast:ms` holds duty 0 with both legs
off, the bridge Hi-Z, so the terminals float on the back-EMF. `segments` names
each seg by its entry: a drive followed by `then:` is the `stall` pre-pulse,
the last drive is the `spin`, and the coast entries are the `coast`.

  cycles(ds, exp)                   the run log's cycle rows, one per k
  frame(ds, k, exp)                 cycle k's per-tick rows, sense checked
  segments(frame, steps)            {"stall", "spin", "coast"} frames
  line_freq(x, fs, lo, hi)          the strongest spectral line in a band
  count_cycles(x, f0, fs, frac)     mean frequency over a whole number of cycles

SPEED BY COUNTING

The commutation ripple of a 3-slot motor is 6 events per motor revolution, on
the drive current and on the floating terminals alike. A spectral peak reports
the settled line and is biased when the line sweeps inside the window (a coast
does nothing else), so `count_cycles` band-passes the signal around a seed
frequency, finds the upward zero crossings to a fraction of a sample, drops
the cycles at the filter's edges, and divides the whole cycles by their span.
"""

import json

import numpy as np
import pandas as pd
from scipy.signal import butter, sosfiltfilt

FS = 20000.0


def runlog(ds, exp="char"):
    """Every event of the run log, in file order."""
    with open(ds.path / exp / "runlog.jsonl") as fh:
        return [json.loads(line) for line in fh if line.strip()]


def cycles(ds, exp="char"):
    """The cycle rows of the run log as a frame indexed by k, with `captured`
    True where capture-k holds per-tick rows. A k logged twice keeps its first
    row."""
    rows = []
    for r in runlog(ds, exp):
        if r.get("event") != "cycle":
            continue
        rows.append(dict(k=r["k"], part=str(r.get("part") or 1), rail=r["rail"], duty=r["duty"], dir=r["dir"],
                         steps=r["steps"], rc=r["rc"], t_sweep0=r.get("t_sweep0"), t_off=r.get("t_off"),
                         pre_trough=r.get("pre", {}).get("trough"), pre_vbus_raw=r.get("pre", {}).get("vbus_raw"),
                         nudge_counts=r.get("nudge_counts"), spin_counts=r.get("spin_counts"),
                         fail=r.get("fail")))
    c = pd.DataFrame(rows).drop_duplicates("k").set_index("k").sort_index()
    c["captured"] = [(ds.path / exp / f"capture-{k}" / "sweep.csv.gz").exists() for k in c.index]
    return c


def frame(ds, k, exp="char"):
    """Cycle k's per-tick rows; reading checks the meta's sense block against
    the dataset's board and its drive rule against the dataset's."""
    return ds.read(exp, f"capture-{k}", "sweep")


def segments(frame, steps):
    """Split a cycle's rows by schedule entry: `stall` the drive a `then:`
    follows, `spin` the last drive, `coast` every coast entry in order (one
    frame). Missing roles are empty frames. Each comes back re-indexed."""
    entries = steps.split(",")
    drives = [i for i, e in enumerate(entries, 1) if not e.startswith("coast")]
    coasts = [i for i, e in enumerate(entries, 1) if e.startswith("coast")]
    spin = drives[-1] if drives else None
    stall = drives[0] if len(drives) > 1 and entries[drives[1] - 1].startswith("then:") else None
    pick = lambda segs: frame[frame.seg.isin(segs)].reset_index(drop=True)
    return dict(stall=pick([stall] if stall else []), spin=pick([spin] if spin else []), coast=pick(coasts))


def line_freq(x, fs=FS, lo=150.0, hi=3000.0):
    """(frequency, prominence) of the strongest line in [lo, hi] Hz: the peak of
    a Hann-windowed spectrum zero-padded 8x, and its height over the band's
    median."""
    x = np.asarray(x, float) - np.mean(x)
    n = len(x)
    X = np.abs(np.fft.rfft(x * np.hanning(n), 8 * n))
    f = np.fft.rfftfreq(8 * n, 1 / fs)
    m = (f >= lo) & (f <= hi)
    k = np.argmax(X[m])
    return float(f[m][k]), float(X[m][k] / np.median(X[m]))


def count_cycles(x, f0, fs=FS, frac=0.35):
    """(mean frequency, first sample, last sample, cycles) over a whole number
    of cycles of the line near f0: a 2nd-order band-pass f0 x (1 +/- frac) run
    forward and back, upward zero crossings interpolated to a fraction of a
    sample, the first and last crossing dropped as filter edge. NaN and zero
    cycles when fewer than four crossings are found."""
    sos = butter(2, [f0 * (1 - frac), f0 * (1 + frac)], btype="band", fs=fs, output="sos")
    y = sosfiltfilt(sos, np.asarray(x, float) - np.mean(x))
    s = np.flatnonzero((y[:-1] < 0) & (y[1:] >= 0))
    if len(s) < 4:
        return np.nan, 0, 0, 0
    tc = s + (-y[s]) / (y[s + 1] - y[s])
    tc = tc[1:-1]
    n = len(tc) - 1
    return n / ((tc[-1] - tc[0]) / fs), int(np.ceil(tc[0])), int(np.floor(tc[-1])), n
