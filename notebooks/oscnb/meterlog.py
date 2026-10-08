"""Meter logs: the OWON B41T+ handheld meter over Bluetooth, as the bench's
dmmlog script writes them, one row per packet the meter sent:

  t_unix,func,value,raw,scale,dec

`raw` is the meter's own 6-byte packet in hex, three little-endian 16-bit
words: word 0 holds the function (bits 6..9), the range scale (bits 3..5) and
the decimals (bits 0..2); word 2 is the reading, bit 15 its sign. `value` is
what the logger decoded from it, in base units (ohms, volts, amps). `decode`
does the same from `raw` alone, so a log can be checked against its packets.

  decode(raw_hex)                  (function name, value in base units)
  read(*paths, func="ohm")         the rows of one function, sorted by time, as
                                   numpy arrays (t_unix, value)
  settle(t, v, t_off, ...)         a 60 s settle after a drive: the first
                                   settled second, the rest (last 10 s), the slope
  stills(t, v, min_s, tol)         stretches where the reading holds still

The meter reads ohms by pushing about a milliamp through the part, so a
reading taken while the bridge drives the same terminals is meaningless. Only
standstill readings mean anything; `settle` and `stills` pick those out.
"""

import numpy as np
import pandas as pd

FUNCS = {0: "DC V", 1: "AC V", 2: "DC A", 3: "AC A", 4: "ohm", 5: "F", 6: "Hz",
         7: "%", 8: "C", 9: "F(temp)", 10: "diode", 11: "cont", 12: "hFE", 13: "NCV"}
# range scale -> power of ten, per function family
VOLT_EXP = {3: -3, 4: 0}
AMP_EXP = {2: -6, 3: -3, 4: 0}
OHM_EXP = {4: 0, 5: 3, 6: 6}


def decode(raw_hex):
    """(function name, value in base units) of one packet given as hex; value
    is None for a function or scale with no known exponent."""
    d = bytes.fromhex(raw_hex)
    if len(d) < 6:
        return None, None
    w0 = d[1] << 8 | d[0]
    w2 = d[5] << 8 | d[4]
    func = (w0 >> 6) & 0x0F
    scale = (w0 >> 3) & 0x07
    dec = w0 & 0x07
    meas = w2 if w2 < 0x7FFF else -(w2 & 0x7FFF)
    exp = None
    if func in (0, 1):
        exp = VOLT_EXP.get(scale)
    elif func in (2, 3):
        exp = AMP_EXP.get(scale)
    elif func == 4:
        exp = OHM_EXP.get(scale)
    elif func in (8, 9):
        exp = 0
    val = meas / 10 ** dec
    return FUNCS.get(func, f"f{func}"), (val * 10 ** exp if exp is not None else None)


def frame(*paths):
    """Every row of the logs, concatenated: t_unix and value numeric (a row the
    BLE link garbled reads NaN), sorted by time."""
    parts = []
    for p in paths:
        d = pd.read_csv(p, on_bad_lines="skip")
        d["t_unix"] = pd.to_numeric(d.t_unix, errors="coerce")
        d["value"] = pd.to_numeric(d.value, errors="coerce")
        parts.append(d)
    return pd.concat(parts, ignore_index=True).sort_values("t_unix", kind="stable").reset_index(drop=True)


def read(*paths, func="ohm"):
    """(t_unix, value) of the rows of one function, sorted by time."""
    d = frame(*paths)
    d = d[d.func == func].dropna(subset=["t_unix", "value"])
    return d.t_unix.to_numpy(float), d.value.to_numpy(float)


def settle(t, v, t_off, span_s=61.0, first=(0.5, 1.5), rest=(50.0, 60.0), floor=1.0):
    """One settle after a drive, with time from `t_off` (torque off): the median
    of the readings `first` seconds in, the median of the `rest` window, and the
    slope of a line through 1 s to the end of the rest window in ohm per minute.
    Readings at or under `floor` (the meter's open-lead and range-change
    glitches) are left out. NaN where a window has no reading."""
    sel = (t >= t_off) & (t < t_off + span_s) & (v > floor)
    tt, vv = t[sel] - t_off, v[sel]
    a = vv[(tt >= first[0]) & (tt < first[1])]
    b = vv[(tt >= rest[0]) & (tt < rest[1])]
    mid = (tt >= 1) & (tt < rest[1])
    return dict(first_s=float(np.median(a)) if len(a) else np.nan,
                rest=float(np.median(b)) if len(b) else np.nan,
                slope_per_min=float(np.polyfit(tt[mid], vv[mid], 1)[0] * 60) if mid.sum() > 5 else np.nan)


def stills(t, v, min_s=20.0, tol=0.03):
    """Stretches of at least `min_s` seconds in which no step between
    consecutive readings exceeds `tol`: t0, dur_s, the median of the first 2 s,
    of 29-31 s, and of the last 2 s, and the slope in ohm per minute."""
    out, i, n = [], 0, len(t)
    while i < n:
        j = i
        while j + 1 < n and abs(v[j + 1] - v[j]) <= tol:
            j += 1
        if t[j] - t[i] >= min_s:
            tt, vv = t[i:j + 1] - t[i], v[i:j + 1]
            at30 = vv[(tt >= 29) & (tt < 31)]
            out.append(dict(t0=t[i], dur_s=t[j] - t[i], v_start=float(np.median(vv[tt < 2])),
                            v30=float(np.median(at30)) if len(at30) else np.nan,
                            v_end=float(np.median(vv[tt > tt[-1] - 2])),
                            slope_per_min=float(np.polyfit(tt, vv, 1)[0] * 60)))
        i = j + 1
    return pd.DataFrame(out, columns=["t0", "dur_s", "v_start", "v30", "v_end", "slope_per_min"])
