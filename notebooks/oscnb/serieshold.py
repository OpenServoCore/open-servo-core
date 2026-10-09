"""Holds against a series ammeter: a stalled motor driven at a fixed duty for
a few seconds while a handheld meter on DC amps sits in series with one motor
lead, polled by the bench `th ladder` / `th drive` helpers at about 25 ms.

A hold csv has one row per poll: t_unix, phase, the TEL fields (current,
current_trough, i_hat, duty_applied_q15, vdiff_mean, vbus_raw, limit_flags,
pos, ...). The phase names the step: `pre` (one torque-off row), `seek...`,
`hold<limit>`, `rest<limit>` for `th ladder`, and `<tag>_pre`, `<tag>_on`,
`<tag>_tail` for `th drive`. One csv can hold several holds (a duty ladder
writes hold100, hold150, ...). The meter log is `meterlog`'s format; only the
`DC A` rows count here.

These csvs carry no meta.json and no `sense` block. The current reads in raw
counts over the trough sample of the same period (`current - current_trough`),
which cancels the zero and whatever the driver's own supply adds to both
samples, and converts at a counts-per-amp the notebook pins against the same
meter at 100% duty, where the bridge does not switch and the meter and the
chip read one DC current.

  read(path)                         the hold csv as a DataFrame
  holds(df)                          {phase: rows} of every hold in it
  rest_after(df, phase)              the torque-off rows that follow a hold
  score(rows, meter, ...)            one hold against the meter
"""

import numpy as np
import pandas as pd

Q15 = 32768


def read(path):
    return pd.read_csv(path)


def holds(df):
    """{phase: rows} for every hold phase (`hold...` or `..._on`), in the order they ran."""
    ph = [p for p in dict.fromkeys(df["phase"]) if p.startswith("hold") or p.endswith("_on")]
    return {p: df[df["phase"] == p] for p in ph}


def rest_after(df, phase):
    """The torque-off rows of the step after a hold: `rest<limit>` for `hold<limit>`."""
    if phase.startswith("hold"):
        return df[df["phase"] == "rest" + phase[4:]]
    return df[df["phase"] == phase[:-3] + "_tail"]


def score(rows, meter=None, skip_s=3.0, counts_per_a=1118.0, v_per_count=None):
    """One hold, scored from `skip_s` after its first row to its last.

    D          mean |applied duty|, fraction
    vterm      median |vdiff_mean| over the rows that carry one, x v_per_count
               (V; NaN without v_per_count)
    crest_mA   mean (current - current_trough) / counts_per_a
    ihat_mA    mean |i_hat| / counts_per_a, the firmware's gained reading
    trough     mean current_trough, counts (for a zero of your own)
    crest      mean current, counts
    meter_mA   mean |reading| of the meter's DC A rows inside the window
    meter_sign +1 / -1, which way the current crossed the meter
    gap_mA     crest_mA - meter_mA
    n, n_meter rows and meter readings used"""
    t0, t1 = float(rows["t_unix"].iloc[0]), float(rows["t_unix"].iloc[-1])
    w = rows[rows["t_unix"] >= t0 + skip_s]
    vd = w["vdiff_mean"].abs()
    vd = vd[vd != 0]
    out = dict(D=float(w["duty_applied_q15"].abs().mean() / Q15),
               vterm=float(vd.median() * v_per_count) if v_per_count and len(vd) else float("nan"),
               crest_mA=float((w["current"] - w["current_trough"]).mean() / counts_per_a * 1e3),
               ihat_mA=float(w["i_hat"].abs().mean() / counts_per_a * 1e3),
               trough=float(w["current_trough"].mean()), crest=float(w["current"].mean()),
               vbus=float(w["vbus_raw"].mean()), limit_flags="/".join(sorted({str(x) for x in w["limit_flags"]})),
               pos_min=int(w["pos"].min()), pos_max=int(w["pos"].max()), n=len(w),
               meter_mA=float("nan"), meter_sign=0, gap_mA=float("nan"), n_meter=0)
    if meter is not None:
        t, v = meter
        sel = (t >= t0 + skip_s) & (t <= t1)
        if sel.any():
            out.update(meter_mA=float(np.mean(np.abs(v[sel])) * 1e3), meter_sign=int(np.sign(np.mean(v[sel]))),
                       n_meter=int(sel.sum()))
            out["gap_mA"] = out["crest_mA"] - out["meter_mA"]
    return out
