"""Settled windows by the applied duty.

A rung is settled once the drive has arrived and the shaft has caught up
with it. Two clocks say when:

- The drive. `duty_q15` is the duty the servo applied, `cmd_duty_q15` the
  one commanded. In a `free` dataset (captured before the servo limited
  open-loop current) the two agree from the first sample or two. In a
  `limit` dataset the limiter holds the applied duty under the goal while the
  motor spins up, and a rung reaches its goal tens of ms in, later the
  higher its duty.
- The shaft. Even at the goal from the first sample, a free rung's current
  is still falling for a good part of the rung: from 40 ms past the goal
  to the end, its mean reads 1 to 8% over the last 60%'s on the mg90-a 2S
  grid, 10 to 100% duty. The fixed tail the notebooks always used, the last
  60% of a rung (40% for plant.py's fits), is what holds that out.

`settled(g, tick_hz)` is the part of rung `g` both allow: SETTLE_MS past the
last sample under the goal, and inside the tail. On a free rung the tail
always opens later, so the window is the fixed tail exactly. A rung whose
applied duty ends under its goal has no settled part.
"""

import numpy as np

SETTLE_MS = 40      # after the applied duty reaches the goal, as the capture pilot waits
TAIL = 0.6          # the fraction of a rung kept at most, the fixed tail it replaces


def goal_from(g):
    """Index of the first sample from which the applied duty holds the goal
    to the end of rung `g`; None if the rung ends under it."""
    at = g["duty_q15"].to_numpy() == g["cmd_duty_q15"].to_numpy()
    if len(at) == 0 or not at[-1]:
        return None
    under = np.flatnonzero(~at)
    return int(under[-1]) + 1 if len(under) else 0


def settled_from(g, tick_hz, tail=TAIL, settle_ms=SETTLE_MS):
    """Index where the settled part of rung `g` opens; None if it has none."""
    k = goal_from(g)
    if k is None:
        return None
    n = len(g)
    a = max(int(n * (1 - tail)), k + int(round(settle_ms * tick_hz / 1000)))
    return a if a < n else None


def settled(g, tick_hz, tail=TAIL, settle_ms=SETTLE_MS):
    """The settled part of rung `g`, empty if it has none."""
    a = settled_from(g, tick_hz, tail, settle_ms)
    return g.iloc[a:] if a is not None else g.iloc[:0]
