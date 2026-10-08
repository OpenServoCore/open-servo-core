"""Winding R as a thermometer, and a two-node thermal model to read it with.

THE R-V0 RIDGE

The burst's waveform fit cannot tell R from V0 well inside one run: both make
the current settle lower, so a run that lands on a more negative V0 lands on a
higher R. Across many runs at one temperature the (V0, R) points lie on a line,
and the scatter of R is mostly where each run fell along it. Moving every run
along that line to one reference V0 takes the trade out:

  slope, icept, corr = ridge(r, v0)          R against V0 across runs
  rc = project(r, v0, v0_ref, slope)          R moved along the line to v0_ref

The slope can also come from the fit itself (hold V0 on a grid, nb02 sec 22);
the two agreeing is what says the scatter is the fit's trade, not the motor.

COPPER

Copper's resistance is proportional to (T0_CU + T) with T in C, a line that
would reach zero at -234.5 C; referred to R at 20 C that is 0.393 %/C.

  t = copper_c(r, r_ref, t_ref)              the temperature R implies, linear in ALPHA_CU
  r = copper_ohm(t, r_ref, t_ref)            and back
  t = thermometer_c(r, r_ref, t_ref)         the same on the (T0_CU + T) line (nb17 sec 7)
  r = copper_carry(r, t, t_ref)              R read at t, carried to t_ref along that line
  copper_pct_per_c(t)                        the coefficient at t, % of R(t) per C

THE TWO-NODE MODEL

Heat goes into the winding, crosses to the can, and leaves the can to the air.
A share s of the heat lands on the winding and the rest straight on the can:

  C_w dT_w/dt = s I^2 R(T_w) - (T_w - T_c) / R_wc
  C_c dT_c/dt = (1 - s) I^2 R(T_w) + (T_w - T_c) / R_wc - (T_c - T_a) / R_ca
  R(T_w) = r_ref (1 + alpha (T_w - t_ref)),  T_a = ta0 + tb t

With I^2 held over a segment the system is linear in (T_w, T_c, T_a, 1), so it
steps exactly from one event to the next with a matrix exponential: no time
step to choose. `TwoNode.modes` gives the two time constants of the unheated
network. Neither belongs to one node alone unless the nodes are far apart.

THE CAN NTC

  ntc_c(raw)                                  the J7 NTC, 10K 3950 under a 10K
                                              pull-up to 3V3, in C
"""

from dataclasses import dataclass

import numpy as np
from scipy.linalg import expm

ALPHA_CU = 0.0039               # copper, per degree C near room temperature
T0_CU = 234.5                   # copper: R is proportional to (T0_CU + T), T in C
NTC_B = 3950.0
NTC_T0_K = 298.15
ADC_FULL = 4095


def ntc_c(raw):
    """The can NTC reading in C: 10K 3950 to GND under a 10K pull-up to 3V3."""
    raw = np.asarray(raw, float)
    return 1 / (1 / NTC_T0_K + np.log(raw / (ADC_FULL - raw)) / NTC_B) - 273.15


def ridge(r, v0):
    """R against V0 across runs: (slope ohm/V, intercept ohm, correlation)."""
    r, v0 = np.asarray(r, float), np.asarray(v0, float)
    slope, icept = np.polyfit(v0, r, 1)
    return float(slope), float(icept), float(np.corrcoef(v0, r)[0, 1])


def project(r, v0, v0_ref, slope):
    """R moved along a ridge of `slope` ohm/V from its own V0 to v0_ref."""
    return np.asarray(r, float) + slope * (v0_ref - np.asarray(v0, float))


def copper_c(r, r_ref, t_ref, alpha=ALPHA_CU):
    return t_ref + (np.asarray(r, float) / r_ref - 1) / alpha


def copper_ohm(t, r_ref, t_ref, alpha=ALPHA_CU):
    return r_ref * (1 + alpha * (np.asarray(t, float) - t_ref))


def copper_pct_per_c(t):
    return 100 / (T0_CU + np.asarray(t, float))


def copper_carry(r, t, t_ref):
    return np.asarray(r, float) * (T0_CU + t_ref) / (T0_CU + np.asarray(t, float))


def thermometer_c(r, r_ref, t_ref):
    return t_ref + (np.asarray(r, float) / r_ref - 1) * (T0_CU + t_ref)


@dataclass(frozen=True)
class TwoNode:
    """Heat capacities in J/C, thermal resistances in C/W, `share` the part
    of the heat that lands on the winding."""
    c_w: float
    c_c: float
    r_wc: float
    r_ca: float
    share: float = 1.0

    def modes(self):
        """The unheated network's two time constants in s, (fast, slow)."""
        a = np.array([[-1 / (self.r_wc * self.c_w), 1 / (self.r_wc * self.c_w)],
                      [1 / (self.r_wc * self.c_c), -(1 / self.r_wc + 1 / self.r_ca) / self.c_c]])
        lam = np.sort(np.linalg.eigvals(a).real)
        return float(-1 / lam[0]), float(-1 / lam[1])

    def _a(self, i2, r_ref, t_ref, alpha, tb):
        cw, cc, rwc, rca, s = self.c_w, self.c_c, self.r_wc, self.r_ca, self.share
        heat_t, heat_1 = i2 * r_ref * alpha, i2 * r_ref * (1 - alpha * t_ref)
        return np.array([
            [(s * heat_t - 1 / rwc) / cw, 1 / (rwc * cw), 0.0, s * heat_1 / cw],
            [((1 - s) * heat_t + 1 / rwc) / cc, -(1 / rwc + 1 / rca) / cc, 1 / (rca * cc), (1 - s) * heat_1 / cc],
            [0.0, 0.0, 0.0, tb],
            [0.0, 0.0, 0.0, 0.0],
        ])

    def run(self, t0, x0, heat, t_obs, r_ref, t_ref, alpha=ALPHA_CU, tb=0.0):
        """Winding and can temperatures at the times t_obs.

        t0: when the state is x0 = (T_w, T_c, T_a). heat: (start, end, I^2)
        windows, I^2 in A^2 rms, nothing outside them. t_obs must not precede
        t0. Returns (T_w, T_c, T_a) arrays in the order of t_obs."""
        t_obs = np.asarray(t_obs, float)
        if np.any(t_obs < t0):
            raise ValueError("an observation precedes the start state")
        edges = sorted({t for a, b, _ in heat for t in (a, b) if t > t0} | set(t_obs.tolist()))
        x = np.array([*x0, 1.0], float)
        t, at = t0, {}
        cache = {}
        for te in edges:
            if te > t:
                mid = 0.5 * (t + te)
                i2 = next((q for a, b, q in heat if a <= mid < b), 0.0)
                key = (i2, round(te - t, 9))
                if key not in cache:
                    cache[key] = expm(self._a(i2, r_ref, t_ref, alpha, tb) * (te - t))
                x = cache[key] @ x
                t = te
            at[te] = x[:3].copy()
        out = np.array([at[float(tt)] for tt in t_obs])
        return out[:, 0], out[:, 1], out[:, 2]
