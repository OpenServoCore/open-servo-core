"""Steady-state validation of the chip's autonomous HSI trim.

The chip owns its clock trim: it watches inter-byte timing on every non-Status
packet (`Ping`, `Read`, `Write`, …), batches the drift, and nudges HSITRIM
toward zero autonomously (`firmware/lib/drivers/src/dxl/uart/clock.rs`). There
is no master CAL and no readable trim register — the host only observes how long
the chip takes to transmit a reply, via the pirate's stable-clock `Round` stamp
(`drift.probe_drift_ppm`).

This test asserts the chip *holds* lock under sustained Ping load: after the
session warm-up has converged the trim, a fresh burst of Pings must leave the
measured drift inside one trim step plus host-probe noise. The cold-start
convergence curve (which needs a reboot to factory drift) lives in
`scripts/pirate_chip_ping_convergence.py`.
"""

from drift import DRIFT_STEP_PPM, probe_drift_ppm
from dxl_packet import build_ping

# Pings to drive before probing — a couple of steady batches (~3.3 pings each)
# on top of the session warm-up.
LOAD_PINGS = 60
# Probe averaging: more samples than the default to tighten the host-side
# estimate against per-byte resync quantization.
PROBE_SAMPLES = 16
# Steady deadband is a half-step and the integrator may sit anywhere inside it;
# add a full step of host-probe + EMA slack.
DRIFT_BOUND_PPM = 2 * DRIFT_STEP_PPM


def test_steady_state_drift_bounded(pirate, osc_id, baud):
    for _ in range(LOAD_PINGS):
        pirate.xfer(build_ping(osc_id), reply_us=200_000)

    drift = probe_drift_ppm(pirate, osc_id, baud, samples=PROBE_SAMPLES)
    print(f"\n  steady-state drift = {drift:+.0f} ppm  (bound ±{DRIFT_BOUND_PPM})",
          flush=True)
    assert abs(drift) <= DRIFT_BOUND_PPM, (
        f"chip drift {drift:+.0f} ppm exceeds ±{DRIFT_BOUND_PPM} under Ping load "
        f"— autonomous trim not holding lock"
    )
