"""The wavefit port on synthetic bursts: the interleaved layout, and a winding read off both
terminals with the low-side path left out of R, as osc-ident's
`both_terminals_take_the_low_side_out_of_r` pins it."""

import numpy as np
import pytest

from oscnb import wavefit as wf

LSB = 3.3 / 4096
SC = wf.Scales(LSB / (15 * 0.060), LSB * 10_100 / 3_300, LSB * 2.5, LSB, 0.060)
DIFF = wf.CHAN_VMOTOR_A | wf.CHAN_VMOTOR_B | wf.CHAN_INTERLEAVE
STEP_INDEX, LATCH, ARR, BIAS, VB = 485, 21, 1200, 112.0, 779.0
PLANT = dict(r=4.28, l=0.79e-3, v0=0.12, v_rail=7.2, rds=0.14, r_low=0.0, r_shunt=0.060,
             settle_us=1.29, tap_us=0.33, split=0.0)


def frame(chans):
    # each slot's channel bit, 0 for the shunt
    slots = [0]
    for b in (wf.CHAN_VMOTOR_A, wf.CHAN_VMOTOR_B, wf.CHAN_VBUS):
        if chans & b:
            if chans & wf.CHAN_INTERLEAVE and len(slots) > 1:
                slots.append(0)
            slots.append(b)
    return slots


def capture(index, step_q15, chans, p):
    """osc-ident's SynthBurst on a stiff rail from rest: the bridge explicit, `r_low` of copper
    in each low-side leg, the shunt through its amplifier lag, each tap through its RC."""
    sub, period = 8, 2 * ARR / 52.0     # substeps per conversion; conversions per PWM period
    crest0, d = STEP_INDEX + LATCH, abs(step_q15) / 32767
    dt = wf.SAMPLE_US * 1e-6 / sub
    a_amp = 1 - np.exp(-wf.SAMPLE_US / sub / p["settle_us"])
    a_tap = 1 - np.exp(-wf.SAMPLE_US / sub / p["tap_us"])
    vb_v, low = VB * LSB, p["rds"] + p["r_low"]
    fwd = step_q15 >= 0
    slots = frame(chans)
    rng = np.random.default_rng(index)
    i = 0.0
    m, ta, tb = BIAS, vb_v, vb_v
    out = []
    for n in range(960):
        for s in range(sub):
            x = n + s / sub
            k = round((x - crest0) / period)
            u = x - (crest0 + k * period)
            dk = d if (k if u >= 0 else k - 1) >= 0 else 0.0
            on = abs(u) <= dk * period / 2
            if dk == 0.0:
                hi, lo = vb_v, vb_v
                i_sh = 0.0
            else:
                i_sh = i if on else 0.0
                pgnd = i_sh * p["r_shunt"]
                if on:
                    hi, lo = p["v_rail"] - i_sh * p["r_shunt"] + pgnd - i * p["rds"], pgnd + i * low
                else:
                    hi, lo = -i * low, i * low
                v0 = p["v0"] if i > 0 else 0.0
                i += (hi - lo - i * p["r"] - v0) / p["l"] * dt
                if not on:
                    i = max(i, 0.0)
            m += (BIAS + i_sh / SC.amps_per_count - m) * a_amp
            va, vbv = (hi, lo) if fwd else (lo, hi)
            ta += (va - ta) * a_tap
            tb += (vbv - tb) * a_tap
        code = {0: m,
                wf.CHAN_VMOTOR_A: VB + (ta - vb_v) / SC.v_term_per_count,
                wf.CHAN_VMOTOR_B: VB + (tb - vb_v) / SC.v_term_per_count + p["split"]}[slots[n % len(slots)]]
        out.append(np.clip(round(code + rng.uniform(-1, 1)), 0, 4095))
    meta = dict(step_q15=step_q15, pre_q15=0, seated=0, step_index=STEP_INDEX, frame_len=len(slots),
                chans=chans, pwm_arr=ARR, vbus_raw=round(p["v_rail"] / SC.v_rail_per_count), pos=2048)
    return wf.Capture(index, meta, np.array(out, float))


def captures(p, chans_of, rungs=(25, 40)):
    caps = []
    for pct in rungs:
        for sgn in (1, -1):
            for _ in range(2):
                q = int(sgn * pct * 32767 / 100)
                caps.append(capture(len(caps), q, chans_of(q), p))
    return caps


def driven(q):
    return wf.CHAN_VMOTOR_A if q >= 0 else wf.CHAN_VMOTOR_B


def test_the_interleaved_layout_puts_a_shunt_ahead_of_every_extra():
    meta = dict(chans=DIFF, frame_len=4)
    c = wf.Capture(0, meta, np.arange(960.0))
    assert (c.shunt_stride, c.slot(wf.CHAN_VMOTOR_A), c.slot(wf.CHAN_VMOTOR_B)) == (2, 1, 3)
    assert np.array_equal(c.shunt(), np.arange(0, 960, 2))
    assert np.array_equal(c.stream(3), np.arange(3, 960, 4))
    plain = wf.Capture(0, dict(chans=3, frame_len=3), np.arange(960.0))
    assert (plain.shunt_stride, plain.slot(wf.CHAN_VMOTOR_B)) == (3, 2)


@pytest.fixture(scope="module")
def fits():
    hot = dict(PLANT, rds=0.20, r_low=0.08, split=6.0)
    out = {}
    for name, p in (("nominal", PLANT), ("hot", hot)):
        diff = captures(p, lambda q: DIFF)
        for key, caps, both in ((name, diff, True), (name + " beside", diff, False),
                                (name + " driven", captures(p, driven), True)):
            r = wf.Run.build(caps, SC, ke=0.0, both=both)
            out[key] = (r, r.fit_like_ident()[0])
    return out


def test_both_terminals_take_the_low_side_out_of_r(fits):
    run_hot, hot = fits["hot"]
    _, nominal = fits["nominal"]
    assert run_hot.both_terminals and len(run_hot.prepared) == 8
    assert hot.p["r"] == pytest.approx(nominal.p["r"], rel=0.01)
    assert hot.p["r"] == pytest.approx(PLANT["r"], rel=0.015)
    assert hot.p["l_mh"] == pytest.approx(PLANT["l"] * 1e3, rel=0.015)
    assert hot.p["v0"] == pytest.approx(PLANT["v0"], abs=0.025)
    on_drop, off, lo = run_hot.bridge(hot.p)
    assert on_drop == 0.0
    assert off == pytest.approx(2 * 0.28, rel=0.03)
    assert lo == pytest.approx(0.28 + 0.06, rel=0.03)
    # the driven terminal alone reads the low side it assumes wrong into R, beside the
    # difference on the same captures and on the plant sampled shunt, A
    for alone in (" beside", " driven"):
        assert not fits["hot" + alone][0].both_terminals
        assert fits["hot" + alone][1].p["r"] - fits["nominal" + alone][1].p["r"] > 0.15
