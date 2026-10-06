"""The thermal helpers against answers known in closed form or by brute force."""

import numpy as np
import pytest

from oscnb import thermal as th

NET = th.TwoNode(c_w=1.5, c_c=9.0, r_wc=40.0, r_ca=12.0)


def euler(net, t0, x0, heat, t_end, r_ref, t_ref, alpha, tb, dt=0.01):
    tw, tc, ta = x0
    for n in range(int(round((t_end - t0) / dt))):
        t = t0 + n * dt
        i2 = next((q for a, b, q in heat if a <= t + dt / 2 < b), 0.0)
        p = i2 * r_ref * (1 + alpha * (tw - t_ref))
        q = (tw - tc) / net.r_wc
        tw, tc, ta = (tw + dt * (net.share * p - q) / net.c_w,
                      tc + dt * ((1 - net.share) * p + q - (tc - ta) / net.r_ca) / net.c_c,
                      ta + dt * tb)
    return tw, tc, ta


def test_ntc_reads_25_c_at_mid_scale():
    assert th.ntc_c(th.ADC_FULL / 2) == pytest.approx(25.0)
    assert th.ntc_c(1500) > th.ntc_c(2000)


def test_copper_line_reads_the_handbook_coefficient():
    assert th.copper_pct_per_c(20.0) == pytest.approx(100 / 254.5)
    assert th.copper_carry(5.09, 61.5, 28.6) == pytest.approx(5.09 * 263.1 / 296.0)
    r = th.copper_carry(4.52, 28.6, 61.5)
    assert th.thermometer_c(r, 4.52, 28.6) == pytest.approx(61.5)
    assert th.thermometer_c(4.52, 4.52, 28.6) == pytest.approx(28.6)


def test_modes_are_the_roots_of_the_network():
    fast, slow = NET.modes()
    a = 1 / (NET.r_wc * NET.c_w)
    b = 1 / (NET.r_wc * NET.c_c)
    c = 1 / (NET.r_ca * NET.c_c)
    assert fast < slow
    assert 1 / fast + 1 / slow == pytest.approx(a + b + c)
    assert 1 / (fast * slow) == pytest.approx(a * c)


def test_a_long_steady_heat_settles_on_the_resistances():
    p, r_ref = 0.8, 5.0
    tw, tc, ta = NET.run(0.0, (25.0, 25.0, 25.0), [(0.0, 1e5, p / r_ref)], [1e5 - 1],
                         r_ref, 25.0, alpha=0.0)
    assert tw[0] - ta[0] == pytest.approx(p * (NET.r_wc + NET.r_ca), rel=1e-6)
    assert tc[0] - ta[0] == pytest.approx(p * NET.r_ca, rel=1e-6)


def test_heat_on_the_can_leaves_the_winding_at_the_can():
    net = th.TwoNode(NET.c_w, NET.c_c, NET.r_wc, NET.r_ca, share=0.0)
    tw, tc, ta = net.run(0.0, (25.0, 25.0, 25.0), [(0.0, 1e5, 0.16)], [1e5 - 1], 5.0, 25.0, alpha=0.0)
    assert tw[0] == pytest.approx(tc[0], abs=1e-9)
    assert tc[0] - ta[0] == pytest.approx(0.8 * NET.r_ca, rel=1e-6)


@pytest.mark.parametrize("share", [1.0, 0.3])
def test_steps_match_a_fine_euler_run(share):
    net = th.TwoNode(NET.c_w, NET.c_c, NET.r_wc, NET.r_ca, share)
    heat = [(10.0, 110.0, 0.18), (130.0, 230.0, 0.17)]
    x0, args = (30.0, 28.0, 27.0), dict(r_ref=5.0, t_ref=26.0, alpha=th.ALPHA_CU, tb=-8e-4)
    t_obs = np.array([5.0, 60.0, 115.0, 230.0, 400.0])
    tw, tc, ta = net.run(0.0, x0, heat, t_obs, **args)
    for j, t in enumerate(t_obs):
        ew, ec, ea = euler(net, 0.0, x0, heat, t, **args)
        assert tw[j] == pytest.approx(ew, abs=2e-3)
        assert tc[j] == pytest.approx(ec, abs=2e-3)
        assert ta[j] == pytest.approx(ea, abs=1e-9)


def test_observations_keep_the_callers_order():
    heat = [(0.0, 50.0, 0.2)]
    a = NET.run(0.0, (25.0, 25.0, 25.0), heat, [10.0, 40.0, 80.0], 5.0, 25.0)
    b = NET.run(0.0, (25.0, 25.0, 25.0), heat, [80.0, 10.0, 40.0], 5.0, 25.0)
    assert np.allclose(a[0], b[0][[1, 2, 0]])


def test_an_observation_before_the_start_is_refused():
    with pytest.raises(ValueError):
        NET.run(10.0, (25.0, 25.0, 25.0), [], [5.0], 5.0, 25.0)


def test_projection_along_an_exact_ridge_flattens_it():
    rng = np.random.default_rng(1)
    v0 = rng.uniform(-0.3, -0.1, 30)
    r = 4.85 - 2.1 * (v0 + 0.19)
    slope, icept, corr = th.ridge(r, v0)
    assert slope == pytest.approx(-2.1)
    assert corr == pytest.approx(-1.0)
    assert np.allclose(th.project(r, v0, -0.19, slope), 4.85)


def test_copper_round_trips():
    t = np.array([20.0, 26.8, 60.0])
    r = th.copper_ohm(t, 4.85, 26.8)
    assert r[1] == pytest.approx(4.85)
    assert r[2] / r[1] - 1 == pytest.approx(th.ALPHA_CU * 33.2)
    assert np.allclose(th.copper_c(r, 4.85, 26.8), t)
