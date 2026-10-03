"""The fiducial pipeline against synthetic frames whose truth is known:
tilted camera, lens distortion, the horn plane above the reference plane, and
a printer that scales X and Y differently."""

import numpy as np
import pytest

from oscnb import fiducial as fd
from oscnb import fiducial_synth as sy

TOL_DEG = 0.02
PRINT_K = (1.004, 0.996)
SCALE = fd.PrintScale(fd.CHECK_MM * PRINT_K[0], fd.CHECK_MM * PRINT_K[1])


@pytest.fixture(scope="module")
def renderer():
    return sy.Renderer()


def measure(img, cam, scale=SCALE):
    found = fd.detect(img)
    horn, _ = fd.refine(img, found[fd.HORN_ID], cam)
    ref, _ = fd.refine(img, found[fd.REF_ID], cam)
    return fd.horn_angle(horn, ref, scale)


def test_similarity_recovers_a_known_turn():
    model = fd.tag_model(SCALE)
    for deg in (-170.0, -3.25, 0.0, 0.01, 47.0, 179.0):
        a = np.radians(deg)
        # counterclockwise as seen with y down is a math rotation by -a
        rot = np.array([[np.cos(a), np.sin(a)], [-np.sin(a), np.cos(a)]])
        pts = 1.7 * model @ rot.T + [12.0, -4.0]
        got, scale, resid = fd.similarity(model, pts)
        assert got == pytest.approx(deg, abs=1e-9)
        assert scale == pytest.approx(1.7)
        assert resid < 1e-9


def test_camera_round_trips_through_json():
    cam = sy.SynthCamera().camera()
    back = fd.Camera.from_json(cam.to_json())
    np.testing.assert_allclose(back.K, cam.K)
    np.testing.assert_allclose(back.dist, cam.dist)
    assert back.size == cam.size


@pytest.mark.parametrize("deg", [0.0, 0.37, -12.5, 33.3, 90.0, 141.7, -178.6])
def test_synthetic_angle_within_tolerance(renderer, deg):
    R, t = sy.pose()
    img = renderer.render(sy.tag_scene(deg, k=PRINT_K), R, t, seed=int(abs(deg) * 100))
    got, _, resid_mm = measure(img, renderer.cam.camera())
    assert got - deg == pytest.approx(0.0, abs=TOL_DEG)
    assert resid_mm < 0.01


def test_steeper_tilt_and_higher_horn(renderer):
    R, t = sy.pose(tilt_deg=(14.0, -10.0), roll_deg=-21.0, distance_mm=210.0)
    for i, deg in enumerate((-61.0, 8.8, 122.4)):
        img = renderer.render(sy.tag_scene(deg, k=PRINT_K, height_mm=16.0), R, t, seed=i)
        got, _, _ = measure(img, renderer.cam.camera())
        assert got - deg == pytest.approx(0.0, abs=TOL_DEG)


def test_blurred_noisy_frames(renderer):
    # out of focus or moving: a 3 px blur
    R, t = sy.pose()
    for i, deg in enumerate((-30.0, 37.0, 133.0)):
        img = renderer.render(sy.tag_scene(deg, k=PRINT_K), R, t, seed=20 + i, blur_px=3.0)
        got, _, _ = measure(img, renderer.cam.camera())
        assert got - deg == pytest.approx(0.0, abs=TOL_DEG)


def test_print_aspect_is_second_order():
    # exact corners, no rendering: both tags printed with an X/Y mismatch e
    # and seen through a tilted camera; read with the nominal model the
    # angle is off by at most ~e^2 rad, read with the caliper values by ~0 (float32 homography)
    H = np.array([[1.9, 0.21, 400.0], [-0.17, 2.05, 300.0], [2e-4, -3e-4, 1.0]])
    for e in (0.002, 0.005, 0.01):
        sc = fd.PrintScale(50 * (1 + e), 50 * (1 - e))
        ref = fd.to_plane(H, fd.tag_model(sc) + [-46.0, 4.0])
        worst_nominal = worst_corrected = 0.0
        for deg in np.arange(-180, 180, 7.5):
            a = np.radians(deg)
            rot = np.array([[np.cos(a), np.sin(a)], [-np.sin(a), np.cos(a)]])
            horn = fd.to_plane(H, fd.tag_model(sc) @ rot.T)
            nominal = fd.horn_angle(horn, ref)[0]
            corrected = fd.horn_angle(horn, ref, sc)[0]
            worst_nominal = max(worst_nominal, abs((nominal - deg + 180) % 360 - 180))
            worst_corrected = max(worst_corrected, abs((corrected - deg + 180) % 360 - 180))
        assert worst_nominal < 1.2 * np.degrees(e * e)
        assert worst_nominal > 0.3 * np.degrees(e * e)
        assert worst_corrected < 1e-4


def test_calibrated_camera_carries_the_angle(renderer):
    poses = [((0, 0), 0, 330), ((22, 0), 5, 340), ((-22, 0), -5, 340), ((0, 24), 90, 330),
             ((0, -24), 85, 330), ((15, 15), 30, 300), ((-15, 18), -25, 310), ((18, -15), 60, 320)]
    board = sy.board_scene()
    rng = np.random.default_rng(7)
    views = [renderer.render([board], *sy.pose(tilt, roll, dist, look_at=rng.uniform(-30, 30, 2)), seed=i)
             for i, (tilt, roll, dist) in enumerate(poses)]
    cam = fd.calibrate(views)
    assert cam.views == len(poses)
    assert cam.rms_px < 0.1
    np.testing.assert_allclose(np.diag(cam.K)[:2], renderer.cam.f, rtol=2e-3)
    R, t = sy.pose()
    for i, deg in enumerate((-44.4, 17.2, 61.0)):
        img = renderer.render(sy.tag_scene(deg, k=PRINT_K), R, t, seed=10 + i)
        got, _, _ = measure(img, cam)
        assert got - deg == pytest.approx(0.0, abs=TOL_DEG)


def test_track_and_angles_unwrap_past_half_turn(renderer):
    truth = np.array([172.0, 178.5, 183.0, 190.25])
    R, t = sy.pose()
    frames = ((i, i / 30, renderer.render(sy.tag_scene(d, k=PRINT_K), R, t, seed=i)) for i, d in enumerate(truth))
    df = fd.angles(fd.track(frames, renderer.cam.camera()), SCALE)
    got = df["angle_deg"].to_numpy().copy()
    got += 360 * np.round((truth[0] - got[0]) / 360)
    np.testing.assert_allclose(got, truth, atol=TOL_DEG)
    assert (df["ref_motion_px"] < 0.05).all()


def test_frames_missing_a_tag_stay_nan():
    blank = np.full((240, 320), 200, np.uint8)
    df = fd.angles(fd.track([(0, 0.0, blank)], sy.SynthCamera().camera()))
    assert np.isnan(df["angle_deg"]).all()


def test_lamp_bursts_fit_the_clock():
    fps, offset, rate = 59.94, 3.217, 1.0002
    log_on = np.array([1.0, 1.8, 2.6, 61.0, 61.8, 62.6])
    t = np.arange(0, 70, 1 / fps)
    lit = np.zeros_like(t, bool)
    for on in offset + rate * log_on:
        lit |= (t >= on) & (t < on + 0.4)
    frames = []
    for i, ti in enumerate(t):
        img = np.full((20, 20, 3), 40, np.uint8)
        if lit[i]:
            img[5:15, 5:15, 0] = 160
        frames.append((i, ti, img))
    trace = fd.lamp_trace(frames, (0, 0, 20, 20))
    b = fd.bursts(trace["t"], trace["level"])
    assert len(b) == len(log_on)
    off, r, rms = fd.fit_clock(b["on"], log_on)
    assert off == pytest.approx(offset, abs=1.5 / fps)
    assert r == pytest.approx(rate, abs=1e-4)
    assert rms < 1 / fps


def test_align_by_motion_finds_the_shift():
    t_log = np.arange(0, 40, 0.04)
    pos = 600 + 200 * np.floor(t_log / 2.5)
    t_video = np.arange(0, 45, 1 / 60)
    angle = np.interp(t_video - 1.37, t_log, pos) / 19.0
    off, corr = fd.align_by_motion(t_video, angle, t_log, pos)
    assert off == pytest.approx(1.37, abs=0.02)
    assert corr > 0.9
