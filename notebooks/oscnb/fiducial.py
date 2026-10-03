"""Output angle from video: two printed ArUco tags, one on the horn and one on
the servo body, filmed straight down the shaft axis.

The pipeline, per frame:

  1. find both tags (OpenCV ArUco, DICT_4X4_50, horn id 1, reference id 2)
  2. refine every tag edge to a straight line through sub-pixel gradient
     peaks, in undistorted pixels, and take the corners as line crossings
  3. map the image onto the reference tag's plane with the homography of its
     four corners, which removes the camera tilt
  4. fit the horn tag's corners in that plane with a rotation, a uniform scale
     and a shift; the rotation is the horn angle relative to the reference

Why the horn plane can sit above the reference plane: a central projection
between two PARALLEL planes is a similarity (a uniform scale about the foot of
the camera), so it keeps angles. Both tags have to lie flat on planes square to
the shaft, and the free uniform scale in step 4 absorbs the height difference.

The printer's X and Y scale enter through `PrintScale`, one per tag: the tag on
paper is a rectangle kx * 25 by ky * 25 mm in its own frame, and a horn tag
that turns carries that rectangle with it, so the model corners are scaled
before the fit. The angle is insensitive to it to first order: a symmetric
stretch has no rotation in it, and the stretch the reference homography puts
on the plane meets the horn's own as a conjugate, which leaves the rotation
alone up to the square of the mismatch (under 0.002 deg for a 0.5% aspect
error, `test_print_aspect_is_second_order`). Shear from a skewed paper feed
behaves the same way. The caliper values are carried anyway because they cost
nothing and they prove the print was at 100%.

Angles are in degrees, positive counterclockwise as the camera sees the tags.
Pixel coordinates follow OpenCV: pixel centres on integers, y down.

DICT_4X4_50 over AprilTag 36h11: a 4x4 marker is 6 cells across against 10, so
at the same printed size its cells are 1.7x larger, which keeps detection solid
in the 1080p slow-motion frames. Only two known ids are ever in frame, so the
larger code distance of 36h11 buys nothing here. The angle itself comes from
the outer black square, which both families have.
"""

import json
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import pandas as pd

DICT = cv2.aruco.DICT_4X4_50
HORN_ID = 1
REF_ID = 2
# outer edge of the black square, nominal
TAG_MM = 25.0
# the caliper check square printed beside the tags
CHECK_MM = 50.0

# camera calibration board: a different dictionary so a board marker is never
# taken for a tag
BOARD_DICT = cv2.aruco.DICT_5X5_100
BOARD_SQUARES = (7, 10)
BOARD_SQUARE_MM = 25.0
BOARD_MARKER_MM = 18.0

UNDISTORT_CRITERIA = (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 50, 1e-12)


def dictionary():
    return cv2.aruco.getPredefinedDictionary(DICT)


def charuco_board():
    return cv2.aruco.CharucoBoard(BOARD_SQUARES, BOARD_SQUARE_MM, BOARD_MARKER_MM,
                                  cv2.aruco.getPredefinedDictionary(BOARD_DICT))


@dataclass(frozen=True)
class PrintScale:
    """The printer's scale from the caliper check: the 50.00 mm square as
    measured along the page's X (across) and Y (down)."""
    x_mm: float = CHECK_MM
    y_mm: float = CHECK_MM

    @property
    def kx(self):
        return self.x_mm / CHECK_MM

    @property
    def ky(self):
        return self.y_mm / CHECK_MM


def tag_model(scale=PrintScale(), side=TAG_MM):
    """The four corners of a printed tag in its own page frame, mm, in ArUco
    order (top-left, top-right, bottom-right, bottom-left), centred."""
    h = side / 2
    q = np.array([[-h, -h], [h, -h], [h, h], [-h, h]])
    return q * [scale.kx, scale.ky]


@dataclass
class Camera:
    """Pinhole camera with OpenCV's 5-term distortion, from `calibrate`."""
    K: np.ndarray
    dist: np.ndarray
    size: tuple
    rms_px: float = float("nan")
    views: int = 0

    def undistort(self, pts):
        """Distorted pixels -> ideal pinhole pixels, same K."""
        p = np.asarray(pts, np.float64).reshape(-1, 1, 2)
        u = cv2.undistortPoints(p, self.K, self.dist, R=np.eye(3), P=self.K, criteria=UNDISTORT_CRITERIA)
        return u.reshape(-1, 2)

    def to_json(self):
        return json.dumps({"K": self.K.tolist(), "dist": self.dist.ravel().tolist(), "size": list(self.size),
                           "rms_px": self.rms_px, "views": self.views}, indent=2)

    @classmethod
    def from_json(cls, text):
        d = json.loads(text)
        return cls(np.array(d["K"], float), np.array(d["dist"], float), tuple(d["size"]),
                   d.get("rms_px", float("nan")), d.get("views", 0))

    def save(self, path):
        Path(path).write_text(self.to_json())

    @classmethod
    def load(cls, path):
        return cls.from_json(Path(path).read_text())


def gray(img):
    return img if img.ndim == 2 else cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


def board_points(img, board=None, scale=PrintScale()):
    """(object mm Nx3, image px Nx2) of the ChArUco corners found in one image,
    or None. The board's X and Y take the printer scale like the tags do."""
    board = board or charuco_board()
    cc, ids, _, _ = cv2.aruco.CharucoDetector(board).detectBoard(gray(img))
    if ids is None or len(ids) < 6:
        return None
    obj, pix = board.matchImagePoints(cc, ids)
    obj = obj.reshape(-1, 3) * [scale.kx, scale.ky, 1.0]
    return obj.astype(np.float32), pix.reshape(-1, 2).astype(np.float32)


def calibrate(images, scale=PrintScale(), min_corners=12, flags=0):
    """Camera from a set of board views (arrays). Views with fewer than
    min_corners board corners are skipped. All views must share one size, one
    lens and one locked focus: the one the tag video is shot with."""
    board = charuco_board()
    objs, pixs, size = [], [], None
    for img in images:
        g = gray(img)
        if size is None:
            size = g.shape[::-1]
        elif g.shape[::-1] != size:
            raise ValueError(f"view size {g.shape[::-1]} != {size}")
        bp = board_points(g, board, scale)
        if bp is not None and len(bp[0]) >= min_corners:
            objs.append(bp[0])
            pixs.append(bp[1])
    if len(objs) < 4:
        raise ValueError(f"only {len(objs)} usable board views, need at least 4")
    rms, K, dist, _, _ = cv2.calibrateCamera(objs, pixs, size, None, None, flags=flags)
    return Camera(K, dist.ravel(), tuple(size), float(rms), len(objs))


def detector():
    p = cv2.aruco.DetectorParameters()
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return cv2.aruco.ArucoDetector(dictionary(), p)


def detect(img, det=None):
    """{id: 4x2 distorted corner pixels} for every tag found."""
    corners, ids, _ = (det or detector()).detectMarkers(gray(img))
    if ids is None:
        return {}
    return {int(i): c.reshape(4, 2).astype(np.float64) for i, c in zip(ids.ravel(), corners)}


def _edge_points(g, a, b, reach, step=0.25, span=(0.12, 0.88), every_px=2.0, plateau=3):
    """Edge crossings across the segment a->b, distorted pixels.

    Each crossing is where an ideal step between the edge's paper and ink
    levels would hold the same ink: the area under the normalized profile. A
    gradient peak is the textbook choice, but on a bilinear-sampled profile
    the gradient is flat across each pixel and its peak snaps to the pixel
    grid, about 0.3 px rms on a straight edge. The area is unbiased only on
    a window centred on the edge, so a second pass re-centres on the first."""
    d = b - a
    length = np.hypot(*d)
    n_pts = max(8, int(length * (span[1] - span[0]) / every_px))
    s = np.linspace(span[0], span[1], n_pts)
    nrm = np.array([-d[1], d[0]]) / length
    off = np.arange(-reach, reach + step / 2, step)
    base = a + s[:, None] * d
    ok = np.ones(n_pts, bool)
    for _ in range(2):
        p = base[:, None, :] + off[None, :, None] * nrm
        prof = cv2.remap(g, p[..., 0].astype(np.float32), p[..., 1].astype(np.float32), cv2.INTER_LINEAR)
        lo, hi = prof[:, :plateau].mean(axis=1), prof[:, -plateau:].mean(axis=1)
        ok &= np.abs(lo - hi) > 0.5 * np.median(np.abs(lo - hi))
        # one paper and one ink level per edge: a level error at a single
        # point shifts its crossing by the whole window width times the error
        lo, hi = np.median(lo[ok]), np.median(hi[ok])
        frac = (prof - hi) / (lo - hi)
        o = off[0] + np.trapezoid(frac, off, axis=1)
        ok &= np.abs(o) < reach - plateau * step
        base = base + np.where(ok, o, 0.0)[:, None] * nrm
    return base[ok]


def _fit_line(pts):
    c = pts.mean(axis=0)
    _, _, vt = np.linalg.svd(pts - c)
    n = vt[1]
    return c, n, float(np.sqrt(np.mean(((pts - c) @ n) ** 2)))


def refine(img, corners, cam, reach=3.0):
    """Corners as crossings of the four edge lines, fitted in undistorted
    pixels. Returns (4x2 undistorted corners, line rms px)."""
    g = gray(img).astype(np.float32)
    lines, rms = [], []
    for i in range(4):
        pts = _edge_points(g, corners[i], corners[(i + 1) % 4], reach)
        c, n, r = _fit_line(cam.undistort(pts))
        lines.append((c, n))
        rms.append(r)
    out = np.empty((4, 2))
    for i in range(4):
        (c0, n0), (c1, n1) = lines[i - 1], lines[i]
        out[i] = np.linalg.solve(np.array([n0, n1]), np.array([n0 @ c0, n1 @ c1]))
    return out, float(np.sqrt(np.mean(np.square(rms))))


def plane_homography(ref_px, scale=PrintScale()):
    """Undistorted image pixels -> the reference tag's page plane, mm."""
    return cv2.getPerspectiveTransform(np.asarray(ref_px, np.float32), tag_model(scale).astype(np.float32))


def to_plane(H, pts):
    return cv2.perspectiveTransform(np.asarray(pts, np.float64).reshape(-1, 1, 2), H).reshape(-1, 2)


def similarity(model, pts):
    """Least-squares rotation + uniform scale + shift taking model onto pts:
    (angle deg counterclockwise as seen, scale, residual rms in pts units)."""
    a = model - model.mean(axis=0)
    b = pts - pts.mean(axis=0)
    za = a[:, 0] + 1j * a[:, 1]
    zb = b[:, 0] + 1j * b[:, 1]
    z = np.sum(zb * np.conj(za)) / np.sum(np.abs(za) ** 2)
    resid = zb - z * za
    # y points down, so a positive math angle turns clockwise on screen
    return -np.degrees(np.angle(z)), float(np.abs(z)), float(np.sqrt(np.mean(np.abs(resid) ** 2)))


def horn_angle(horn_px, ref_px, scale=PrintScale(), ref_scale=None):
    """Horn angle from both tags' undistorted corners:
    (angle deg, scale, fit residual mm). ref_scale defaults to scale, both
    tags off one sheet."""
    H = plane_homography(ref_px, ref_scale or scale)
    return similarity(tag_model(scale), to_plane(H, horn_px))


def video_frames(path, start=0, stop=None, step=1):
    """(frame index, container time s, BGR frame) from a video file."""
    cap = cv2.VideoCapture(str(path))
    if not cap.isOpened():
        raise OSError(f"cannot open {path}")
    try:
        cap.set(cv2.CAP_PROP_POS_FRAMES, start)
        i = start
        while stop is None or i < stop:
            ok, frame = cap.read()
            if not ok:
                break
            t = cap.get(cv2.CAP_PROP_POS_MSEC) / 1e3
            if (i - start) % step == 0:
                yield i, t, frame
            i += 1
    finally:
        cap.release()


def image_frames(paths):
    for i, p in enumerate(sorted(paths)):
        img = cv2.imread(str(p))
        if img is None:
            raise OSError(f"cannot read {p}")
        yield i, float("nan"), img


CORNER_COLS = [f"{tag}{ax}{k}" for tag in ("h", "r") for k in range(4) for ax in "xy"]


def track(frames, cam, det=None):
    """One row per frame: refined undistorted corners of both tags, their line
    fit rms, and how large each tag is in pixels. Missing tags leave NaN."""
    det = det or detector()
    rows = []
    for i, t, img in frames:
        g = gray(img)
        found = detect(g, det)
        row = {"frame": i, "t": t}
        for tag, tid in (("h", HORN_ID), ("r", REF_ID)):
            if tid in found:
                c, rms = refine(g, found[tid], cam)
                row[f"{tag}_line_rms_px"] = rms
                row[f"{tag}_side_px"] = float(np.mean(np.hypot(*(c - np.roll(c, -1, axis=0)).T)))
                for k in range(4):
                    row[f"{tag}x{k}"], row[f"{tag}y{k}"] = c[k]
        rows.append(row)
    df = pd.DataFrame(rows)
    for col in CORNER_COLS + ["h_line_rms_px", "r_line_rms_px", "h_side_px", "r_side_px"]:
        if col not in df:
            df[col] = np.nan
    return df


def corners(df, tag):
    return df[[f"{tag}{ax}{k}" for k in range(4) for ax in "xy"]].to_numpy().reshape(-1, 4, 2)


def angles(df, scale=PrintScale(), ref_scale=None, static_ref=True, unwrap=True):
    """Add angle_deg, plane_scale and resid_mm to a `track` table.

    static_ref: the camera is on a tripod and the reference does not move, so
    one homography from the median reference corners serves every frame and
    the reference's own corner noise drops out of the angle."""
    out = df.copy()
    horn, ref = corners(df, "h"), corners(df, "r")
    seen = np.isfinite(ref).all(axis=(1, 2))
    ref_med = np.median(ref[seen], axis=0) if static_ref and seen.any() else np.full((4, 2), np.nan)
    res = np.full((len(df), 3), np.nan)
    for k in range(len(df)):
        r = ref_med if static_ref else ref[k]
        if np.isfinite(horn[k]).all() and np.isfinite(r).all():
            res[k] = horn_angle(horn[k], r, scale, ref_scale)
    a = res[:, 0]
    ok = np.isfinite(a)
    if unwrap and ok.any():
        a[ok] = np.degrees(np.unwrap(np.radians(a[ok])))
    out["angle_deg"], out["plane_scale"], out["resid_mm"] = a, res[:, 1], res[:, 2]
    if static_ref:
        out["ref_motion_px"] = np.sqrt(np.mean(np.sum((ref - ref_med) ** 2, axis=2), axis=1))
    return out


def lamp_trace(frames, roi):
    """Blue-lamp level per frame: mean of (blue - red) over the ROI (x, y, w,
    h, pixels). The DATA lamp lights while the bus line is low, so it glows
    only during traffic."""
    x, y, w, h = roi
    rows = []
    for i, t, img in frames:
        p = img[y:y + h, x:x + w].astype(np.float32)
        rows.append({"frame": i, "t": t, "level": float(np.mean(p[..., 0] - p[..., 2]))})
    return pd.DataFrame(rows)


def bursts(t, level, min_gap_s=0.15, min_len_s=0.05, k=6.0):
    """Stretches where the lamp level stands clear of its resting floor:
    a DataFrame of on/off times. Frames closer than min_gap_s merge; stretches
    shorter than min_len_s are dropped."""
    t, level = np.asarray(t, float), np.asarray(level, float)
    base = np.median(level)
    mad = np.median(np.abs(level - base)) * 1.4826
    thr = base + max(k * mad, 0.25 * (np.percentile(level, 99.5) - base))
    on = np.flatnonzero(level > thr)
    if not len(on):
        return pd.DataFrame(columns=["on", "off"])
    groups = np.split(on, np.flatnonzero(np.diff(t[on]) > min_gap_s) + 1)
    rows = [{"on": t[g[0]], "off": t[g[-1]]} for g in groups]
    df = pd.DataFrame(rows)
    return df[df["off"] - df["on"] >= min_len_s].reset_index(drop=True)


def fit_clock(video_on, log_on):
    """video_t = offset + rate * log_t from matched burst onsets, in order.
    Returns (offset s, rate, rms s)."""
    v, g = np.asarray(video_on, float), np.asarray(log_on, float)
    if len(v) != len(g) or len(v) < 2:
        raise ValueError(f"{len(v)} video bursts against {len(g)} logged: match them by hand")
    rate, offset = np.polyfit(g, v, 1)
    return float(offset), float(rate), float(np.sqrt(np.mean((offset + rate * g - v) ** 2)))


def align_by_motion(t_video, angle, t_log, pos, search_s=5.0, step_s=0.005, rate=1.0):
    """Offset (s) that best lines the logged pot trace up with the video
    angle, video_t = offset + rate * log_t, by the correlation of their first
    differences. A cross-check of the lamp sync, never a replacement."""
    t_video, angle = np.asarray(t_video, float), np.asarray(angle, float)
    t_log, pos = np.asarray(t_log, float), np.asarray(pos, float)
    ok = np.isfinite(angle)
    grid = np.arange(t_video[ok].min(), t_video[ok].max(), step_s)
    va = np.diff(np.interp(grid, t_video[ok], angle[ok]))
    best = (-np.inf, 0.0)
    for off in np.arange(-search_s, search_s + step_s / 2, step_s):
        lp = np.diff(np.interp((grid - off) / rate, t_log, pos))
        if lp.std() == 0:
            continue
        c = abs(np.corrcoef(va, lp)[0, 1])
        if c > best[0]:
            best = (c, off)
    return float(best[1]), float(best[0])
