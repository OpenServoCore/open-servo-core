"""Synthetic frames for the fiducial pipeline: tags on two parallel planes,
seen by a tilted pinhole camera with lens distortion, rendered with
supersampling, blur and sensor noise. The truth is known exactly, so the
pipeline in `fiducial` can be graded against it.

World frame: page coordinates in mm, x right, y down, the reference plane at
z = 0 and the camera in front of it at negative z looking along +z. The horn
plane is parallel to the reference plane, height_mm closer to the camera.
"""

from dataclasses import dataclass, field

import cv2
import numpy as np

from . import fiducial as fd

PAPER, INK, BACKGROUND = 225.0, 30.0, 95.0


@dataclass(frozen=True)
class SynthCamera:
    size: tuple = (1600, 1200)
    f: float = 1500.0
    dist: tuple = (-0.11, 0.07, 0.0006, -0.0004, 0.0)
    # small skew of the principal point off centre, as a real lens has
    c_off: tuple = (14.0, -9.0)

    @property
    def K(self):
        w, h = self.size
        return np.array([[self.f, 0, (w - 1) / 2 + self.c_off[0]],
                         [0, self.f, (h - 1) / 2 + self.c_off[1]], [0, 0, 1.0]])

    def camera(self):
        return fd.Camera(self.K, np.array(self.dist, float), self.size)


def pose(tilt_deg=(6.0, -4.0), roll_deg=3.0, distance_mm=230.0, look_at=(0.0, 0.0)):
    """Camera rotation and translation (world -> camera) for a camera
    distance_mm from look_at on the reference plane, tilted about x then y."""
    rx, ry, rz = np.radians([tilt_deg[0], tilt_deg[1], roll_deg])
    Rx = np.array([[1, 0, 0], [0, np.cos(rx), -np.sin(rx)], [0, np.sin(rx), np.cos(rx)]])
    Ry = np.array([[np.cos(ry), 0, np.sin(ry)], [0, 1, 0], [-np.sin(ry), 0, np.cos(ry)]])
    Rz = np.array([[np.cos(rz), -np.sin(rz), 0], [np.sin(rz), np.cos(rz), 0], [0, 0, 1]])
    R = Rz @ Ry @ Rx
    target = np.array([look_at[0], look_at[1], 0.0])
    C = target - distance_mm * (R.T @ np.array([0, 0, 1.0]))
    return R, -R @ C


def tag_texture(tag_id, dictionary=None, px_per_mm=40, side=fd.TAG_MM, quiet_cells=1.5):
    """A tag with its white quiet zone, as a float image, px_per_mm, and the
    half width of the textured square in mm (tag centre at the image centre)."""
    d = dictionary or fd.dictionary()
    cells = d.markerSize + 2
    bits = cv2.aruco.generateImageMarker(d, tag_id, cells, borderBits=1)
    cell_px = int(round(side / cells * px_per_mm))
    px_per_mm = cell_px * cells / side
    img = np.kron(bits, np.ones((cell_px, cell_px))).astype(np.float32)
    q = int(round(quiet_cells * cell_px))
    img = np.pad(img, q, constant_values=255)
    tex = INK + (PAPER - INK) * img / 255.0
    return tex.astype(np.float32), px_per_mm, img.shape[0] / px_per_mm / 2


@dataclass
class Plane:
    """A textured square on a plane z = z_mm: texture centre at centre_mm,
    turned angle_deg counterclockwise as seen, page scale (kx, ky)."""
    texture: np.ndarray
    px_per_mm: float
    half_mm: float
    centre_mm: tuple
    z_mm: float = 0.0
    angle_deg: float = 0.0
    k: tuple = (1.0, 1.0)


@dataclass
class Renderer:
    cam: SynthCamera = field(default_factory=SynthCamera)
    ss: int = 2
    grid_px: int = 8

    def __post_init__(self):
        w, h = self.cam.size
        W, H = w * self.ss, h * self.ss
        # undistort a coarse lattice and interpolate: the distortion field is
        # smooth, and a full-resolution iterative inverse is slow
        gx = np.arange(0, W + self.grid_px, self.grid_px, dtype=np.float64)
        gy = np.arange(0, H + self.grid_px, self.grid_px, dtype=np.float64)
        u, v = np.meshgrid((gx + 0.5) / self.ss - 0.5, (gy + 0.5) / self.ss - 0.5)
        pts = np.stack([u, v], -1).reshape(-1, 1, 2)
        n = cv2.undistortPoints(pts, self.cam.K, np.array(self.cam.dist), criteria=fd.UNDISTORT_CRITERIA)
        n = n.reshape(len(gy), len(gx), 2)
        fx = (np.arange(W) / self.grid_px).astype(np.float32)
        fy = (np.arange(H) / self.grid_px).astype(np.float32)
        mx, my = np.meshgrid(fx, fy)
        self._nx = cv2.remap(n[..., 0].astype(np.float32), mx, my, cv2.INTER_LINEAR).astype(np.float64)
        self._ny = cv2.remap(n[..., 1].astype(np.float32), mx, my, cv2.INTER_LINEAR).astype(np.float64)

    def render(self, planes, R, t, blur_px=0.6, noise=1.5, seed=0):
        """uint8 grayscale frame; planes nearer the camera (lower z) cover
        the ones behind."""
        Rt = R.T
        C = -Rt @ t
        d = np.stack([self._nx, self._ny, np.ones_like(self._nx)], -1) @ Rt.T
        img = np.full(self._nx.shape, BACKGROUND, np.float32)
        done = np.zeros(self._nx.shape, bool)
        for p in sorted(planes, key=lambda p: p.z_mm):
            lam = (p.z_mm - C[2]) / d[..., 2]
            X = C[0] + lam * d[..., 0] - p.centre_mm[0]
            Y = C[1] + lam * d[..., 1] - p.centre_mm[1]
            # undo the visual counterclockwise turn (a math rotation by -a in
            # y-down coordinates), then the printer scale
            a = np.radians(p.angle_deg)
            qx = (np.cos(a) * X - np.sin(a) * Y) / p.k[0]
            qy = (np.sin(a) * X + np.cos(a) * Y) / p.k[1]
            inside = (np.abs(qx) < p.half_mm) & (np.abs(qy) < p.half_mm) & ~done
            tx = ((qx + p.half_mm) * p.px_per_mm - 0.5).astype(np.float32)
            ty = ((qy + p.half_mm) * p.px_per_mm - 0.5).astype(np.float32)
            if not inside.any():
                continue
            # point samples of a sharp edge alias into a staircase that biases
            # the edge fit, so the texture is prefiltered to the sample spacing
            # and a point sample stands in for the area a sensor pixel collects
            spacing = np.median(np.hypot(np.diff(tx, axis=1), np.diff(ty, axis=1))[inside[:, 1:]])
            tex = cv2.GaussianBlur(p.texture, (0, 0), max(0.5 * spacing, 0.3))
            s = cv2.remap(tex, tx, ty, cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
            img[inside] = s[inside]
            done |= inside
        w, h = self.cam.size
        out = cv2.resize(img, (w, h), interpolation=cv2.INTER_AREA)
        if blur_px > 0:
            out = cv2.GaussianBlur(out, (0, 0), blur_px)
        out = out + np.random.default_rng(seed).normal(0, noise, out.shape)
        return np.clip(np.round(out), 0, 255).astype(np.uint8)


def tag_scene(horn_deg, k=(1.0, 1.0), height_mm=9.0, horn_at=(0.0, 0.0), ref_at=(-46.0, 4.0), ref_deg=0.0):
    """The bench layout: the horn tag over the shaft, the reference tag on
    the servo body beside it, the horn height_mm above the body."""
    tex_h, ppm_h, half_h = tag_texture(fd.HORN_ID)
    tex_r, ppm_r, half_r = tag_texture(fd.REF_ID)
    return [Plane(tex_h, ppm_h, half_h, horn_at, -height_mm, horn_deg, k),
            Plane(tex_r, ppm_r, half_r, ref_at, 0.0, ref_deg, k)]


def board_scene(px_per_mm=12, k=(1.0, 1.0)):
    """The ChArUco calibration board as one plane, centred on the origin."""
    b = fd.charuco_board()
    cols, rows = fd.BOARD_SQUARES
    w_mm, h_mm = cols * fd.BOARD_SQUARE_MM, rows * fd.BOARD_SQUARE_MM
    img = b.generateImage((int(w_mm * px_per_mm), int(h_mm * px_per_mm)), marginSize=0, borderBits=1)
    pad = int(10 * px_per_mm)
    img = np.pad(img, pad, constant_values=255).astype(np.float32)
    tex = INK + (PAPER - INK) * img / 255.0
    # the board is not square; pad the short side so one half width covers it
    side = max(tex.shape)
    tex = np.pad(tex, ((0, side - tex.shape[0]), (0, side - tex.shape[1])), constant_values=PAPER)
    half = side / px_per_mm / 2
    # board corner (0, 0) sits pad mm in from the texture's corner; shift so
    # the board's own centre lands on the origin
    centre = (half - (pad / px_per_mm + w_mm / 2), half - (pad / px_per_mm + h_mm / 2))
    return Plane(tex.astype(np.float32), px_per_mm, half, centre, 0.0, 0.0, k)
