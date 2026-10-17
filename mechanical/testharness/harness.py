"""Rev 2A harness. X east, Y north, Z up; print underside on the bed."""
from dataclasses import dataclass

from build123d import Align, Box, Cylinder, Part, Pos, RectangleRounded, chamfer, extrude


@dataclass(frozen=True)
class Spec:
    count: int = 6
    columns: int = 3
    row_gap: float = 14.0
    # PCB source: hardware/boards/osc-dev-v006/osc-dev-v006.kicad_pcb.
    board_w: float = 40.0
    board_l: float = 50.0
    board_r: float = 4.0
    hole_x: float = 16.0
    hole_y: float = 21.0
    board_clear: float = 0.15
    board_t: float = 1.6
    board_recess: float = 1.6
    underside_clear: float = 2.8
    screw_pilot_d: float = 1.8
    screw_floor: float = 0.8
    mount_pad_r: float = 2.8
    # Nominal housing: https://www.vorpalrobotics.com/wiki/index.php/Vorpal_MG90_Micro_Servo
    servo_w: float = 12.2
    servo_l: float = 22.8
    servo_clear: float = 0.05  # stenciljig/common.py FIT_TIGHT, per side
    servo_depth: float = 6.0
    wire_w: float = 7.0
    wire_chamfer: float = 0.6
    pair_gap: float = 10.0
    web: float = 3.2
    border: float = 4.0
    floor: float = 2.4

    @property
    def station_w(self):
        return self.servo_w + 2 * self.servo_clear + self.pair_gap + self.board_w + 2 * self.board_clear

    @property
    def pitch(self):
        return self.station_w + self.web

    @property
    def rows(self):
        return (self.count + self.columns - 1) // self.columns

    @property
    def row_pitch(self):
        return self.board_l + 2 * self.board_clear + self.row_gap

    @property
    def width(self):
        return min(self.count, self.columns) * self.pitch - self.web + 2 * self.border

    @property
    def south(self):
        return -(self.rows - 1) * self.row_pitch / 2 - self.board_l / 2 - self.board_clear - self.border

    @property
    def north(self):
        return -self.south

    def station(self, index):
        row, column = divmod(index, self.columns)
        occupied = min(self.columns, self.count - row * self.columns)
        center = (column - (occupied - 1) / 2) * self.pitch
        board_x = center + (self.servo_w + 2 * self.servo_clear + self.pair_gap) / 2
        servo_x = center - (self.board_w + 2 * self.board_clear + self.pair_gap) / 2
        y = ((self.rows - 1) / 2 - row) * self.row_pitch
        return board_x, servo_x, y

    @property
    def top(self):
        return self.floor + max(self.servo_depth, self.board_recess + self.underside_clear)

    @property
    def seat(self):
        return self.top - self.board_recess


def box(x, y, z, w, l, h):
    return Pos(x, y, z) * Box(w, l, h, align=(Align.CENTER, Align.CENTER, Align.MIN))


def build(spec: Spec = Spec()) -> Part:
    s = spec
    if s.count < 1 or s.columns < 1 or min(s.servo_w, s.servo_l, s.servo_depth) <= 0:
        raise ValueError("Count and servo dimensions must be positive")
    if s.board_recess < s.board_t:
        raise ValueError("Board recess must contain the PCB thickness")
    if not 0 < s.screw_floor < s.floor:
        raise ValueError("Screw holes must leave a floor below the mounting pads")
    if not 0 < s.screw_pilot_d < 2 * s.mount_pad_r:
        raise ValueError("Screw pilots must fit within the mounting pads")
    if s.servo_clear < 0 or s.wire_w >= s.servo_w or s.wire_w <= 0:
        raise ValueError("Wire slot must leave both servo side walls intact")
    if not 0 < s.wire_chamfer < min(s.servo_depth, (s.servo_w - s.wire_w) / 2):
        raise ValueError("Wire chamfer must leave the servo end walls intact")
    if s.width > 250 or s.north - s.south > 210:
        raise ValueError("Harness exceeds the MK3S 250 x 210 mm bed")
    if s.pair_gap < s.web or s.row_gap < s.border:
        raise ValueError("Stations must leave clearance between recesses")
    if s.servo_l + 2 * s.servo_clear > s.board_l + 2 * s.board_clear:
        raise ValueError("Servo pocket must fit within the station length")
    p = Pos(0, (s.north + s.south) / 2) * extrude(
        RectangleRounded(s.width, s.north - s.south, 2.0), s.top)
    for i in range(s.count):
        x, servo_x, y = s.station(i)
        p -= Pos(x, y, s.floor) * extrude(RectangleRounded(
            s.board_w + 2 * s.board_clear, s.board_l + 2 * s.board_clear,
            s.board_r + s.board_clear), s.top - s.floor + 0.1)
        for dx in (-s.hole_x, s.hole_x):
            for dy in (-s.hole_y, s.hole_y):
                p += Pos(x + dx, y + dy, s.floor) * Cylinder(
                    s.mount_pad_r, s.seat - s.floor,
                    align=(Align.CENTER, Align.CENTER, Align.MIN))
                p -= Pos(x + dx, y + dy, s.screw_floor) * Cylinder(
                    s.screw_pilot_d / 2, s.seat - s.screw_floor + 0.1,
                    align=(Align.CENTER, Align.CENTER, Align.MIN))
        p -= box(servo_x, y, s.top - s.servo_depth,
                 s.servo_w + 2 * s.servo_clear,
                 s.servo_l + 2 * s.servo_clear, s.servo_depth + 0.1)
    servo_columns = {s.station(i)[1] for i in range(s.count)}
    for servo_x in servo_columns:
        p -= box(servo_x, (s.south + s.north) / 2, s.top - s.servo_depth,
                 s.wire_w, s.north - s.south + 0.2, s.servo_depth + 0.1)
    wire_edges = [e for e in p.edges()
                  if abs(e.bounding_box().min.Z - s.top) < 1e-6
                  and abs(e.bounding_box().max.Z - s.top) < 1e-6
                  and any(abs(abs(e.center().X - x) - s.wire_w / 2) < 1e-6
                          for x in servo_columns)]
    p = chamfer(wire_edges, s.wire_chamfer)
    p.label = "testharness"
    return p
