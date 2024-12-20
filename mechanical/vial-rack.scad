function offset(edge_spacing, spacing, hole_diameter, col, row) = [
  edge_spacing.x + hole_diameter / 2 + (hole_diameter + spacing.x) * col,
  edge_spacing.y + hole_diameter / 2 + (hole_diameter + spacing.y) * row,
];

module vial_rack_leg(leg_size, leg_thickness) {
  difference() {
    cube([leg_size.x, leg_size.y, leg_size.z]);
    translate([leg_thickness, leg_thickness, -1]) cube([leg_size.x - leg_thickness + 1, leg_size.y - leg_thickness + 1, leg_size.z + 2]);
  }
}

module vial_rack(
  hole_diameter = 15,
  thickness = 8,
  grid = [10, 5],
  spacing = [10, 10],
  edge_spacing = [15, 15],
  leg_size = [15, 15, 35],
  leg_thickness = 5,
) {
  length = (hole_diameter + spacing.x) * grid.x - spacing.x + edge_spacing.x * 2;
  width = (hole_diameter + spacing.y) * grid.y - spacing.y + edge_spacing.y * 2;

  translate([-length/2, -width/2]) {
    union() {
      // rack top
      difference() {
        cube([length, width, thickness]);

        union() {
          for (x = [0:grid.x - 1]) {
            for (y = [0:grid.y - 1]) {
              pos = offset(edge_spacing, spacing, hole_diameter, x, y);
              translate([pos.x, pos.y, -1]) {
                cylinder(h=thickness + 2, d=hole_diameter);
              }
            }
          }
        }
      }

      // rack leg lower left
      translate([0, 0, thickness]) {
        rotate(0) {
          vial_rack_leg(leg_size, leg_thickness);
        }
      }

      // rack leg lower right
      translate([length, 0, thickness]) {
        rotate(90) {
          vial_rack_leg(leg_size, leg_thickness);
        }
      }

      // rack leg upper right
      translate([length, width, thickness]) {
        rotate(180) {
          vial_rack_leg(leg_size, leg_thickness);
        }
      }

      // rack leg upper left
      translate([0, width, thickness]) {
        rotate(270) {
          vial_rack_leg(leg_size, leg_thickness);
        }
      }
    }
  }
}

vial_rack();
