include <./lib/gears.scad>

difference() {
    $fn = 128;
    cylinder(3, 10, 10);

    translate([0,0,0.5]) {
        spur_gear(0.23, 10, 3, 0);
    }
}