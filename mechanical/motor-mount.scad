module arc(start, end, angle, dir = "cw") {
  // find the distance of start and end vectors.
  x = end.x - start.x; // 3
  y = end.y - start.y; // 4
  d = sqrt(x * x + y * y); // 5
  rotation = atan2(y, x);

  opposite = d / 2;
  hypotenuse = opposite / sin(angle / 2);
  adjacent = cos(angle / 2) * hypotenuse;

  echo(hypotenuse, opposite, adjacent, rotation);

  translate([x/2 + start.x, y/2 + start.y, 0])
  rotate(dir == "cw" ? rotation : -rotation, [0, 0, 1])
  translate([0, - adjacent, 0])
  difference() {
    circle(hypotenuse, $fn=100);
    translate([0, adjacent - hypotenuse, 0]) 
      square(hypotenuse*2, true);
  }


    // translate([-opposite*sin(rotation), -opposite*cos(rotation), 0])
    // rotate(-rotation, [0, 0, 1])
    //   square(hypotenuse*2, true);
}

module outline(r, delta, chamfer) {
  difference() {
    offset(r=r, delta=delta, chamfer=chamfer) children();
    children();
  }
}

module motor_mount(length=6, width=8, diameter=10, thickness=2) {

    // intersection() {
      square([length, width], true);
      circle(d=diameter);
    // }

}

motor_mount();

// arc([3, 2], [3, -2], 80);
