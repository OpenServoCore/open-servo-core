"""PCB solder-paste stencil jig + tool tote. Independent build123d
implementation of the mechanism from "PCB Stencil Jig for applying
solder paste (parametric, works with double-sided boards)" by
bobstay, https://www.printables.com/model/1209372 - credited with
thanks as the origin of the design idea. Same overall mechanism;
changes for this shop:
- heat-set inserts -> plain M3 nuts in printed pockets (encbench
  benched flat-pocket numbers: 5.7 drop-out, 5.4 broach). Every nut
  SEAT is press fit (FIT_PRESS); deep pockets ride the nut up a
  loose shaft first, and the screw from the far side drags it home
- bobstay's under-base thumbscrews -> T-nut style: socket screws
  drive from the TOP through low ear tabs on the plate's outer foot,
  through the rail slot, into a nut bar sliding under the base. One
  allen key runs the whole jig from above, the plate top stays
  unbroken for the stencil
- keel rail added between the rail slots: a dovetail ridge on the
  base rides a matching groove under each plate, so a loosened plate
  stays square AND cannot lift
- freestanding tray + separate bench caddy -> a two-part stack: the
  jig stands on four screw-on feet beside an open tote box (box.py)
  standing on four of the SAME feet (finger room underneath to lift
  it), and stacks onto it as the lid for transport. The jig's rear
  feet carry hook noses that latch slots in the box wall; two
  quarter-turn cam knobs at the front seat under the box's
  descending ceiling tents and draw the base down onto the wall
  tops - clamped by geometry and PETG spring. TPU desk pads snap
  into the feet's through bores (one pad model, geometry retains it)
- credit-card squeegee -> printed standing wedge, both faces at 60
  deg: attack angle from geometry, not the hand
- side plates are thickness-agnostic and identical left/right; board
  thickness lives in screw-down adapter planks, one thickness per
  long edge (flip 180 to swap): 1.6/1.0, 1.2/0.8, 0.6/0.4, 2.0/1.6 -
  the full JLC rigid ladder

Mechanism: two side plates slide on slotted rails to clamp the board
width; the board rests on 1 mm ledges with UNDER mm free beneath
(double-sided boards). The frameless stencil slides under the rear
clamp plate, gets aligned by eye, and is clamped down. Boards then
swap out the front - the ejection tongue shoves the pasted board
forward - so one alignment covers a batch.

Screws: M3 button head (ISO 7380), one allen key; M3x16 everywhere
except the twelve M3x10s (eight feet, four adapter planks). Nuts: 4
(nut bars) + 4 (captive in the side plates) + 2..7 (clamp plate -
two is enough); foot screws thread-form their host pilots, no nut. Print 1 base, 2 side plates (same part), 2
planks per thickness pair, 2 nut bars, 1 clamp plate, 1 tongue, a
squeegee per stencil size, 8 feet
(6 plain + 2 hooked), 1 box, 2 cam knobs (each a flat-printed
handle keyed onto a standing-printed rod, spined by an M3x16
self-tapped down the shaft - steel carries the cam tension), 8 pads
(TPU, the one non-PETG part). bobstay recommends PETG, 6 walls, 1.6 top/bottom -
especially around the rail slots.

Layout: one part per file, every shared dimension in common.py,
mating cavities offset() from the owning part's profile, and every
part exports pre-oriented for the slicer (python -m stenciljig).

First target: osc-dev-m007, 50 x 50 x 1.579 board, QFN-20 - the 1.5
ledge drop seats it 0.08 proud so the stencil rests on the board.
Board max 120 x 120; stencil max 168 wide (the JLC 100 x 100
frameless sheet fits with 34 to spare each side)."""
