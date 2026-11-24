# The cruise step's fixture

Closed-loop Velocity cruises of the mg90-d on dev-v006-2A-D, set of record
`mg90-d__dev-v006-2A-D`, from the friction sitting's captures in the bringup
repo (`captures/friction-settle/`): K1-cl, K2-cl-a, K2-cl-b, K3-cl, H-cl and
R-c300. `tests/cruise.rs` reads them through `fit::cruise` and holds each leg
and the line to `expected.json`.

## How it was cut

`gen.py` cuts the fixture and computes the expected values with a Python
reference of the step's method. Run read-only from a notebooks checkout:

    cd notebooks && .venv/bin/python <this folder>/gen.py \
        <bringup>/captures/friction-settle \
        <bringup>/kb/scratch/friction-spins-vs-cruise/sitting/line.csv

- Every leg of a nonzero goal in those runs, but three: R-c300 segments 4
  and 5 and H-cl segment 7, left out for size (`LEFT_OUT`); the R-c1500 run
  is not in: none of its legs passes the steadiness rule under the step's
  row cut (they are not centered on the travel and cross only the top of
  its middle third).
- Per leg only the rows the fit reads: from 32 rows before the first settled
  row inside its speed's window to the last. Columns: the window flag, the
  raw pot and `duty_counts` as deltas, `current_raw` on read rows only, and
  the tap difference on one row in 32. The polled bias after each leg is
  kept beside it.
- `legs.json.gz` is 300 KB (167512 rows); `expected.json` 21 KB holds the
  sources with their sha1, the table points and slots, the constants the
  fit reads, each leg's numbers, the lines on the cut rows, on every row of
  the captures (`full`), on every leg of the runs (`every_leg`), and the
  analyst's line from the sitting's `line.csv`.

## The numbers

The step's line is every valid leg's, the 300 c/s legs anchoring the offset;
a leg is valid when its speed is held and its inertia correction is inside
1.5 mA, its speed's halves reported but not judged.
Each standard error is the larger of the legs' scatter about the line (legs
less two degrees of freedom) and the speeds' means about it (speeds less two),
which one said. All on the fixture's 1135.01 counts/A.

| line | coulomb, mA | viscous, mA per rad/s | legs | rad/s |
|---|---:|---:|---:|---|
| the step's: 300, 2150 and 4025 c/s | 37.37 +/- 1.43 (speeds; legs 0.45) | 0.04313 +/- 0.00203 (speeds; legs 0.00064) | 21 | 79 to 1074 |
| without the 300 c/s legs | 40.53 +/- 0.76 (legs) | 0.03965 +/- 0.00089 (legs) | 17 | 565 to 1074 |
| the analyst, with 300 c/s | 37.33 +/- 0.64 | 0.04318 +/- 0.00112 | | |
| the analyst, 1500 to 4025 c/s | 39.42 +/- 1.18 | 0.04036 +/- 0.00172 | | 400 to 1076 |
