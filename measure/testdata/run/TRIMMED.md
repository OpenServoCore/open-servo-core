# The recorded run, trimmed

`00000000ffffffff76e5bc220ef6abcd/20261127T130203Z/` is the run folder `sources.json` names, cut from 777 files
(29.1 MB) to 445 files (4.35 MB on disk, about 2.7 MB as a git pack). Every file kept is a byte copy of the run's
but the 29 that carried the bench's absolute paths (below). No file was renamed or added inside the run folder; this
note and `sources.json` sit beside it.

## Rewritten

- **Paths.** In every text file, the run folder's absolute path as the bench wrote it became `/bench/run`, and the
  source tree the tool was built from (`.../fixture-run-f-src`, open-servo-core 76ceb7c9) became `/bench/src`.
  That touched 29 files: in `inputs/` and `final/inputs/`, `measured.json`, `pos_lut.json`, `wide-codes.json`,
  `cross-checks.json` and `resolved.json`; `final/measured.json` and `final/set.json`; `tune/tune.json`,
  `tune/set.json` and `tune/back-off.json`; and each try's `measured.json` and `set.json`.
- **Sha1s.** Each rewritten file's new sha1 replaces its old one wherever another file records it: the nine set
  files (`final/set.json`, `tune/set.json`, each try's `set.json`) record the sha1s of the `pos_lut.json`,
  `wide-codes.json` and `measured.json` they were built from. A set's own hashes are over its values, so they stand.
- **Regenerated.** `measured.json`, `pos_lut.json`, `wide-codes.json` and `cross-checks.json` in `inputs/` (gathered
  without `results/cruise.json`, as the run gathered before its cruise step) and in `final/inputs/` (with it) are
  the gather's output over `results/` named under `/bench/run`. The results files carry no path and are the run's
  bytes, so the regenerated files equal the rewritten ones byte for byte, but for the two `measured.json` files: a
  later gather gives `drive_polarity` `sd` null and `n` 2, and the table step's `table` `sd` null and `n` 1, which
  the run's gather did not write. The tune's and the final check's files (`tune/`, `final/measured.json`,
  `final/set.json`) are the run's, read from its own `measured.json`. No file records either `measured.json`'s
  sha1. `the_recorded_run_gathers_to_its_own_inputs` gathers them again on every test run.
- The run's own sha1s of `run.json` and both `measured.json` files are in `sources.json`.
- **The rated travel.** The run read `measure/testdata/mg90-d/nameplate.json` when that file stated `travel_deg`
  184.33, the travel stop to stop, which the profile no longer carries. `inputs/nameplate.json` and
  `final/inputs/nameplate.json` are that file as it is now, `rated_travel_deg` 180 in its place; its new sha1
  replaces the old one (7028920c) in the nine set files that record it. The two `cross-checks.json` files are
  gathered again against it: the travel against the rated 180, and the pacing price, 4096 over 180 counts/deg,
  against the table's scale. The run's steps paced at 4096 over 184.33; their results say so and stay the run's.
- **The stops by ripple.** `results/table.json` is the table step's result re-judged at revision 2 from the run's
  full table captures (the bringup path `sources.json` gives): each stop's first contact by the motor's ripple, the
  stop rungs' rows cut 4 cycles before it, the table pinned at each contact moved outward to the grid (96 and 3760
  where the run pinned it at the stops step's seats 6 and 3737), and each stop rung's reach as the revision 2 rule
  reads it. The drives, rungs' plans, path and every other key are the run's. `inputs/` and `final/inputs/`
  `measured.json`, `pos_lut.json`, `wide-codes.json` and `cross-checks.json` are gathered again from it. The tune's
  and the final check's files stay the run's, built on the table it ran with.

## Kept

- Every JSON file: `run.json`, `results/`, `inputs/`, `final/` (its `inputs/`, `measured.json`, `set.json`,
  `results/verify.json`), `tune/` (`tune.json`, `set.json`, `back-off.json`, `backup/stored.json`, each try's
  `set.json`, `measured.json` and `results/verify.json`), and every capture's `.meta.json`, the dropped captures' too.
  A `.meta.json` with no rows file beside it is a capture trimmed out.
- `final/captures/verify/` whole: the 18 captures of the final check (the first enable, the four small steps, the
  lead, the 12 self-test moves).
- In each step's `captures/<step>/`, the first capture of each file kind, with every part of its rung:

| Folder | Kept |
|---|---|
| `captures/rest` | `01-rest` (rest, torque off), `02-raw` (raw, torque on at zero duty): both, one of each kind |
| `captures/probe` | `01-raw` (pulse 1 drive), `burst-0` (the first part A burst) |
| `captures/stops` | `burst-0` (seat 1 level 0), `polls.csv` |
| `captures/table` | `01-raw`, `02-raw` (rung 1, pass 1 down, free: its drive and coast parts) |
| `captures/window` | `01-raw` (onset 1) |
| `captures/spins` | `01-raw`, `02-raw` (the pilot: its drive and coast parts) |
| `captures/cruise` | `01-raw` (the enable) |

## Dropped

The rows files only (`.csv.gz`, `.csv`); their `.meta.json` stay. 332 files, 24.8 MB:

| Folder | Dropped rows files | Size |
|---|---|---|
| `captures/probe` | `02-raw` to `16-raw`, `burst-1` to `burst-15` | 362 kB |
| `captures/stops` | `burst-1` to `burst-47` | 389 kB |
| `captures/table` | `03-raw` to `60-raw` | 3.84 MB |
| `captures/window` | `02-raw` to `24-raw` | 171 kB |
| `captures/spins` | `03-raw` to `30-raw` | 888 kB |
| `captures/cruise` | `02-raw` to `21-raw` | 2.51 MB |
| `tune/tries/01` to `07/captures/verify` | `01-raw` to `18-raw` in each | 16.6 MB |

The full run is in the bringup repo at the path `sources.json` gives.
