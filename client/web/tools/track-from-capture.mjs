#!/usr/bin/env node
// One telemetry capture (notebooks/telemetry/<rig>/<campaign>/capture-N) to a
// Track JSON for OscClient.fakeWithTracks:
//
//   node track-from-capture.mjs <capture-dir> [--file <name.csv.gz>]
//                               [--from <row>] [--rows <n>] [--out <path>]
//
// The window is a row range into the capture's stream (the CSV `tick`
// column restarts per segment, so rows count from the file's first row).
// Default: 10000 rows (0.5 s at 20 kHz) from the first window_valid row.
// Empty CSV cells read as 0. Output: { meta: { source, sense, tick_hz },
// track: { <Track columns> } }, to stdout unless --out.

import { existsSync, readdirSync, readFileSync, writeFileSync } from "node:fs";
import { basename, dirname, join, relative, resolve } from "node:path";
import { gunzipSync } from "node:zlib";

const COLUMNS = {
  pos: "pos",
  current: "current",
  currentTrough: "current_trough",
  dutyQ15: "duty_q15",
  vdiff: "vdiff",
  vbus: "vbus",
  currentRaw: "current_raw",
  vmotorA: "vmotor_a",
  vmotorB: "vmotor_b",
  vbusRaw: "vbus_raw",
  ntcRaw: "ntc_raw",
  windowValid: "window_valid",
};
const DEFAULT_ROWS = 10000;

function usage(msg) {
  console.error(`track-from-capture: ${msg}`);
  console.error(
    "usage: node track-from-capture.mjs <capture-dir> [--file <name.csv.gz>] [--from <row>] [--rows <n>] [--out <path>]",
  );
  process.exit(2);
}

function parseArgs(argv) {
  const opts = { dir: null, file: null, from: null, rows: DEFAULT_ROWS, out: null };
  for (let i = 0; i < argv.length; i++) {
    const a = argv[i];
    const next = () => {
      if (i + 1 >= argv.length) usage(`${a} needs a value`);
      return argv[++i];
    };
    if (a === "--file") opts.file = next();
    else if (a === "--from") opts.from = Number(next());
    else if (a === "--rows") opts.rows = Number(next());
    else if (a === "--out") opts.out = next();
    else if (a.startsWith("--")) usage(`unknown option ${a}`);
    else if (opts.dir === null) opts.dir = a;
    else usage(`unexpected argument ${a}`);
  }
  if (opts.dir === null) usage("capture directory required");
  if (opts.from !== null && !(Number.isInteger(opts.from) && opts.from >= 0)) usage("--from must be a row index");
  if (!(Number.isInteger(opts.rows) && opts.rows > 0)) usage("--rows must be a positive count");
  return opts;
}

function pickCsv(dir, file) {
  const gz = readdirSync(dir).filter((f) => f.endsWith(".csv.gz")).sort();
  if (file !== null) {
    if (!gz.includes(file)) usage(`${file} not in ${dir} (have: ${gz.join(", ") || "none"})`);
    return file;
  }
  if (gz.length === 1) return gz[0];
  usage(gz.length === 0 ? `no .csv.gz in ${dir}` : `several captures in ${dir}, pick one with --file: ${gz.join(", ")}`);
}

function readMeta(dir, csv) {
  const stem = csv.slice(0, -".csv.gz".length);
  for (const name of [`${stem}.meta.json`, "meta.json"]) {
    const p = join(dir, name);
    if (existsSync(p)) return JSON.parse(readFileSync(p, "utf8"));
  }
  usage(`no meta.json beside ${csv}`);
}

// Repo-relative when the capture sits in a git checkout, else as given.
function sourcePath(dir) {
  for (let d = resolve(dir); ; d = dirname(d)) {
    if (existsSync(join(d, ".git"))) return relative(d, resolve(dir));
    if (dirname(d) === d) return dir;
  }
}

function main() {
  const opts = parseArgs(process.argv.slice(2));
  const csv = pickCsv(opts.dir, opts.file);
  const meta = readMeta(opts.dir, csv);
  const text = gunzipSync(readFileSync(join(opts.dir, csv))).toString("utf8");
  const lines = text.split("\n");
  if (lines.length && lines[lines.length - 1] === "") lines.pop();
  const header = lines.shift().split(",");
  const index = {};
  for (const [key, col] of Object.entries(COLUMNS)) {
    const i = header.indexOf(col);
    if (i < 0) usage(`${csv} has no ${col} column`);
    index[key] = i;
  }

  let from = opts.from;
  if (from === null) {
    const wv = index.windowValid;
    from = lines.findIndex((l) => l.split(",")[wv] === "1");
    if (from < 0) from = 0;
  }
  if (from >= lines.length) usage(`--from ${from} is past the capture's ${lines.length} rows`);
  const to = Math.min(from + opts.rows, lines.length);

  const track = Object.fromEntries(Object.keys(COLUMNS).map((k) => [k, []]));
  for (let r = from; r < to; r++) {
    const cells = lines[r].split(",");
    for (const [key, i] of Object.entries(index)) {
      const v = cells[i];
      track[key].push(v === "" || v === undefined ? 0 : Number(v));
    }
  }

  const out = {
    meta: {
      source: join(sourcePath(opts.dir), csv),
      sense: meta.sense,
      tick_hz: meta.tick_hz,
    },
    track,
  };
  const json = JSON.stringify(out);
  if (opts.out === null) {
    process.stdout.write(json + "\n");
  } else {
    writeFileSync(opts.out, json + "\n");
  }
  console.error(
    `${basename(opts.dir)}/${csv}: rows ${from}..${to} of ${lines.length} (${to - from} samples), ${json.length + 1} bytes`,
  );
}

main();
