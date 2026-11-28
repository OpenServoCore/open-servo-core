// The run as a page drives it in its worker: host.js on one end of a
// MessageChannel, worker.js's serve on the other, the built module on the
// simulated servo. Needs `wasm-pack build --target web --out-dir pkg` first.
import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import { test } from "node:test";
import init, * as ident from "../pkg/osc_client_web_ident.js";
import { IdentWorker } from "../host.js";
import { serve } from "../worker.js";

const wasm = await readFile(new URL("../pkg/osc_client_web_ident_bg.wasm", import.meta.url));
const loaded = init({ module_or_path: wasm }).then(() => ident);

const testdata = new URL("../../../measure/testdata/mg90-d/", import.meta.url);
const file = async (name, edit = (t) => t) => ({
  path: `profiles/mg90-d/${name}`,
  bytes: new TextEncoder().encode(
    edit(new TextDecoder().decode(await readFile(new URL(name, testdata)))),
  ),
});
const args = async (edit) => ({
  id: 1,
  nameplate: "profiles/mg90-d/nameplate.json",
  policy: "profiles/mg90-d/policy.json",
  files: await Promise.all(
    ["nameplate.json", "policy.json", "thermal.json"].map((n) =>
      file(n, n === "thermal.json" ? edit : undefined),
    ),
  ),
});

/** A worker on a MessageChannel: what `new Worker` gives a page, minus the thread. */
function spawn(module = loaded) {
  const { port1, port2 } = new MessageChannel();
  serve(port1, module);
  const worker = {
    postMessage: (m, transfer) => port2.postMessage(m, transfer),
    terminate: () => {
      port1.close();
      port2.close();
    },
    set onmessage(f) {
      port2.onmessage = f;
    },
    onerror: null,
  };
  return new IdentWorker(worker);
}

async function inWorker(start) {
  const w = spawn();
  const told = [];
  try {
    await w.ready;
    const outcome = await start(w, (p) => told.push(p));
    return { outcome, told };
  } finally {
    w.close();
  }
}

const manifest = (o) =>
  JSON.parse(new TextDecoder().decode(o.files.find((f) => f.path.endsWith("/run.json")).bytes));
const relative = (o) => o.files.map((f) => f.path.slice(o.root.length));
const started = (told) => told.filter((p) => p.event === "started").map((p) => p.step);

test("the worker's run is the page's run: the same files, the same lines, each told as it goes", async () => {
  const a = await args();
  const { outcome, told } = await inWorker((w, on) => w.runFake(a, on));
  const direct = await new ident.IdentRun().runFake(await args(), () => undefined);

  assert.deepEqual(started(told), ["rest", "probe"]);
  const fed = told.filter((p) => p.event === "lines").flatMap((p) => p.lines);
  assert.deepEqual(fed, outcome.lines);
  assert.deepEqual(relative(outcome), relative(direct));
  const unrooted = (o) => o.lines.map((l) => l.replaceAll(o.root, "<root>"));
  assert.deepEqual(unrooted(outcome), unrooted(direct));
  // Two runs differ in the host clock's stamps alone; the rest is byte for byte.
  const clockless = (o) =>
    o.files.map((f) =>
      new TextDecoder()
        .decode(f.bytes)
        .replaceAll(o.root, "<root>")
        .replace(/"(host_t_start|host_t_end|started|ended)": [0-9.]+/g, '"$1": 0'),
    );
  assert.deepEqual(clockless(outcome), clockless(direct));
  assert.equal(manifest(outcome).steps.probe.state, "aborted");
  assert.ok(outcome.files.every((f) => f.bytes instanceof Uint8Array && f.bytes.length > 0));
});

test("a stop posted before the run stops it at its first step", async () => {
  const a = await args();
  const { outcome } = await inWorker((w, on) => {
    w.stop();
    return w.runFake(a, on);
  });
  const m = manifest(outcome);
  assert.equal(m.steps.rest.state, "interrupted");
  assert.equal(m.steps.probe, undefined);
});

test("a stop posted as the probe starts ends the probe through its way out", async () => {
  const a = await args();
  const { outcome } = await inWorker((w, on) =>
    w.runFake(a, (p) => {
      on(p);
      if (p.event === "started" && p.step === "probe") w.stop();
    }),
  );
  assert.equal(manifest(outcome).steps.probe.state, "interrupted");
  assert.ok(outcome.lines.includes("  torque_enable 0, read back 0"), outcome.lines.join("\n"));
});

test("a device the site was not granted is refused before anything is sent", async () => {
  Object.defineProperty(globalThis.navigator ?? (globalThis.navigator = {}), "usb", {
    value: { getDevices: async () => [] },
    configurable: true,
    writable: true,
  });
  const lent = { vendorId: 0x1209, productId: 0x0001, serialNumber: "A1" };
  await assert.rejects(
    inWorker(async (w, on) => w.run(lent, await args(), on)),
    /not among this site's granted devices/,
  );
});

test("a worker runs once", async () => {
  const a = await args();
  await inWorker(async (w, on) => {
    await w.runFake(a, on);
    await assert.rejects(w.runFake(a, on), /another run takes a new worker/);
  });
});

test("a module that does not load fails the worker's ready and its run", async () => {
  const w = spawn(Promise.reject(new Error("no wasm here")));
  try {
    await assert.rejects(w.ready, /no wasm here/);
    await assert.rejects(w.runFake(await args(), () => undefined), /no wasm here/);
  } finally {
    w.close();
  }
});

/** The changes told, replayed in order onto `files` (a Map of path to bytes), as a page mirrors them. */
function replay(files, changes) {
  for (const c of changes) {
    if (c.op === "write") {
      files.set(c.path, c.bytes);
    } else {
      const moved = [...files.keys()].filter((k) => k === c.from || k.startsWith(`${c.from}/`));
      assert.ok(moved.length > 0, `${c.from} is there to move`);
      for (const k of moved) {
        files.set(`${c.to}${k.slice(c.from.length)}`, files.get(k));
        files.delete(k);
      }
    }
  }
  return files;
}
const filesOf = (o) => new Map(o.files.map((f) => [f.path, f.bytes]));
const sameFiles = (a, b) => {
  assert.deepEqual([...a.keys()].sort(), [...b.keys()].sort());
  for (const [k, v] of a) assert.deepEqual(Buffer.from(v), Buffer.from(b.get(k)), k);
};

test("each file goes to onStore as it is written, one change at a time, and the run settles after the last", async () => {
  const a = await args();
  const changes = [];
  let applying = 0;
  let overlapped = false;
  const onStore = async (c) => {
    applying += 1;
    overlapped ||= applying > 1;
    await new Promise((go) => setTimeout(go, 1));
    changes.push(c);
    applying -= 1;
  };
  const { outcome } = await inWorker((w, on) => w.runFake(a, on, onStore));
  assert.equal(overlapped, false, "one change at a time");
  assert.ok(changes.length > 0);
  sameFiles(replay(new Map(), changes), filesOf(outcome));
  const runJson = changes.filter((c) => c.path?.endsWith("/run.json"));
  assert.ok(runJson.length > 2, "run.json as each step begins and ends");
});

// The sim spends simulated time on the cooling wait, and stops itself past a
// simulated minute: a heat model with a short slow time constant keeps the
// wait inside it.
const quick = (t) => t.replace('"value": 128.92', '"value": 5');

test("a run cancelled in the probe resumes in a new worker from its mirror, and ends as an unbroken run does", async () => {
  const { outcome: whole } = await inWorker(async (w, on) => w.runFake(await args(quick), on));
  const changes = [];
  const { outcome: stopped } = await inWorker(async (w, on) =>
    w.runFake(
      await args(quick),
      (p) => {
        on(p);
        if (p.event === "started" && p.step === "probe") w.stop();
      },
      (c) => {
        changes.push(c);
      },
    ),
  );
  assert.equal(manifest(stopped).steps.probe.state, "interrupted");
  const folder = replay(new Map(), changes);
  sameFiles(folder, filesOf(stopped));

  const base = await args(quick);
  const resumeArgs = {
    id: base.id,
    nameplate: base.nameplate,
    policy: base.policy,
    folder: stopped.root,
    files: [...base.files, ...[...folder].map(([path, bytes]) => ({ path, bytes }))],
  };
  const more = [];
  const w = spawn();
  let plan, resumed;
  const told = [];
  try {
    await w.ready;
    plan = await w.planResume(resumeArgs);
    resumed = await w.resumeFake(resumeArgs, (p) => told.push(p), (c) => {
      more.push(c);
    });
  } finally {
    w.close();
  }
  assert.equal(plan.from, "probe");
  assert.equal(plan.segment, 2);
  assert.ok(plan.coolS > 0 && plan.coolS <= 5 * Math.log(30) + 1e-9, String(plan.coolS));
  assert.equal(resumed.refused, undefined, resumed.lines.join("\n"));
  assert.deepEqual(started(told), ["resume", "probe"]);
  assert.ok(
    told.some((p) => p.event === "unit" && p.step === "resume" && p.what === "s cooling"),
    "the cooling wait is told",
  );
  sameFiles(replay(folder, more), filesOf(resumed));
  const m = manifest(resumed);
  assert.equal(m.segments[1].from, "probe");
  assert.equal(m.steps.rest.segment, 1);
  assert.equal(m.steps.probe.segment, 2);
  assert.ok(folder.has(`${stopped.root}/segments/1/results/probe.json`));

  // The same folder as the unbroken run's, but for the host clock's stamps,
  // the segments, and the re-run probe's readings of the sim's noise, which
  // move by a count or so with the later sim time.
  const readings = { rail_v: (5.0 / 1241) * 1.5, position_before_counts: 1.5, position_after_counts: 1.5 };
  const masked = (files, root) => {
    const strip = (v) => {
      if (Array.isArray(v)) return v.map(strip);
      if (v === null || typeof v !== "object") return v;
      const out = {};
      for (const [k, x] of Object.entries(v)) {
        if (k === "segments" || k === "segment") continue;
        out[k] =
          ["host_t_start", "host_t_end", "started", "ended"].includes(k) && typeof x === "number"
            ? 0
            : strip(x);
      }
      return out;
    };
    const out = new Map();
    for (const [p, b] of files) {
      const rel = p.slice(root.length + 1);
      if (rel.startsWith("segments/")) continue;
      out.set(rel, rel.endsWith(".json") ? strip(JSON.parse(new TextDecoder().decode(b))) : Buffer.from(b));
    }
    const probe = out.get("results/probe.json");
    const read = {};
    for (const k of Object.keys(readings)) {
      read[k] = probe[k];
      delete probe[k];
    }
    return { out, read };
  };
  const again = masked(filesOf(resumed), resumed.root);
  const once = masked(filesOf(whole), whole.root);
  for (const [k, within] of Object.entries(readings)) {
    assert.ok(Math.abs(again.read[k] - once.read[k]) <= within, `${k}: ${again.read[k]} against ${once.read[k]}`);
  }
  assert.deepEqual(again.out, once.out);
});

test("a resume the folder refuses is refused before the bus, in the tool's words, and planResume says the same", async () => {
  const { outcome } = await inWorker(async (w, on) => w.runFake(await args(), on));
  const base = await args();
  const resumeArgs = {
    id: 1,
    nameplate: base.nameplate,
    policy: base.policy,
    folder: outcome.root,
    files: [...base.files, ...outcome.files.filter((f) => !f.path.endsWith("/run.json"))],
  };
  const words = `${outcome.root}: no run.json: not a run folder`;
  const changes = [];
  const { outcome: refused } = await inWorker(async (w, on) => {
    await assert.rejects(w.planResume(resumeArgs), (e) => e.message === words);
    return w.resumeFake(resumeArgs, on, (c) => changes.push(c));
  });
  assert.equal(refused.refused, words);
  assert.equal(changes.length, 0);
});

test("the worker's save stores a folder's accepted final set and reads it back clean", async () => {
  const uid = "c94b8419d1092aec87de5fd151ce290f";
  const folder = `${uid}/20261127T120000Z`;
  const bytes = async (name) => new Uint8Array(await readFile(new URL(name, testdata)));
  const set = await bytes("constants.json");
  const hash = JSON.parse(new TextDecoder().decode(set)).hash.set;
  const run = { servo_uid: uid, steps: {}, final: { state: "accepted", set: hash, set_file: "final/set.json" } };
  const a = {
    id: 1,
    folder,
    files: [
      { path: `${folder}/run.json`, bytes: new TextEncoder().encode(JSON.stringify(run)) },
      { path: `${folder}/final/set.json`, bytes: set },
      { path: `${folder}/final/inputs/pos_lut.json`, bytes: await bytes("pos_lut.json") },
    ],
  };
  const stored = [];
  const { outcome, told } = await inWorker((w, on) =>
    w.saveFake(a, on, (s) => {
      stored.push(s.op === "write" ? s.path : `${s.from} -> ${s.to}`);
    }),
  );
  assert.equal(outcome.refused, undefined, outcome.lines.join("\n"));
  assert.equal(outcome.saved, true);
  assert.deepEqual(outcome.differences, []);
  assert.equal(outcome.bootsAt, "b3000000");
  const fed = told.filter((p) => p.event === "lines").flatMap((p) => p.lines);
  assert.deepEqual(fed, outcome.lines);
  assert.ok(outcome.lines.some((l) => l.startsWith("0 differences; settings loaded; unsaved no")));
  assert.deepEqual(
    [...stored].sort(),
    outcome.files.map((f) => f.path),
  );
});
