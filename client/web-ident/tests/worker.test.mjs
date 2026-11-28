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
const file = async (name) => ({
  path: `profiles/mg90-d/${name}`,
  bytes: new Uint8Array(await readFile(new URL(name, testdata))),
});
const args = async () => ({
  id: 1,
  nameplate: "profiles/mg90-d/nameplate.json",
  policy: "profiles/mg90-d/policy.json",
  files: await Promise.all(["nameplate.json", "policy.json", "thermal.json"].map(file)),
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
