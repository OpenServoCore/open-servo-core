// The identification run in a dedicated worker, driven from the page by
// host.js. A hidden page's timers are throttled; a dedicated worker's are
// not, so the run's pauses keep their length with the tab in the background.
import init, * as ident from "./pkg/osc_client_web_ident.js";

const RUNS = {
  run: (run, m, device, told, stored) => run.run(device, m.args, told, stored),
  runFake: (run, m, _device, told, stored) => run.runFake(m.args, told, stored),
  resume: (run, m, device, told, stored) => run.resume(device, m.args, told, stored),
  resumeFake: (run, m, _device, told, stored) => run.resumeFake(m.args, told, stored),
  save: (run, m, device, told, stored) => run.save(device, m.args, told, stored),
  saveFake: (run, m, _device, told, stored) => run.saveFake(m.args, told, stored),
};

/**
 * One run (or one save) served over `port` (the worker's scope, or a MessagePort in a
 * test), on the module `loaded` resolves to. Tells `ready` once the module
 * is in, each `progress` and each `store` change as the run goes, in the
 * order they happen, then its `outcome`, or `failed` with the message of
 * whatever threw. A `stop` before the run stops it at its first step, as
 * on the page. A `plan` is answered at any time and is not the run.
 */
export function serve(port, loaded) {
  let run;
  let stopped = false;
  const post = (m, transfer = []) => port.postMessage(m, transfer);
  const failed = (e) => post({ kind: "failed", message: e instanceof Error ? e.message : String(e) });

  async function go(m) {
    const mod = await loaded;
    if (run !== undefined) {
      failed(new Error("this worker has run; another run takes a new worker"));
      return;
    }
    run = new mod.IdentRun();
    if (stopped) run.stop();
    const told = (progress) => post({ kind: "progress", progress });
    const stored = (change) =>
      post({ kind: "store", stored: change }, change.op === "write" ? [change.bytes.buffer] : []);
    let device;
    try {
      if (m.device !== undefined) device = await mod.findDevice(m.device);
      const outcome = await RUNS[m.kind](run, m, device, told, stored);
      const buffers = new Set(outcome.files.map((f) => f.bytes.buffer));
      post({ kind: "outcome", outcome }, [...buffers]);
    } catch (e) {
      // A run that threw past its own close would leave the page's reclaim a
      // device this worker still holds.
      if (device?.opened) await device.close().catch(() => undefined);
      failed(e);
    }
  }

  async function plan(m) {
    const mod = await loaded;
    try {
      post({ kind: "planned", id: m.id, plan: mod.planResume(m.args, m.now) });
    } catch (e) {
      post({ kind: "planned", id: m.id, refused: e instanceof Error ? e.message : String(e) });
    }
  }

  port.onmessage = (e) => {
    const m = e.data;
    if (m.kind === "stop") {
      stopped = true;
      run?.stop();
    } else if (m.kind === "plan") {
      plan(m).catch(failed);
    } else if (m.kind in RUNS) {
      go(m).catch(failed);
    }
  };
  loaded.then(() => post({ kind: "ready" }), failed);
}

if (typeof WorkerGlobalScope !== "undefined" && self instanceof WorkerGlobalScope) {
  serve(self, init().then(() => ident));
}
