// The identification run in a dedicated worker, driven from the page by
// host.js. A hidden page's timers are throttled; a dedicated worker's are
// not, so the run's pauses keep their length with the tab in the background.
import init, * as ident from "./pkg/osc_client_web_ident.js";

/**
 * One run served over `port` (the worker's scope, or a MessagePort in a
 * test), on the module `loaded` resolves to. Tells `ready` once the module
 * is in, each `progress` as the run goes, then its `outcome`, or `failed`
 * with the message of whatever threw. A `stop` before the run stops it at
 * its first step, as on the page.
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
    let device;
    try {
      let outcome;
      if (m.kind === "runFake") {
        outcome = await run.runFake(m.args, told);
      } else {
        device = await mod.findDevice(m.device);
        outcome = await run.run(device, m.args, told);
      }
      const buffers = new Set(outcome.files.map((f) => f.bytes.buffer));
      post({ kind: "outcome", outcome }, [...buffers]);
    } catch (e) {
      // A run that threw past its own close would leave the page's reclaim a
      // device this worker still holds.
      if (device?.opened) await device.close().catch(() => undefined);
      failed(e);
    }
  }

  port.onmessage = (e) => {
    const m = e.data;
    if (m.kind === "stop") {
      stopped = true;
      run?.stop();
    } else if (m.kind === "run" || m.kind === "runFake") {
      go(m).catch(failed);
    }
  };
  loaded.then(() => post({ kind: "ready" }), failed);
}

if (typeof WorkerGlobalScope !== "undefined" && self instanceof WorkerGlobalScope) {
  serve(self, init().then(() => ident));
}
