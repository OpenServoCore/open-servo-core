// The page's side of the run worker (worker.js). Loads nothing itself: the
// identification module is the worker's alone.

/** The fields `findDevice` matches the lent device by, read off the page's `USBDevice`. */
export function keyOf(device) {
  return {
    vendorId: device.vendorId,
    productId: device.productId,
    serialNumber: device.serialNumber ?? undefined,
  };
}

/**
 * One run in `worker`, a dedicated worker on worker.js: `ready` settles once
 * the worker's module is in, one `run` or `runFake` resolves with the
 * outcome, `stop` raises the run's stop flag (before the run too), `close`
 * ends the worker.
 */
export class IdentWorker {
  #worker;
  #job;
  #used = false;
  #loaded = false;
  #ready;
  #readyGo;
  #readyFail;

  constructor(worker) {
    this.#worker = worker;
    this.#ready = new Promise((go, fail) => {
      this.#readyGo = go;
      this.#readyFail = fail;
    });
    // Unhandled until someone awaits `ready`; the failure still reaches a run.
    this.#ready.catch(() => undefined);
    worker.onmessage = (e) => this.#on(e.data);
    worker.onerror = (e) => {
      e.preventDefault?.();
      this.#fail(new Error(e.message || "the identification worker did not load"));
    };
  }

  get ready() {
    return this.#ready;
  }

  run(device, args, onProgress) {
    return this.#start({ kind: "run", device: keyOf(device), args }, onProgress);
  }

  runFake(args, onProgress) {
    return this.#start({ kind: "runFake", args }, onProgress);
  }

  stop() {
    this.#worker.postMessage({ kind: "stop" });
  }

  close() {
    this.#worker.terminate();
  }

  #start(message, onProgress) {
    if (this.#used) {
      return Promise.reject(new Error("this worker has run; another run takes a new worker"));
    }
    this.#used = true;
    const done = new Promise((resolve, reject) => {
      this.#job = { resolve, reject, onProgress };
    });
    this.#worker.postMessage(message);
    return done;
  }

  #on(m) {
    switch (m.kind) {
      case "ready":
        this.#loaded = true;
        this.#readyGo();
        break;
      case "progress":
        this.#job?.onProgress(m.progress);
        break;
      case "outcome":
        this.#job?.resolve(m.outcome);
        this.#job = undefined;
        break;
      case "failed":
        this.#fail(new Error(m.message));
        break;
    }
  }

  #fail(e) {
    if (!this.#loaded) this.#readyFail(e);
    this.#job?.reject(e);
    this.#job = undefined;
  }
}
