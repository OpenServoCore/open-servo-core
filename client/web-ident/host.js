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
 * the worker's module is in, one `run`, `runFake`, `resume` or `resumeFake`
 * resolves with the outcome, `stop` raises the run's stop flag (before the
 * run too), `close` ends the worker. `planResume` asks what a resume would
 * do, at any time. Each store change goes to `onStore` on one serial queue,
 * the next once the last one's promise settled, and the run settles once
 * the queue has drained, so the page's mirror is whole by then.
 */
export class IdentWorker {
  #worker;
  #job;
  #used = false;
  #loaded = false;
  #ready;
  #readyGo;
  #readyFail;
  #plans = new Map();
  #planned = 0;

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

  run(device, args, onProgress, onStore) {
    return this.#start({ kind: "run", device: keyOf(device), args }, onProgress, onStore);
  }

  runFake(args, onProgress, onStore) {
    return this.#start({ kind: "runFake", args }, onProgress, onStore);
  }

  resume(device, args, onProgress, onStore) {
    return this.#start({ kind: "resume", device: keyOf(device), args }, onProgress, onStore);
  }

  resumeFake(args, onProgress, onStore) {
    return this.#start({ kind: "resumeFake", args }, onProgress, onStore);
  }

  planResume(args, now) {
    const id = (this.#planned += 1);
    return new Promise((resolve, reject) => {
      this.#plans.set(id, { resolve, reject });
      this.#worker.postMessage({ kind: "plan", id, args, now });
    });
  }

  stop() {
    this.#worker.postMessage({ kind: "stop" });
  }

  close() {
    this.#worker.terminate();
  }

  #start(message, onProgress, onStore) {
    if (this.#used) {
      return Promise.reject(new Error("this worker has run; another run takes a new worker"));
    }
    this.#used = true;
    const done = new Promise((resolve, reject) => {
      this.#job = { resolve, reject, onProgress, onStore, queue: Promise.resolve() };
    });
    this.#worker.postMessage(message);
    return done;
  }

  /** `then` once every store change before it was applied. */
  #after(then) {
    const job = this.#job;
    this.#job = undefined;
    if (job !== undefined) void job.queue.then(() => then(job));
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
      case "store": {
        const job = this.#job;
        if (job?.onStore === undefined) break;
        // A mirror that throws cannot stop the run, as a feed that throws cannot.
        job.queue = job.queue.then(() => job.onStore(m.stored)).catch(() => undefined);
        break;
      }
      case "outcome":
        this.#after((job) => job.resolve(m.outcome));
        break;
      case "planned": {
        const p = this.#plans.get(m.id);
        this.#plans.delete(m.id);
        if (m.refused === undefined) p?.resolve(m.plan);
        else p?.reject(new Error(m.refused));
        break;
      }
      case "failed":
        this.#fail(new Error(m.message));
        break;
    }
  }

  #fail(e) {
    if (!this.#loaded) {
      this.#readyFail(e);
      for (const p of this.#plans.values()) p.reject(e);
      this.#plans.clear();
    }
    this.#after((job) => job.reject(e));
  }
}
