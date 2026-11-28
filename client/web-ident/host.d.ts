import type { DeviceKey, Progress, RunArgs, RunOutcome } from "./pkg/osc_client_web_ident.js";

/** The fields `findDevice` matches the lent device by, read off the page's `USBDevice`. */
export function keyOf(device: USBDevice): DeviceKey;

/** What `IdentWorker` posts to worker.js, and what it answers. */
export type ToWorker =
  | { kind: "run"; device: DeviceKey; args: RunArgs }
  | { kind: "runFake"; args: RunArgs }
  | { kind: "stop" };
export type FromWorker =
  | { kind: "ready" }
  | { kind: "progress"; progress: Progress }
  | { kind: "outcome"; outcome: RunOutcome }
  | { kind: "failed"; message: string };

/**
 * One run in `worker`, a dedicated worker on worker.js: `ready` settles once
 * the worker's module is in, one `run` or `runFake` resolves with the
 * outcome, `stop` raises the run's stop flag (before the run too), `close`
 * ends the worker.
 */
export class IdentWorker {
  constructor(worker: Worker);
  readonly ready: Promise<void>;
  /** `IdentRun.run` in the worker, on its own object for `device`, which the page's client has released. */
  run(device: USBDevice, args: RunArgs, onProgress: (p: Progress) => void): Promise<RunOutcome>;
  /** `IdentRun.runFake` in the worker. */
  runFake(args: RunArgs, onProgress: (p: Progress) => void): Promise<RunOutcome>;
  stop(): void;
  close(): void;
}
