import type {
  DeviceKey,
  Progress,
  ResumeArgs,
  ResumePlan,
  RunArgs,
  RunOutcome,
  SaveArgs,
  SaveOutcome,
  Stored,
} from "./pkg/osc_client_web_ident.js";

/** The fields `findDevice` matches the lent device by, read off the page's `USBDevice`. */
export function keyOf(device: USBDevice): DeviceKey;

/** What `IdentWorker` posts to worker.js, and what it answers. */
export type ToWorker =
  | { kind: "run"; device: DeviceKey; args: RunArgs }
  | { kind: "runFake"; args: RunArgs }
  | { kind: "resume"; device: DeviceKey; args: ResumeArgs }
  | { kind: "resumeFake"; args: ResumeArgs }
  | { kind: "save"; device: DeviceKey; args: SaveArgs }
  | { kind: "saveFake"; args: SaveArgs }
  | { kind: "plan"; id: number; args: ResumeArgs; now?: number }
  | { kind: "stop" };
export type FromWorker =
  | { kind: "ready" }
  | { kind: "progress"; progress: Progress }
  | { kind: "store"; stored: Stored }
  | { kind: "outcome"; outcome: RunOutcome | SaveOutcome }
  | { kind: "planned"; id: number; plan?: ResumePlan; refused?: string }
  | { kind: "failed"; message: string };

/** Where the page mirrors each change to the run's files; a promise holds the next one back. */
export type OnStore = (s: Stored) => void | Promise<void>;

/**
 * One run in `worker`, a dedicated worker on worker.js: `ready` settles once
 * the worker's module is in, one `run`, `runFake`, `resume`, `resumeFake`,
 * `save` or `saveFake` resolves with the outcome, `stop` raises the run's stop flag (before the
 * run too), `close` ends the worker. `planResume` asks what a resume would
 * do, at any time. Each store change goes to `onStore` on one serial queue,
 * the next once the last one's promise settled, and the run settles once
 * the queue has drained, so the page's mirror is whole by then.
 */
export class IdentWorker {
  constructor(worker: Worker);
  readonly ready: Promise<void>;
  /** `IdentRun.run` in the worker, on its own object for `device`, which the page's client has released. */
  run(
    device: USBDevice,
    args: RunArgs,
    onProgress: (p: Progress) => void,
    onStore?: OnStore,
  ): Promise<RunOutcome>;
  /** `IdentRun.runFake` in the worker. */
  runFake(args: RunArgs, onProgress: (p: Progress) => void, onStore?: OnStore): Promise<RunOutcome>;
  /** `IdentRun.resume` in the worker, as `run`. */
  resume(
    device: USBDevice,
    args: ResumeArgs,
    onProgress: (p: Progress) => void,
    onStore?: OnStore,
  ): Promise<RunOutcome>;
  /** `IdentRun.resumeFake` in the worker. */
  resumeFake(
    args: ResumeArgs,
    onProgress: (p: Progress) => void,
    onStore?: OnStore,
  ): Promise<RunOutcome>;
  /** `IdentRun.save` in the worker, on its own object for `device`, which the page's client has released. */
  save(
    device: USBDevice,
    args: SaveArgs,
    onProgress: (p: Progress) => void,
    onStore?: OnStore,
  ): Promise<SaveOutcome>;
  /** `IdentRun.saveFake` in the worker. */
  saveFake(
    args: SaveArgs,
    onProgress: (p: Progress) => void,
    onStore?: OnStore,
  ): Promise<SaveOutcome>;
  /** `planResume` in the worker: rejects with the refusal, in the tool's words. */
  planResume(args: ResumeArgs, now?: number): Promise<ResumePlan>;
  stop(): void;
  close(): void;
}
