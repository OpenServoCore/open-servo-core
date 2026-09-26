"""Session captures: one sweep holding every free-shaft procedure, sliced back.

A session capture replaces six classic experiments with two recordings:

  telemetry/<dataset>/session/capture-N/slow.csv.gz   grid + coast + step +
                                                      reversal + breakaway
                                       /fast.csv.gz   the grid under fast decay

Running every procedure within minutes of the others puts them on the same
rail and the same gear temperature, which the classic experiments, captured
hours apart, never were. The block order rotates per capture, and
meta["session"]["blocks"] records where each block landed in the schedule.

`osc sweep` numbers segments as seg = 1 + dir_index * len(schedule) + k, with
seg 0 the torque-off baseline. A slice keeps seg 0 and renumbers the kept
steps as a classic sweep of the sub-schedule would have numbered them, so
code written against the classic recordings reads a view unchanged:

  df, meta = session.read(ds, "capture-1", "step:40")
"""

from .datasets import Dataset

CLASSIC = {
    "grid":      ["grid/sweep", "bridge/slow", "ripple/sweep"],
    "fast":      ["bridge/fast"],
    "coast":     ["bridge/spindown"],
    "step:20":   ["stepcoast/step20"],
    "step:40":   ["stepcoast/step40"],
    "step:60":   ["stepcoast/step60"],
    "step:80":   ["stepcoast/step80"],
    "step:120":  ["stepcoast/step120"],
    "reversal":  ["reversal/sweep"],
    "breakaway": ["breakaway/sweep"],
}


def parse(step, window_ms):
    """(kind, pct, window_ms) of one schedule string, pct unsigned by dir."""
    for kind in ("coast", "brake"):
        if step.startswith(kind + ":"):
            return kind, 0, int(step.split(":", 1)[1])
    head, _, w = step.partition("@")
    ms = int(w) if w else window_ms
    if head.startswith("then:"):
        return "then", int(head[5:]), ms
    return "drive", int(head), ms


def _block_of(meta, k):
    for b in meta.get("session", {}).get("blocks", []):
        if b["first"] <= k < b["first"] + b["count"]:
            return b["name"]
    return None


def segment_map(meta) -> list:
    sched, n = meta["schedule"], len(meta["schedule"])
    out = []
    for d, sign in enumerate(meta["dirs"]):
        for k, step in enumerate(sched):
            kind, pct, ms = parse(step, meta["window_ms"])
            out.append({"seg": 1 + d * n + k, "dir": sign, "k": k, "step": step,
                        "block": _block_of(meta, k), "kind": kind,
                        "pct": sign * pct, "window_ms": ms})
    return out


def slice_steps(df, meta, first, count, block=None):
    sched, n = meta["schedule"], len(meta["schedule"])
    if not (0 <= first and count > 0 and first + count <= n):
        raise IndexError(f"steps {first}..{first + count - 1} outside a {n}-step schedule")
    renum = {0: 0}
    for d in range(len(meta["dirs"])):
        for j in range(count):
            renum[1 + d * n + first + j] = 1 + d * count + j
    out = df[df["seg"].isin(renum)].copy()
    out["seg"] = out["seg"].map(renum).astype(df["seg"].dtype)
    out = out.reset_index(drop=True)

    sub = sched[first:first + count]
    m = dict(meta)
    m.pop("session", None)
    m["schedule"] = sub
    m["session_block"] = block
    # Effective windows, so a step without @MS keeps its meaning.
    ws = {ms for kind, _, ms in (parse(s, meta["window_ms"]) for s in sub)
          if kind in ("drive", "then")}
    if len(ws) == 1:
        m["window_ms"] = ws.pop()
    return out, m


def block(df, meta, name):
    blocks = meta.get("session", {}).get("blocks", [])
    for b in blocks:
        if b["name"] == name:
            return slice_steps(df, meta, b["first"], b["count"], block=name)
    raise KeyError(f"no session block {name!r}; have {[b['name'] for b in blocks]}")


def chain(df, meta, steps):
    sched, steps = meta["schedule"], list(steps)
    hits = [i for i in range(len(sched) - len(steps) + 1)
            if sched[i:i + len(steps)] == steps]
    if len(hits) != 1:
        raise KeyError(f"chain {steps} occurs {len(hits)} times in the schedule, need exactly 1")
    return slice_steps(df, meta, hits[0], len(steps), block=_block_of(meta, hits[0]))


def _recording(ds: Dataset, capture, name):
    recs = [r for r in ds.recordings("session", capture) if r.name == name]
    if len(recs) != 1:
        raise KeyError(f"{ds.key}/session/{capture}: no {name!r} recording")
    return recs[0]


def read(ds: Dataset, capture, view, check=True):
    """(df, meta) of one view of a session capture; views are the CLASSIC keys."""
    if view not in CLASSIC:
        raise ValueError(f"unknown session view {view!r}; pick one of {list(CLASSIC)}")
    rec = _recording(ds, capture, "fast" if view == "fast" else "slow")
    df, meta = rec.frame(check=check), rec.meta
    if view == "fast":
        return df, dict(meta)
    if not view.startswith("step:"):
        return block(df, meta, view)
    # The 30% drive and whatever coast follows it: the coast length is the
    # session's choice, the drive window is what names the view.
    drive, sched = f"30@{view[5:]}", meta["schedule"]
    tails = [sched[i + 1] for i in range(len(sched) - 1)
             if sched[i] == drive and sched[i + 1].startswith("coast:")]
    if len(tails) != 1:
        raise KeyError(f"{rec}: {drive} followed by a coast occurs {len(tails)} times, need 1")
    return chain(df, meta, [drive, tails[0]])
