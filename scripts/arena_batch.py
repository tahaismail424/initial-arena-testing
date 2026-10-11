"""Run a queue of recorded Arena simulations, one after another ("batch scheduler").

Each JOB = one Arena launch (one planner) that records episodes, then stops. A job runs either ONE scenario
(`scenario` + `episodes`, the original format) or MANY scenarios in the same launch (`scenarios` +
`episodes_per_scenario`): the scheduler hot-switches the task generator's `task.scenario.file` between episodes
(docs/14 §9), which saves ~1 min of launch per scenario. Jobs run strictly sequentially: one simulation uses ~5.5
of this VM's 8 cores, and two would share ROS/Gazebo topics.

Usage
-----
    uv run scripts/arena_batch.py first_experiment/jobs.yaml
    uv run scripts/arena_batch.py first_experiment/jobs.yaml --dry-run        # show what would run, with an ETA
    uv run scripts/arena_batch.py first_experiment/jobs.yaml --resume         # skip finished jobs / scenarios
    uv run scripts/arena_batch.py --planner dsrnn dwb --episodes 5 --out ~/arena_runs/quick_test   # one-off

Jobs file format (YAML)
-----------------------
    out: data                       # where runs go (relative to this file); one run folder per launch
    defaults:                       # apply to every job unless the job overrides them
      world: school_hallway
      scenario: hallway_tame.json
      episodes: 20
      headless: true
      args: []                      # extra `ros2 launch` arguments, e.g. ["global_planner:=smac"]
    jobs:
      - planner: dsrnn                                  # single scenario (defaults.scenario), 20 episodes
      - {planner: dwb, name: dwb_amcl, args: ["localization:=amcl"]}
      - name: campus_dwb                                # many scenarios in ONE launch
        planner: dwb
        world: campus
        scenarios: ["corridor_counterflow_s*.json", "door_exit_s0.json"]   # names or globs in this repo's
        episodes_per_scenario: 2                        #   worlds/<world>/scenarios/ (sorted, de-duplicated)
        order: sequential           # or: shuffled (seeded with `seed`, default 0)
        seed: 0

A job with `scenarios` ignores `scenario`/`episodes`. Its name must be unique (it keys the manifest).

How a multi-scenario job runs
-----------------------------
1. takes the shared simulator lock (/tmp/arena-sim.lock; waits if another batch or agent holds it), stops any
   leftover Arena processes, launches `arena-launch.sh --record=<out> ... scenario:=<first scenario>`
2. polls the recorder's run folder every few seconds. Every episode's meta.json says which scenario the task
   generator ACTUALLY applied at that reset (latched /task_generator_node/scenario_info, our arena-rosnav
   patch), so counting is done from those labels, never from assumptions.
3. switching: while the LAST wanted episode of scenario S runs, it sets task.scenario.file to the next scenario
   (rcl_interfaces SetParameters service; `ros2 param set` hangs against the task generator), so the automatic
   reset at the end of that episode loads the next one. If the parameter lands too late (the reset already
   started), the extra episode of S is recorded with its correct label, cut short with /reset_task (outcome
   "reset") and not counted.
4. when every scenario has its episodes (or no new episode starts within `stall_limit` s, or the launch dies),
   it stops the simulation, releases the lock and writes <out>/batch_manifest.json (per scenario: requested and
   complete episodes; run folders; status complete / stalled / failed). Launch log: <out>/logs/<job>.log.
--resume reruns only what is missing: finished jobs are skipped, and a multi-scenario job relaunches with just
the scenarios that still lack episodes (and only the missing number of them).

Ctrl+C stops the current simulation cleanly and releases the lock.
"""
from __future__ import annotations

import argparse
import fnmatch
import json
import os
import random
import re
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parent.parent
LAUNCH = REPO / "scripts" / "arena-launch.sh"
STOP = REPO / "scripts" / "arena-stop.sh"
ARENA_WS = Path.home() / "arena_ws"
LOCK = Path("/tmp/arena-sim.lock")
# Rough seconds per episode for the dry-run ETA (from the hallway_tame runs); unknown planners use the default.
EPISODE_SECONDS = {"dsrnn": 90, "dwb": 290, "default": 300}
STARTUP_SECONDS = 60
POLL_SECONDS = 4
TASKGEN = "/task_generator_node"

DEFAULTS = {"world": "school_hallway", "scenario": "hallway_tame.json", "task_mode": "scenario", "episodes": 20,
            "headless": True, "args": [], "stall_limit": 900, "order": "sequential", "seed": 0}


def log(msg: str):
    print(f"[{datetime.now():%H:%M:%S}] {msg}", flush=True)


def fmt_minutes(seconds: float) -> str:
    return f"{seconds / 60:.0f} min" if seconds < 5400 else f"{seconds / 3600:.1f} h"


# ------------------------------------------------------------------------------------------------- jobs

def expand_scenarios(world: str, patterns: list[str]) -> list[str]:
    """File names / globs -> scenario file names in this repo's worlds/<world>/scenarios/ (sorted per pattern,
    first occurrence wins). A plain name that isn't there is kept (it may exist only in Arena) with a warning."""
    sdir = REPO / "worlds" / world / "scenarios"
    available = sorted(p.name for p in sdir.glob("*.json")) if sdir.is_dir() else []
    out: list[str] = []
    for pat in patterns:
        if any(ch in pat for ch in "*?["):
            matches = fnmatch.filter(available, pat)
            if not matches:
                sys.exit(f"scenario pattern {pat!r} matches nothing in {sdir}")
        else:
            matches = [pat]
            if pat not in available:
                log(f"warning: {pat} is not in {sdir} (fine if it exists in Arena's copy)")
        out += [m for m in matches if m not in out]
    return out


def load_jobs(args) -> tuple[list[dict], Path]:
    """Jobs (each a complete dict of settings) and the output directory, from a jobs file or the command line.
    Every job gets `plan`: the ordered list of [scenario, episodes] it must record."""
    if args.jobs_file:
        spec = yaml.safe_load(Path(args.jobs_file).read_text()) or {}
        base = Path(args.jobs_file).resolve().parent
        out = Path(args.out or spec.get("out", "data")).expanduser()
        out = out if out.is_absolute() else base / out
        defaults = {**DEFAULTS, **(spec.get("defaults") or {})}
        raw_jobs = spec.get("jobs") or []
    else:
        out = Path(args.out or Path.home() / "arena_runs").expanduser()
        defaults = {**DEFAULTS, "episodes": args.episodes, "scenario": args.scenario, "world": args.world,
                    "headless": not args.gui, "args": args.launch_args or []}
        raw_jobs = [{"planner": p} for p in (args.planner or [])]
    jobs = []
    for i, j in enumerate(raw_jobs):
        job = {**defaults, **j}
        if "planner" not in job:
            sys.exit(f"job {i} has no 'planner'")
        job.setdefault("name", f"{i:02d}_{job['planner']}")
        if job.get("scenarios"):
            pats = job["scenarios"] if isinstance(job["scenarios"], list) else [job["scenarios"]]
            names = expand_scenarios(job["world"], [str(p) for p in pats])
            if job["order"] == "shuffled":
                random.Random(job["seed"]).shuffle(names)
            elif job["order"] != "sequential":
                sys.exit(f"job {job['name']}: order must be sequential or shuffled")
            k = int(job.get("episodes_per_scenario", job["episodes"]))
            job["multi"] = True
            job["plan"] = [[s, k] for s in names]
        else:
            job["multi"] = False
            job["plan"] = [[job["scenario"], int(job["episodes"])]]
        jobs.append(job)
    if not jobs:
        sys.exit("no jobs (give a jobs file or --planner ...)")
    if len({j["name"] for j in jobs}) != len(jobs):
        sys.exit("job names must be unique")
    return jobs, out


def launch_command(job: dict, out: Path, first_scenario: str) -> list[str]:
    cmd = ["bash", str(LAUNCH)]
    if job["headless"]:
        cmd.append("--headless")
    cmd += [f"--record={out}", job["world"], job["task_mode"]]
    if job["task_mode"] == "scenario":
        cmd.append(f"scenario:={first_scenario}")
    cmd.append(f"local_planner:={job['planner']}")
    cmd += [str(a) for a in job["args"]]
    return cmd


# ------------------------------------------------------------------------------------------------- lock

def acquire_lock(owner: str, poll: float = 30.0):
    """The simulator lock shared with other batches / agents: an atomic mkdir."""
    waited = False
    while True:
        try:
            LOCK.mkdir()
            (LOCK / "owner").write_text(f"{owner} (pid {os.getpid()})\n")
            return
        except FileExistsError:
            if not waited:
                holder = (LOCK / "owner").read_text().strip() if (LOCK / "owner").exists() else "?"
                log(f"simulator busy (lock held by {holder}); waiting...")
                waited = True
            time.sleep(poll)


def release_lock():
    owner = LOCK / "owner"
    if owner.exists() and f"pid {os.getpid()}" in owner.read_text():
        owner.unlink()
        LOCK.rmdir()


def stop_sim():
    subprocess.run(["bash", str(STOP)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


# ------------------------------------------------------------------------------------------------- ROS calls

def ros2(cmd: str, timeout: float = 45.0) -> str | None:
    """Run a `ros2 ...` command in Arena's environment (this script runs in the uv env). None on failure."""
    script = (f"cd {ARENA_WS} && source arena.bash >/dev/null 2>&1; unset FASTRTPS_DEFAULT_PROFILES_FILE; "
              f"timeout {int(timeout)} {cmd}")
    try:
        r = subprocess.run(["bash", "-c", script], capture_output=True, text=True, timeout=timeout + 30)
    except subprocess.TimeoutExpired:
        return None
    return r.stdout if r.returncode == 0 else None


def set_scenario_param(scenario: str, attempts: int = 3) -> bool:
    """task.scenario.file := scenario, via the SetParameters service (the task generator's parameter service can
    be slow under load: generous timeout, a few retries)."""
    req = ("{parameters: [{name: task.scenario.file, value: {type: 4, string_value: " + scenario + "}}]}")
    for i in range(attempts):
        out = ros2(f"ros2 service call {TASKGEN}/set_parameters rcl_interfaces/srv/SetParameters '{req}'")
        if out and "successful=True" in out:
            return True
        log(f"setting task.scenario.file={scenario} failed (attempt {i + 1}/{attempts})")
    return False


def reset_task() -> bool:
    return ros2(f"ros2 service call {TASKGEN}/reset_task std_srvs/srv/Empty '{{}}'") is not None


# ------------------------------------------------------------------------------------------------- one job

def read_episodes(run_dir: Path) -> list[dict]:
    """meta.json of every episode folder (sorted), skipping ones that are mid-write."""
    eps = []
    for d in sorted(run_dir.glob("episode_*")):
        try:
            m = json.loads((d / "meta.json").read_text())
        except (OSError, ValueError):
            continue
        m["_dir"] = d.name
        eps.append(m)
    return eps


def counted(m: dict, wanted: set) -> bool:
    """An episode counts for its scenario if it ended normally (not cut by the recorder / scheduler)."""
    return m.get("outcome") not in ("running", "incomplete", "reset", None) and m.get("scenario") in wanted


def run_job(job: dict, out: Path, plan: list[list]) -> dict:
    """Record plan = [[scenario, episodes], ...] in one launch. Returns this launch's result:
    {status, run_dir, counts {scenario: complete}, minutes, extra_episodes, switches}."""
    planner = job["planner"]
    (out / "logs").mkdir(parents=True, exist_ok=True)
    logfile = out / "logs" / f"{job['name']}.log"
    if logfile.exists():   # keep the log of an earlier attempt (resume)
        logfile.rename(logfile.with_name(f"{logfile.stem}.{datetime.now():%Y%m%d-%H%M%S}.log"))
    order = [s for s, _ in plan]
    want = {s: int(k) for s, k in plan}
    total = sum(want.values())
    counts = {s: 0 for s in order}

    acquire_lock(f"arena_batch:{job['name']}")
    started = time.time()
    result = {"status": "failed", "run_dir": None, "counts": counts, "extra_episodes": 0, "switches": [],
              "minutes": 0.0}
    try:
        stop_sim()   # nothing left over from earlier runs
        cmd = launch_command(job, out, order[0])
        log(f"{job['name']}: launching ({len(order)} scenario(s), {total} episodes): {' '.join(cmd[2:])}")
        with open(logfile, "w") as f:
            # own process group, so Ctrl+C on the batch doesn't hit the sim directly (stop_sim() handles it)
            proc = subprocess.Popen(cmd, cwd=ARENA_WS, stdout=f, stderr=subprocess.STDOUT, start_new_session=True)

        run_dir = None
        for _ in range(60):                                # the recorder creates the run folder at launch
            time.sleep(5)
            candidates = [d for d in out.glob(f"*_{planner}_*") if d.is_dir() and d.stat().st_mtime >= started]
            if candidates:
                run_dir = max(candidates, key=lambda d: d.stat().st_mtime)
                break
        if run_dir is None:
            log(f"{job['name']}: no run folder appeared; see {logfile}")
            return result
        result["run_dir"] = str(run_dir)
        log(f"{job['name']}: recording into {run_dir}")

        param = order[0]               # what task.scenario.file is (the launch argument)
        n_dirs, last_change, first_ep_wall = -1, time.time(), None
        cut: set[str] = set()          # episode dirs we cut short with reset_task
        status = "complete"
        while True:
            time.sleep(POLL_SECONDS)
            eps = read_episodes(run_dir)
            if len(eps) != n_dirs:
                n_dirs, last_change = len(eps), time.time()
                if eps and first_ep_wall is None:
                    first_ep_wall = time.time()
            for s in order:
                counts[s] = sum(1 for m in eps if m.get("scenario") == s and counted(m, {s}))
            done = sum(min(counts[s], want[s]) for s in order)
            remaining = [s for s in order if counts[s] < want[s]]
            running = eps[-1] if eps and eps[-1].get("outcome") == "running" else None

            if not remaining:
                log(f"{job['name']}: all {total} episodes complete")
                break
            if proc.poll() is not None:
                log(f"{job['name']}: the launch exited (code {proc.returncode}); see {logfile}")
                status = "failed"
                break
            if time.time() - last_change > float(job["stall_limit"]):
                log(f"{job['name']}: no new episode for {job['stall_limit']} s, giving up on this launch")
                status = "stalled"
                break

            # which scenario should the NEXT reset load?
            target = remaining[0]
            cur = running.get("scenario") if running else None
            cur_known = running is not None and running.get("scenario_source") != "launch"
            desired = target
            if cur == target and counts[target] == want[target] - 1 and len(remaining) > 1:
                desired = remaining[1]        # the running episode is the last one of `target`
            if desired != param and set_scenario_param(desired):
                log(f"{job['name']}: next reset loads {desired}")
                result["switches"].append({"t": datetime.now().isoformat(timespec="seconds"), "scenario": desired,
                                           "during": running["_dir"] if running else None})
                param = desired
            # an episode of a scenario that already has enough (the switch came too late): skip it
            if running and cur_known and cur in want and counts.get(cur, 0) >= want[cur] \
                    and running["_dir"] not in cut and param != cur:
                log(f"{job['name']}: {running['_dir']} is an extra episode of {cur}; resetting to {param}")
                cut.add(running["_dir"])
                result["extra_episodes"] += 1
                reset_task()

            # progress (once per finished episode)
            if done != result.get("_done_logged"):
                result["_done_logged"] = done
                if done and first_ep_wall:
                    per_ep = (time.time() - first_ep_wall) / max(done + len(cut), 1)
                    per_s = ", ".join(f"{s.removesuffix('.json')} {min(counts[s], want[s])}/{want[s]}"
                                      for s in order if counts[s] or s == target)
                    log(f"{job['name']}: {done}/{total} episodes, ~{fmt_minutes(per_ep * (total - done))} left "
                        f"[{per_s}]")
        result["status"] = status
        return result
    finally:
        result.pop("_done_logged", None)
        result["minutes"] = round((time.time() - started) / 60, 1)
        stop_sim()
        release_lock()


def merge_entry(job: dict, previous: dict | None, launch: dict) -> dict:
    """Manifest entry for a job after one launch, adding to an earlier (resumed) entry if there was one."""
    want = {s: int(k) for s, k in job["plan"]}
    prev_counts = {s: v.get("complete", 0) for s, v in ((previous or {}).get("scenarios") or {}).items()}
    scen = {s: {"requested": want[s], "complete": min(want[s], prev_counts.get(s, 0) + launch["counts"].get(s, 0))}
            for s in want}
    runs = list((previous or {}).get("runs") or []) + [{
        "run_dir": launch["run_dir"], "status": launch["status"], "minutes": launch["minutes"],
        "episodes": {s: c for s, c in launch["counts"].items() if c}, "extra_episodes": launch["extra_episodes"],
        "switches": launch["switches"]}]
    complete = all(v["complete"] >= v["requested"] for v in scen.values())
    entry = {"name": job["name"], "planner": job["planner"], "world": job["world"], "args": job["args"],
             "started": (previous or {}).get("started") or datetime.now().isoformat(timespec="seconds"),
             "status": "complete" if complete else launch["status"] if launch["status"] != "complete" else "partial",
             "episodes_requested": sum(v["requested"] for v in scen.values()),
             "episodes_complete": sum(v["complete"] for v in scen.values()),
             "minutes": round(sum(r["minutes"] for r in runs), 1),
             "run_dir": launch["run_dir"],         # the latest launch (all of them: `runs`)
             "scenarios": scen, "runs": runs}
    if not job["multi"]:
        entry["scenario"] = job["scenario"]       # the original single-scenario fields
    else:
        entry["order"] = job["order"]
    return entry


# ------------------------------------------------------------------------------------------------- main

def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("jobs_file", nargs="?", help="YAML jobs file (see the module docstring)")
    ap.add_argument("--out", help="output directory (overrides the jobs file's `out`)")
    ap.add_argument("--planner", nargs="+", help="one-off: run these planners (one job each)")
    ap.add_argument("--episodes", type=int, default=20)
    ap.add_argument("--scenario", default="hallway_tame.json")
    ap.add_argument("--world", default="school_hallway")
    ap.add_argument("--gui", action="store_true", help="show the Gazebo window (default: headless)")
    ap.add_argument("--launch-args", nargs="*", help="extra ros2 launch args, e.g. localization:=amcl")
    ap.add_argument("--resume", action="store_true",
                    help="skip jobs the manifest marks complete; rerun only missing scenarios/episodes")
    ap.add_argument("--dry-run", action="store_true", help="print the jobs and an ETA, run nothing")
    args = ap.parse_args()

    jobs, out = load_jobs(args)
    manifest_file = out / "batch_manifest.json"
    manifest = json.loads(manifest_file.read_text()) if manifest_file.exists() else []
    previous = {m["name"]: m for m in manifest}

    todo = []   # (job, plan for this launch, previous manifest entry or None)
    for job in jobs:
        prev = previous.get(job["name"]) if args.resume else None
        if prev and prev.get("status") == "complete":
            continue
        plan = job["plan"]
        if prev and prev.get("scenarios"):   # resume: only what is still missing
            plan = [[s, k - prev["scenarios"].get(s, {}).get("complete", 0)] for s, k in plan]
            plan = [[s, k] for s, k in plan if k > 0]
        elif prev:                           # an old-format entry: rerun the whole job
            prev = None
        if plan:
            todo.append((job, plan, prev))

    eta = sum(STARTUP_SECONDS + sum(k for _, k in plan) * EPISODE_SECONDS.get(j["planner"], EPISODE_SECONDS["default"])
              for j, plan, _ in todo)
    log(f"{len(todo)} job(s) -> {out}   (rough ETA {fmt_minutes(eta)}, longer if episodes time out)")
    for j, plan, _ in todo:
        what = (f"{len(plan)} scenarios" if j["multi"] else plan[0][0])
        print(f"    {j['name']:<16} {j['planner']:<8} {sum(k for _, k in plan):>3} episodes  {what}  "
              f"{' '.join(map(str, j['args']))}")
        if j["multi"]:
            print("        " + ", ".join(f"{s}x{k}" for s, k in plan))
    if args.dry_run:
        return

    def on_sigint(*_):
        log("interrupted: stopping the simulation")
        stop_sim()
        release_lock()
        sys.exit(130)
    signal.signal(signal.SIGINT, on_sigint)
    signal.signal(signal.SIGTERM, on_sigint)

    out.mkdir(parents=True, exist_ok=True)
    for job, plan, prev in todo:
        launch = run_job(job, out, plan)
        entry = merge_entry(job, prev, launch)
        manifest = [m for m in manifest if m["name"] != entry["name"]] + [entry]
        manifest_file.write_text(json.dumps(manifest, indent=2))
        per = ", ".join(f"{s} {v['complete']}/{v['requested']}" for s, v in entry["scenarios"].items())
        log(f"{job['name']}: {entry['status']}, {entry['episodes_complete']}/{entry['episodes_requested']} episodes, "
            f"{launch['minutes']} min  [{per}]")
        time.sleep(10)   # let ports / shared memory settle before the next launch
    log(f"all jobs done; manifest: {manifest_file}")


if __name__ == "__main__":
    main()
