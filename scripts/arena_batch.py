"""Run a queue of recorded Arena simulations, one after another ("batch scheduler").

Each JOB = one Arena launch (one planner, one scenario) that records N episodes, then stops. Jobs run strictly
sequentially: one simulation uses ~5.5 of this VM's 8 cores, and two would share ROS/Gazebo topics.

Usage
-----
From a jobs file (recommended; keeps the experiment definition next to its results):

    uv run scripts/arena_batch.py first_experiment/jobs.yaml
    uv run scripts/arena_batch.py first_experiment/jobs.yaml --dry-run        # show what would run, with an ETA
    uv run scripts/arena_batch.py first_experiment/jobs.yaml --resume         # skip jobs the manifest marks done

Or a quick one-off from the command line:

    uv run scripts/arena_batch.py --planner dsrnn dwb --episodes 5 --out ~/arena_runs/quick_test

Jobs file format (YAML):

    out: data                       # where runs go (relative to this file); one run folder per job
    defaults:                       # apply to every job unless the job overrides them
      world: school_hallway
      scenario: hallway_tame.json
      episodes: 20
      headless: true
      args: []                      # extra `ros2 launch` arguments, e.g. ["global_planner:=smac"]
    jobs:
      - planner: dsrnn
      - planner: dwb
      - {planner: dwb, name: dwb_amcl, args: ["localization:=amcl"]}

What it does per job
--------------------
1. takes the shared simulator lock (/tmp/arena-sim.lock; waits if another batch or agent holds it)
2. stops any leftover Arena processes, then launches `arena-launch.sh --record=<out> ...` (log: <out>/logs/<job>.log)
3. watches the recorder's run folder: when folder number N appears, episodes 0..N-1 are complete
   (episode N is cut short and recorded as "incomplete"; the analysis skips it)
4. stops the simulation, releases the lock, appends the result to <out>/batch_manifest.json
A job is abandoned (status "stalled") if no new episode starts within `stall_limit` seconds (default 900; the
episode timeout is 600 sim s), and "failed" if the launch never creates a run folder.

Ctrl+C stops the current simulation cleanly and releases the lock.
"""
from __future__ import annotations

import argparse
import json
import os
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

DEFAULTS = {"world": "school_hallway", "scenario": "hallway_tame.json", "task_mode": "scenario", "episodes": 20,
            "headless": True, "args": [], "stall_limit": 900}


def log(msg: str):
    print(f"[{datetime.now():%H:%M:%S}] {msg}", flush=True)


def fmt_minutes(seconds: float) -> str:
    return f"{seconds / 60:.0f} min" if seconds < 5400 else f"{seconds / 3600:.1f} h"


# ------------------------------------------------------------------------------------------------- jobs

def load_jobs(args) -> tuple[list[dict], Path]:
    """Jobs (each a complete dict of settings) and the output directory, from a jobs file or the command line."""
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
        jobs.append(job)
    if not jobs:
        sys.exit("no jobs (give a jobs file or --planner ...)")
    return jobs, out


def launch_command(job: dict, out: Path) -> list[str]:
    cmd = ["bash", str(LAUNCH)]
    if job["headless"]:
        cmd.append("--headless")
    cmd += [f"--record={out}", job["world"], job["task_mode"]]
    if job["task_mode"] == "scenario":
        cmd.append(f"scenario:={job['scenario']}")
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


# ------------------------------------------------------------------------------------------------- one job

def run_job(job: dict, out: Path) -> dict:
    """Run one job to completion. Returns its manifest entry."""
    planner, n = job["planner"], int(job["episodes"])
    (out / "logs").mkdir(parents=True, exist_ok=True)
    logfile = out / "logs" / f"{job['name']}.log"
    entry = {"name": job["name"], "planner": planner, "scenario": job.get("scenario"), "world": job["world"],
             "episodes_requested": n, "args": job["args"], "started": datetime.now().isoformat(timespec="seconds")}

    acquire_lock(f"arena_batch:{job['name']}")
    try:
        stop_sim()   # nothing left over from earlier runs
        started = time.time()
        cmd = launch_command(job, out)
        log(f"{job['name']}: launching ({n} episodes): {' '.join(cmd[2:])}")
        with open(logfile, "w") as f:
            # own process group, so Ctrl+C on the batch doesn't hit the sim directly (stop_sim() handles it)
            subprocess.Popen(cmd, cwd=ARENA_WS, stdout=f, stderr=subprocess.STDOUT, start_new_session=True)

        run_dir = None
        for _ in range(60):                                # the recorder creates the run folder at launch
            time.sleep(5)
            candidates = [d for d in out.glob(f"*_{planner}_*") if d.is_dir() and d.stat().st_mtime >= started]
            if candidates:
                run_dir = max(candidates, key=lambda d: d.stat().st_mtime)
                break
        if run_dir is None:
            log(f"{job['name']}: no run folder appeared; see {logfile}")
            return {**entry, "status": "failed", "run_dir": None, "episodes_complete": 0,
                    "minutes": round((time.time() - started) / 60, 1)}
        log(f"{job['name']}: recording into {run_dir}")

        count, last_change, status = -1, time.time(), "complete"
        first_episode_t = None
        while True:
            time.sleep(15)
            now_count = len(list(run_dir.glob("episode_*")))
            if now_count != count:
                count, last_change = now_count, time.time()
                if count == 1:
                    first_episode_t = time.time()
                done = max(count - 1, 0)
                if 0 < done < n and first_episode_t:
                    per_ep = (time.time() - first_episode_t) / done
                    log(f"{job['name']}: {done}/{n} episodes done, ~{fmt_minutes(per_ep * (n - done))} left")
            if count > n:
                log(f"{job['name']}: {n} episodes complete")
                break
            if time.time() - last_change > float(job["stall_limit"]):
                log(f"{job['name']}: no new episode for {job['stall_limit']} s, giving up on this job")
                status = "stalled"
                break
        return {**entry, "status": status, "run_dir": str(run_dir), "episodes_complete": min(max(count - 1, 0), n),
                "minutes": round((time.time() - started) / 60, 1)}
    finally:
        stop_sim()
        release_lock()


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
    ap.add_argument("--resume", action="store_true", help="skip jobs whose manifest entry is 'complete'")
    ap.add_argument("--dry-run", action="store_true", help="print the jobs and an ETA, run nothing")
    args = ap.parse_args()

    jobs, out = load_jobs(args)
    manifest_file = out / "batch_manifest.json"
    manifest = json.loads(manifest_file.read_text()) if manifest_file.exists() else []
    finished = {m["name"] for m in manifest if m.get("status") == "complete"}
    if args.resume:
        jobs = [j for j in jobs if j["name"] not in finished]

    eta = sum(STARTUP_SECONDS + int(j["episodes"]) * EPISODE_SECONDS.get(j["planner"], EPISODE_SECONDS["default"])
              for j in jobs)
    log(f"{len(jobs)} job(s) -> {out}   (rough ETA {fmt_minutes(eta)}, longer if episodes time out)")
    for j in jobs:
        print(f"    {j['name']:<16} {j['planner']:<8} {j['episodes']:>3} episodes  {j.get('scenario', '')}  "
              f"{' '.join(map(str, j['args']))}")
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
    for job in jobs:
        entry = run_job(job, out)
        manifest = [m for m in manifest if m["name"] != entry["name"]] + [entry]
        manifest_file.write_text(json.dumps(manifest, indent=2))
        log(f"{job['name']}: {entry['status']}, {entry['episodes_complete']} episodes, {entry['minutes']} min")
        time.sleep(10)   # let ports / shared memory settle before the next launch
    log(f"all jobs done; manifest: {manifest_file}")


if __name__ == "__main__":
    main()
