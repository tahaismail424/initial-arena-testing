"""Run several campus scenarios in ONE Arena launch by switching the scenario file between episodes (docs/14 §9).

Stop-gap until scripts/arena_batch.py supports `scenarios: [globs]` (the infra work); same lock, same recorder.

    uv run campus_verification/hot_switch.py --planner dwb --episodes 1 --out campus_verification/data/dwb_a \
        door_exit_s0.json door_exit_s1.json blind_corner_s0.json

How: launch with warmup.json (a 4 m drive without pedestrians: the first reset after a launch makes HuNav
pedestrians jump several metres, docs/14), then whenever an episode of scenario S starts and it is S's last wanted episode,
set task.scenario.file to the next scenario, so the next reset loads it. Stops after the last episode ends, or
after --max-minutes (then the running episode is recorded as 'incomplete').
"""
from __future__ import annotations

import argparse
import re
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
LAUNCH = REPO / "scripts" / "arena-launch.sh"
STOP = REPO / "scripts" / "arena-stop.sh"
LOCK = Path("/tmp/arena-sim.lock")
ROS = "cd ~/arena_ws && source arena.bash >/dev/null 2>&1; unset FASTRTPS_DEFAULT_PROFILES_FILE; "


def log(msg):
    print(time.strftime("%H:%M:%S"), msg, flush=True)


def acquire_lock(owner):
    while True:
        try:
            LOCK.mkdir()
            (LOCK / "owner").write_text(owner + "\n")
            return
        except FileExistsError:
            holder = (LOCK / "owner").read_text().strip() if (LOCK / "owner").exists() else "?"
            log(f"lock held by {holder}; waiting")
            time.sleep(20)


def release_lock():
    subprocess.run(["bash", str(STOP)], capture_output=True)
    for _ in range(30):
        if subprocess.run(["pgrep", "-f", "[g]z sim"], capture_output=True).returncode != 0:
            break
        time.sleep(1)
    subprocess.run(["rm", "-rf", str(LOCK)])


def set_param(desc, value_yaml, tries=4):
    """SetParameters on the task generator (ros2 param set hangs against it). It can be slow or unresponsive during
    a reset, so each try is bounded and retried."""
    cmd = (ROS + "timeout 25 ros2 service call /task_generator_node/set_parameters "
           f"rcl_interfaces/srv/SetParameters \"{{parameters: [{value_yaml}]}}\"")
    for k in range(tries):
        try:
            r = subprocess.run(["bash", "-c", cmd], capture_output=True, text=True, timeout=40)
            if "successful=True" in r.stdout:
                log(f"{desc}: ok")
                return True
        except subprocess.TimeoutExpired:
            pass
        time.sleep(2)
    log(f"{desc}: FAILED after {tries} tries")
    return False


def set_scenario(name):
    return set_param(f"switch -> {name}", f"{{name: task.scenario.file, value: {{type: 4, string_value: {name}}}}}")


def set_timeout(seconds):
    return set_param(f"episode timeout -> {seconds} s",
                     f"{{name: timeout, value: {{type: 2, integer_value: {int(seconds)}}}}}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("scenarios", nargs="+")
    ap.add_argument("--planner", required=True)
    ap.add_argument("--episodes", type=int, default=1, help="episodes per scenario")
    ap.add_argument("--out", required=True)
    ap.add_argument("--max-minutes", type=float, default=28.0)
    ap.add_argument("--stall-minutes", type=float, default=12.0)
    ap.add_argument("--args", nargs="*", default=[], help="extra launch args")
    ap.add_argument("--timeout", type=int, help="episode timeout (sim s) instead of task_generator.yaml's 600")
    ap.add_argument("--no-warmup", action="store_true", help="launch directly with the first scenario")
    ap.add_argument("--attempts", type=int, default=3, help="relaunches after a crash (Gazebo segfaults happen)")
    a = ap.parse_args()

    out = Path(a.out).expanduser().resolve()
    (out / "logs").mkdir(parents=True, exist_ok=True)
    need = {sc: a.episodes for sc in a.scenarios}          # episodes still wanted per scenario
    t0 = time.time()
    for attempt in range(1, a.attempts + 1):
        todo = [sc for sc in a.scenarios if need[sc] > 0]
        if not todo or time.time() - t0 > a.max_minutes * 60:
            break
        crashed = run_launch(a, out, todo, need, t0)
        if not crashed:
            break
        log(f"launch {attempt} crashed; relaunching for the rest" if attempt < a.attempts else "giving up")
    left = [f"{sc} x{need[sc]}" for sc in a.scenarios if need[sc] > 0]
    if left:
        print("NOT RUN:", " ".join(left))
    return 0


def run_launch(a, out, todo, need, t0):
    """One launch: warmup, then every scenario in `todo` for need[sc] episodes. Decrements `need` for every
    episode that ends with a real outcome. Returns True if the launch died (e.g. Gazebo segfault)."""
    logf = out / "logs" / f"{time.strftime('%Y%m%d-%H%M%S')}_{a.planner}.log"
    queue = ([] if a.no_warmup else ["warmup.json"]) + todo
    wanted = {sc: need.get(sc, 1) for sc in queue}
    if not a.no_warmup:
        wanted["warmup.json"] = 1
    started, ep_scen = {}, {}
    acquire_lock(f"campus-agent hot_switch {a.planner}")
    crashed = False
    try:
        with open(logf, "w") as lf:
            proc = subprocess.Popen(["bash", str(LAUNCH), "--headless", f"--record={out}", "campus", "scenario",
                                     f"scenario:={queue[0]}", f"local_planner:={a.planner}", *a.args],
                                    stdout=lf, stderr=subprocess.STDOUT, cwd=Path.home() / "arena_ws")
        log(f"launched {a.planner} with {queue[0]} (log {logf})")
        pos, cur, last_event, done = 0, 0, time.time(), False
        pending_last, timeout_set = None, False
        while True:
            time.sleep(3)
            text = logf.read_text(errors="replace")
            new, pos = text[pos:], len(text)
            for m in re.finditer(r"arena_recorder\]: (episode (\d+): scenario (\S+)|episode (\d+) ended: (\S+) "
                                 r"after ([\d.]+)s)", new):
                last_event = time.time()
                if m.group(3):
                    scen = m.group(3)
                    ep_scen[m.group(2)] = scen
                    started[scen] = started.get(scen, 0) + 1
                    log(f"episode {m.group(2)} started: {scen} ({started[scen]}/{wanted.get(scen)})")
                    if scen == queue[cur] and started[scen] >= wanted[scen]:
                        if cur + 1 < len(queue):
                            cur += 1
                            set_scenario(queue[cur])
                        else:
                            pending_last = m.group(2)
                    if a.timeout and not timeout_set:
                        timeout_set = set_timeout(a.timeout)
                else:
                    log(f"episode {m.group(4)} ended: {m.group(5)} after {m.group(6)} s")
                    sc = ep_scen.get(m.group(4))
                    if m.group(5) != "incomplete" and sc in need:
                        need[sc] = max(0, need[sc] - 1)
                    if pending_last is not None and m.group(4) == pending_last:
                        done = True
            if done:
                log("all scenarios of this launch done")
                break
            if proc.poll() is not None:
                log("launch exited unexpectedly")
                crashed = True
                break
            if time.time() - t0 > a.max_minutes * 60:
                log("time budget reached")
                break
            if time.time() - last_event > a.stall_minutes * 60:
                log("stalled")
                crashed = True
                break
    finally:
        release_lock()
        log(f"stopped, lock released ({(time.time() - t0) / 60:.1f} min since start)")
    return crashed


if __name__ == "__main__":
    sys.exit(main())
