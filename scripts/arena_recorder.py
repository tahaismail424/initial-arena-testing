"""
Record every Arena episode to disk (CSV + JSON) for analysis in Python / notebooks.

While a simulation runs, this node samples at a fixed SIM-time rate (default 10 Hz):
  * the robot: true pose (x, y, yaw) and odometry twist          -> robot.csv
  * the controller output (cmd_vel_nav) and what reaches the robot (cmd_vel), on the same rows
  * every pedestrian: id, name, x, y, vx, vy, yaw, radius          -> humans.csv
  * the global plan, whenever it changes                          -> plan.csv
  * episode events (episode start, Nav2 goal status changes, end) -> events.csv
  * metadata (planner, scenario, world, start/goal, times, outcome) -> meta.json

One directory per episode:

  ~/arena_runs/<YYYYmmdd-HHMMSS>_<local_planner>_<scenario>/
      run.json                  launch arguments, recorder settings, git commit of this repo
      scenario.json, world/     copies of the launch scenario and the world's map files (runs are self-contained)
      scenarios/<file>.json     copy of every scenario file an episode used (several when hot-switching)
      world/scenario_maps/      the per-scenario maps those episodes used
      episode_000/  robot.csv humans.csv plan.csv events.csv meta.json
      episode_001/  ...

Load it with the `arena_analysis` package of this repo (docs/12). File format: docs/12.

How episodes are detected (read from task_generator/node.py and tasks/task_factory.py):
  /reset_start                    (Empty)  the task generator starts a reset -> close the running episode
  /task_generator_node/task_reset (Int16)  reset finished, robot teleported, goal sent -> new episode
                                           (data = episode index, 0 for the first one)
  /task_generator_node/scenario_info (String, latched; our arena-rosnav patch) JSON {"episode", "file", "map", ...}
                                           published by each reset BEFORE task_reset: the scenario (and map) that
                                           reset actually applied -> meta.json "scenario" / "map" (hot-switching)
  .../navigate_to_pose/_action/status      Nav2 goal status; SUCCEEDED in this episode = success
An episode ends with a reset when the goal is reached or after `timeout` sim seconds (task_generator.yaml).

Usually started for you by `arena-launch.sh --record` (it passes the launch arguments). By hand, in a
second terminal, BEFORE the first "Task Reset!" (it can't see an episode that started before it):

  cd ~/arena_ws && source arena.bash && unset FASTRTPS_DEFAULT_PROFILES_FILE
  python ~/Documents/Lyu_Lab/initial-arena-testing/scripts/arena_recorder.py \
      --launch-args local_planner:=dsrnn world:=school_hallway scenario:=hallway_tame.json

Needs Arena's ROS environment (rclpy, hunav_msgs), not this repo's uv venv. Only the standard library
otherwise (Arena's venv has no pandas), so the output is plain CSV.
"""

import argparse
import csv
import datetime
import json
import math
import os
import shutil
import signal
import subprocess
import time
from pathlib import Path

import rclpy
import rclpy.executors
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped, Twist
from hunav_msgs.msg import Agent, Agents
from nav_msgs.msg import Odometry, Path as PathMsg
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Empty, Int16, String
from rcl_interfaces.srv import GetParameters

REPO = Path(__file__).resolve().parent.parent
ARENA_WORLDS = Path.home() / "arena_ws/src/arena/simulation-setup/worlds"

STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: "unknown", GoalStatus.STATUS_ACCEPTED: "accepted",
    GoalStatus.STATUS_EXECUTING: "executing", GoalStatus.STATUS_CANCELING: "canceling",
    GoalStatus.STATUS_SUCCEEDED: "succeeded", GoalStatus.STATUS_CANCELED: "canceled",
    GoalStatus.STATUS_ABORTED: "aborted",
}

ROBOT_COLUMNS = ["t", "x", "y", "yaw", "odom_vx", "odom_vy", "odom_wz",
                 "cmd_nav_vx", "cmd_nav_wz", "cmd_vx", "cmd_wz"]
HUMAN_COLUMNS = ["t", "t_msg", "id", "name", "group_id", "x", "y", "yaw", "vx", "vy", "radius"]
PLAN_COLUMNS = ["t", "plan_idx", "i", "x", "y"]
EVENT_COLUMNS = ["t", "event", "detail"]


def yaw_of(q) -> float:
    """Heading (rad) of a geometry_msgs Quaternion, assuming the robot stays flat (roll = pitch = 0)."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def parse_launch_args(items: list[str]) -> dict:
    """['local_planner:=dsrnn', 'headless:=2'] -> {'local_planner': 'dsrnn', 'headless': '2'}. Later wins."""
    out = {}
    for item in items:
        if ":=" in item:
            key, value = item.split(":=", 1)
            out[key] = value
    return out


def downsample_path(points: list[tuple], spacing: float) -> list[tuple]:
    """Keep the first point, then only points >= `spacing` m from the last kept one, plus the last point.
    Nav2 plans have one point per costmap cell (5 cm); 25 cm is plenty for plotting and path-length analysis."""
    if len(points) <= 2:
        return points
    kept = [points[0]]
    for p in points[1:-1]:
        if math.dist(p, kept[-1]) >= spacing:
            kept.append(p)
    kept.append(points[-1])
    return kept


def git_commit(path: Path) -> str:
    try:
        return subprocess.run(["git", "-C", str(path), "rev-parse", "--short", "HEAD"],
                              capture_output=True, text=True, timeout=5).stdout.strip()
    except Exception:
        return ""


class CsvTable:
    """A CSV file with a header that we append rows to. Flushed often so a killed recorder loses ~nothing."""

    def __init__(self, path: Path, columns: list[str]):
        self.file = open(path, "w", newline="")
        self.writer = csv.writer(self.file)
        self.writer.writerow(columns)
        self.rows = 0

    def write(self, row: list):
        # floats -> 4 decimals (0.1 mm / 0.1 ms) keeps files ~3x smaller than repr() and loses nothing useful
        self.writer.writerow([f"{v:.4f}" if isinstance(v, float) else v for v in row])
        self.rows += 1

    def flush(self):
        self.file.flush()

    def close(self):
        self.file.close()


class Episode:
    """Everything that belongs to one episode directory."""

    def __init__(self, directory: Path, index: int, t_start: float, meta: dict):
        directory.mkdir(parents=True, exist_ok=True)
        self.dir = directory
        self.index = index
        self.t_start = t_start
        self.meta = meta
        self.robot = CsvTable(directory / "robot.csv", ROBOT_COLUMNS)
        self.humans = CsvTable(directory / "humans.csv", HUMAN_COLUMNS)
        self.plan = CsvTable(directory / "plan.csv", PLAN_COLUMNS)
        self.events = CsvTable(directory / "events.csv", EVENT_COLUMNS)
        self.n_plans = 0
        self.last_plan: list[tuple] | None = None
        self.goal_ids: set[bytes] = set()          # Nav2 goals that belong to this episode
        self.goal_status: dict[bytes, int] = {}    # their latest status
        self.last_xy: tuple | None = None

    def tables(self):
        return [self.robot, self.humans, self.plan, self.events]

    def write_meta(self):
        with open(self.dir / "meta.json", "w") as f:
            json.dump(self.meta, f, indent=2)


class Recorder:
    def __init__(self, node, args):
        self.node = node
        self.args = args
        self.launch = parse_launch_args(args.launch_args)
        ns = args.ns.rstrip("/")
        robot_ns = f"{ns}/{args.robot}"

        # --- latest messages (sampled by the timer) ---
        self.odom: Odometry | None = None
        self.cmd_nav = Twist()
        self.cmd = Twist()
        self.agents: list[Agent] = []
        self.agents_t = -1.0   # sim time the latest human_states arrived (its header stamp is WALL time, unusable)
        self.goal: PoseStamped | None = None
        self.plan_msg: PathMsg | None = None
        self.plan_dirty = False
        self.known_goal_ids: set[bytes] = set()   # every goal id Nav2 currently reports
        self._stale_goal_ids: set[bytes] = set()  # ... of which these were sent before the current episode

        self.episode: Episode | None = None
        self.n_episodes = 0
        # scenario actually applied per Arena episode index, from /task_generator_node/scenario_info
        self.scenario_infos: dict[int, dict] = {}
        self.scenarios_copied: set[str] = set()
        self.run_dir = self._make_run_dir()

        # --- subscriptions ---
        sub = node.create_subscription
        sub(Odometry, f"{robot_ns}/odom", self._on_odom, 10)
        sub(Twist, f"{robot_ns}/cmd_vel_nav", lambda m: setattr(self, "cmd_nav", m), 10)
        sub(Twist, f"{robot_ns}/cmd_vel", lambda m: setattr(self, "cmd", m), 10)
        sub(Agents, f"{ns}/human_states", self._on_agents, 10)
        sub(PoseStamped, f"{robot_ns}/goal_pose", lambda m: setattr(self, "goal", m), 10)
        sub(PathMsg, f"{robot_ns}/{args.plan_topic}", self._on_plan, 10)
        sub(GoalStatusArray, f"{robot_ns}/navigate_to_pose/_action/status", self._on_status,
            QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))  # action status is latched
        sub(Empty, "/reset_start", self._on_reset_start, 10)
        sub(Int16, f"{ns}/task_reset", self._on_task_reset, 10)
        sub(String, f"{ns}/scenario_info", self._on_scenario_info,
            QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        # fallback for an Arena without the scenario_info patch: ask the task generator for task.scenario.file
        self.param_client = node.create_client(GetParameters, f"{ns}/get_parameters")
        self.world_dir = self._world_dir(self.launch.get("world", ""))

        # Sampling is driven by odometry (~30 Hz, stamped with sim time): we take the first odom message after
        # each 1/rate grid point. A sim-time timer was tried first, but it fires irregularly (whenever /clock
        # jumps) and the odom it reads can be up to one message old, which made finite-difference speeds jump.
        self.period = 1.0 / args.rate
        self.next_sample: float | None = None
        self._last_flush = time.monotonic()
        node.get_logger().info(f"recording to {self.run_dir} (waiting for the next 'Task Reset!')")

    # ------------------------------------------------------------------ run directory
    def _make_run_dir(self) -> Path:
        stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
        planner = self.launch.get("local_planner", "dwb")
        scenario = Path(self.launch.get("scenario", "default.json")).stem
        run_dir = Path(self.args.out).expanduser() / f"{stamp}_{planner}_{scenario}"
        run_dir.mkdir(parents=True, exist_ok=True)

        world = self.launch.get("world", "")
        # Copy the scenario and the world's map so the run can be replayed even if the world is regenerated.
        # (Per-scenario maps are copied per episode, only the ones actually used: see _note_scenario.)
        world_dir = self._world_dir(world)
        scenario_file = world_dir / "scenarios" / self.launch.get("scenario", "default.json")
        if scenario_file.is_file():
            shutil.copy(scenario_file, run_dir / "scenario.json")
        if (world_dir / "map").is_dir():
            shutil.copytree(world_dir / "map", run_dir / "world", dirs_exist_ok=True,
                            ignore=shutil.ignore_patterns("scenario_maps"))

        run_meta = {
            "created": datetime.datetime.now().isoformat(timespec="seconds"),
            "launch_args": self.launch,
            "local_planner": planner,
            "global_planner": self.launch.get("global_planner", "navfn"),
            "world": world,
            "scenario": self.launch.get("scenario", "default.json"),   # the LAUNCH scenario
            "scenarios": [],              # every scenario file an episode used, in order of first use
            "robot": self.args.robot,
            "robot_radius": self.args.robot_radius,
            "rate_hz": self.args.rate,
            "plan_topic": self.args.plan_topic,
            "repo_commit": git_commit(REPO),
        }
        with open(run_dir / "run.json", "w") as f:
            json.dump(run_meta, f, indent=2)
        self.run_meta = run_meta
        return run_dir

    @staticmethod
    def _world_dir(world: str) -> Path:
        world_dir = REPO / "worlds" / world
        return world_dir if world_dir.is_dir() else ARENA_WORLDS / world

    def _write_run_meta(self):
        with open(self.run_dir / "run.json", "w") as f:
            json.dump(self.run_meta, f, indent=2)

    # ------------------------------------------------------------------ which scenario is this episode?
    def _on_scenario_info(self, msg: String):
        try:
            info = json.loads(msg.data)
        except ValueError:
            return
        if info.get("episode") is None:
            return
        self.scenario_infos[int(info["episode"])] = info
        for k in [k for k in self.scenario_infos if k < int(info["episode"]) - 5]:
            del self.scenario_infos[k]
        ep = self.episode
        if ep is not None and ep.index == int(info["episode"]) and ep.meta.get("scenario_source") != "scenario_info":
            self._label_episode(ep, info.get("file"), info.get("map"), info.get("map_loaded"), "scenario_info")

    def _label_episode(self, ep, scenario: str, map_rel, map_loaded, source: str):
        ep.meta.update({"scenario": scenario, "map": map_rel, "map_loaded": map_loaded, "scenario_source": source})
        ep.write_meta()
        self._note_scenario(scenario, map_rel)
        if source != "launch":
            self.node.get_logger().info(f"episode {ep.index}: scenario {scenario} (map: {map_rel or 'world map'}, "
                                        f"from {source})")

    def _note_scenario(self, scenario: str, map_rel):
        """Copy a scenario file (and its map) into the run the first time an episode uses it."""
        if not scenario or scenario in self.scenarios_copied:
            return
        self.scenarios_copied.add(scenario)
        src = self.world_dir / "scenarios" / scenario
        if src.is_file():
            (self.run_dir / "scenarios").mkdir(exist_ok=True)
            shutil.copy(src, self.run_dir / "scenarios" / scenario)
        if map_rel:
            map_yaml = self.world_dir / "map" / map_rel
            if map_yaml.is_file():
                dst = self.run_dir / "world" / map_rel
                dst.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy(map_yaml, dst)
                try:
                    import yaml as _yaml   # Arena's python has PyYAML (ROS needs it)
                    image = _yaml.safe_load(map_yaml.read_text())["image"]
                    shutil.copy(map_yaml.parent / image, dst.parent / Path(image).name)
                except Exception as e:  # noqa: BLE001
                    self.node.get_logger().warn(f"could not copy the map image of {map_rel}: {e}")
        self.run_meta["scenarios"].append(scenario)
        self._write_run_meta()

    def _query_scenario_param(self, ep):
        """Fallback when no scenario_info arrives (Arena without our patch): read task.scenario.file.
        Weaker guarantee: the parameter may have changed since the reset (hot-switching)."""
        if not self.param_client.service_is_ready():
            return
        future = self.param_client.call_async(GetParameters.Request(names=["task.scenario.file"]))

        def done(f):
            try:
                value = f.result().values[0].string_value
            except Exception:  # noqa: BLE001
                return
            if value and ep.meta.get("scenario_source") == "launch" and ep is self.episode:
                self._label_episode(ep, value, None, None, "parameter")
        future.add_done_callback(done)

    def now(self) -> float:
        """Current sim time in seconds."""
        return self.node.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------ episode lifecycle
    def _on_task_reset(self, msg: Int16):
        # A reset just finished: robot is at its start, the new goal is published. Normally /reset_start already
        # closed the previous episode; if we somehow missed it, close it here.
        if self.episode is not None:
            self._end_episode("reset")
        self._start_episode(int(msg.data))

    def _on_reset_start(self, _msg: Empty):
        if self.episode is not None:
            self._end_episode(None)

    def _start_episode(self, index: int):
        t = self.now()
        goal = None
        if self.goal is not None:
            p = self.goal.pose
            goal = [p.position.x, p.position.y, yaw_of(p.orientation)]
        meta = {
            **{k: self.run_meta[k] for k in ("local_planner", "global_planner", "world", "scenario",
                                             "robot", "robot_radius")},
            "episode": index,
            "episode_dir": f"episode_{self.n_episodes:03d}",
            "t_start": round(t, 3),
            "t_end": None,
            "duration": None,
            "start": None,                 # filled at the first sample (robot pose right after the reset)
            "goal": goal,
            "goal_tolerance": self.args.goal_tolerance,
            "timeout": self.args.timeout,
            "outcome": "running",
            "outcome_source": None,
            "final_goal_distance": None,
            "nav2_goals": 0,
            "nav2_final_status": None,
            "wall_start": datetime.datetime.now().isoformat(timespec="seconds"),
        }
        self.episode = Episode(self.run_dir / meta["episode_dir"], index, t, meta)
        # The scenario this reset applied: normally scenario_info (published before task_reset) is already here;
        # if it arrives later it relabels the episode in _on_scenario_info.
        info = self.scenario_infos.get(index)
        if info is not None:
            self._label_episode(self.episode, info.get("file"), info.get("map"), info.get("map_loaded"),
                                "scenario_info")
        else:
            meta.update({"map": None, "map_loaded": None, "scenario_source": "launch"})
            self._query_scenario_param(self.episode)
        # Goals Nav2 already knows about belong to earlier episodes (same trick as our robot_manager patch).
        self._stale_goal_ids = set(self.known_goal_ids)
        self.episode.events.write([t, "episode_start", f"index={index}"])
        self.episode.write_meta()
        self.plan_dirty = self.plan_msg is not None   # record the current plan once at the start
        self.next_sample = None
        self.n_episodes += 1
        self.node.get_logger().info(f"episode {index} started at t={t:.1f}s -> {self.episode.dir.name}")

    def _end_episode(self, outcome: str | None):
        """Close the running episode. outcome=None means 'work it out' (success / timeout / aborted / reset)."""
        ep, t = self.episode, self.now()
        self.episode = None
        duration = t - ep.t_start
        statuses = list(ep.goal_status.values())
        final_dist = None
        if ep.last_xy is not None and ep.meta["goal"] is not None:
            final_dist = math.dist(ep.last_xy, ep.meta["goal"][:2])

        if outcome is not None:
            source = "recorder"
        elif GoalStatus.STATUS_SUCCEEDED in statuses:
            outcome, source = "success", "nav2_succeeded"
        elif ep.meta["timeout"] and duration >= ep.meta["timeout"] - 1.0:
            outcome, source = "timeout", "duration"
        elif statuses and statuses[-1] == GoalStatus.STATUS_ABORTED:
            outcome, source = "aborted", "nav2_aborted"
        else:
            outcome, source = "reset", "unknown"   # e.g. a manual "Reset Task" from RViz
        ep.meta.update({
            "t_end": round(t, 3),
            "duration": round(duration, 3),
            "outcome": outcome,
            "outcome_source": source,
            "final_goal_distance": None if final_dist is None else round(final_dist, 3),
            "nav2_goals": len(ep.goal_ids),
            "nav2_final_status": STATUS_NAMES.get(statuses[-1]) if statuses else None,
            "samples": ep.robot.rows,
            "plans": ep.n_plans,
        })
        ep.events.write([t, "episode_end", outcome])
        for table in ep.tables():
            table.close()
        ep.write_meta()
        self.node.get_logger().info(
            f"episode {ep.index} ended: {outcome} after {duration:.1f}s ({ep.robot.rows} samples)")

    def shutdown(self):
        if self.episode is not None:
            self._end_episode("incomplete")   # the recorder (or the sim) stopped mid-episode

    # ------------------------------------------------------------------ callbacks
    def _on_status(self, msg: GoalStatusArray):
        self.known_goal_ids = {bytes(s.goal_info.goal_id.uuid) for s in msg.status_list}
        ep = self.episode
        if ep is None:
            return
        for s in msg.status_list:
            gid = bytes(s.goal_info.goal_id.uuid)
            if gid in self._stale_goal_ids:
                continue
            ep.goal_ids.add(gid)
            if ep.goal_status.get(gid) != s.status:
                ep.goal_status[gid] = s.status
                ep.events.write([self.now(), "nav2_goal", f"{gid.hex()[:8]}:{STATUS_NAMES.get(s.status, s.status)}"])

    def _on_plan(self, msg: PathMsg):
        self.plan_msg = msg
        self.plan_dirty = True

    def _write_plan(self, t: float):
        ep = self.episode
        points = downsample_path([(p.pose.position.x, p.pose.position.y) for p in self.plan_msg.poses],
                                 self.args.plan_spacing)
        self.plan_dirty = False
        if not points:
            return
        # Nav2 replans ~1x per second, mostly to the same path. Only store a plan that actually changed.
        if ep.last_plan is not None and len(ep.last_plan) == len(points) and \
                max(math.dist(a, b) for a, b in zip(ep.last_plan, points)) < 0.05:
            return
        ep.last_plan = points
        for i, (x, y) in enumerate(points):
            ep.plan.write([t, ep.n_plans, i, x, y])
        ep.n_plans += 1

    def _on_agents(self, msg: Agents):
        self.agents = list(msg.agents)
        self.agents_t = self.now()

    def _on_odom(self, msg: Odometry):
        self.odom = msg
        ep = self.episode
        if ep is None:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t < ep.t_start:
            return   # sent before the reset teleported the robot
        if self.next_sample is None or t >= self.next_sample:
            self._sample(t)
            self.next_sample = (math.floor(t / self.period + 1e-6) + 1) * self.period

    def _sample(self, t: float):
        """Write one row of robot.csv (pose at odom stamp t) plus the latest pedestrians, plan and commands."""
        ep = self.episode
        pose = self.odom.pose.pose
        x, y, yaw = pose.position.x, pose.position.y, yaw_of(pose.orientation)
        tw = self.odom.twist.twist
        ep.robot.write([t, x, y, yaw, tw.linear.x, tw.linear.y, tw.angular.z,
                        self.cmd_nav.linear.x, self.cmd_nav.angular.z, self.cmd.linear.x, self.cmd.angular.z])
        ep.last_xy = (x, y)
        if ep.meta["start"] is None:
            ep.meta["start"] = [round(x, 3), round(y, 3), round(yaw, 4)]
            if ep.meta["goal"] is None and self.goal is not None:
                p = self.goal.pose
                ep.meta["goal"] = [p.position.x, p.position.y, yaw_of(p.orientation)]
            ep.write_meta()

        # Pedestrians: the latest message (<= 1/30 s old). Skip messages from before the reset (old positions).
        agents = self.agents if self.agents_t >= ep.t_start else []
        for a in agents:
            if a.type != Agent.PERSON:
                continue
            ep.humans.write([t, self.agents_t, a.id, a.name, a.group_id, a.position.position.x, a.position.position.y,
                             float(a.yaw), a.velocity.linear.x, a.velocity.linear.y, float(a.radius)])

        if self.plan_dirty and self.plan_msg is not None:
            self._write_plan(t)

        if time.monotonic() - self._last_flush > 2.0:
            for table in ep.tables():
                table.flush()
            self._last_flush = time.monotonic()


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--out", default=os.environ.get("ARENA_RUNS", "~/arena_runs"),
                   help="parent directory for run directories (default: $ARENA_RUNS or ~/arena_runs)")
    p.add_argument("--launch-args", nargs="*", default=[],
                   help="the ros2 launch args of the run (key:=value), used for metadata and the directory name")
    p.add_argument("--robot", default="jackal")
    p.add_argument("--ns", default="/task_generator_node", help="task generator namespace")
    p.add_argument("--rate", type=float, default=10.0, help="samples per SIM second (default 10)")
    p.add_argument("--plan-topic", default="plan", help="global plan topic under the robot namespace")
    p.add_argument("--plan-spacing", type=float, default=0.25, help="keep plan points this far apart (m)")
    p.add_argument("--robot-radius", type=float, default=0.27,
                   help="robot radius (m) stored in the metadata and used for collision checks (Jackal ~0.27)")
    p.add_argument("--goal-tolerance", type=float, default=0.5,
                   help="stored in the metadata; analysis counts 'reached' within this distance (m)")
    p.add_argument("--timeout", type=float, default=600.0,
                   help="episode timeout in sim seconds (task_generator.yaml `timeout`), to label timeouts")
    p.add_argument("--exit-with-parent", action="store_true",
                   help="exit when the parent process dies (used by arena-launch.sh, whose exec'd "
                        "ros2 launch becomes our parent)")
    args = p.parse_args()

    rclpy.init()
    node = rclpy.create_node("arena_recorder")
    node.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
    recorder = Recorder(node, args)

    def on_sigterm(*_):
        raise KeyboardInterrupt   # `kill` -> same clean shutdown as Ctrl+C (closes the episode, writes meta.json)
    signal.signal(signal.SIGTERM, on_sigterm)
    parent = os.getppid()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            if args.exit_with_parent and os.getppid() != parent:
                node.get_logger().info("launch exited, stopping the recorder")
                break
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass   # Ctrl+C / kill / rclpy's own SIGINT handler: fall through to the clean shutdown
    finally:
        recorder.shutdown()
        print(f"[arena_recorder] {recorder.n_episodes} episode(s) in {recorder.run_dir}", flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
