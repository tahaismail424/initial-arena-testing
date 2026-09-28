"""
Sanity-check the pedestrians of a running Arena episode (HuNav).

Every --period seconds it prints, per agent class (name prefix, e.g. "runner_1" -> "runner"):
  n        how many agents of that class HuNav reports
  moving   how many are moving faster than 0.1 m/s
  speed    their mean speed vs the mean desired speed (m/s), measured from position change over the last
           --stuck-after SIM seconds (straight-line, so turns at waypoints make it read a bit low)
  RTF      real-time factor: sim seconds per wall second (>1 = faster than real time)
and for the whole crowd:
  min ped-ped gap   closest distance between two pedestrian centres (0 = walking THROUGH each other,
                    i.e. social forces off; a healthy crowd stays around >= 0.5 m)
  min ped-robot     closest pedestrian to the robot (m)
  stuck             agents that haven't moved 0.5 m in the last --stuck-after seconds

Run it in a second terminal while a simulation is up (Arena's ROS environment, not the uv venv):

  cd ~/arena_ws && source arena.bash && unset FASTRTPS_DEFAULT_PROFILES_FILE
  python ~/Documents/Lyu_Lab/initial-arena-testing/scripts/check_crowd.py
"""

import argparse
import itertools
import math
import time
from collections import defaultdict

import rclpy
from hunav_msgs.msg import Agents
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock


def agent_class(name: str) -> str:
    return name.rsplit("_", 1)[0] if name.rsplit("_", 1)[-1].isdigit() else name


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--ns", default="/task_generator_node")
    p.add_argument("--robot", default="jackal")
    p.add_argument("--period", type=float, default=15.0)
    p.add_argument("--stuck-after", type=float, default=20.0, help="SIM seconds without 0.5 m of progress")
    p.add_argument("--timeout", type=float, default=600.0, help="wall seconds")
    args = p.parse_args()

    rclpy.init()
    node = rclpy.create_node("check_crowd")
    state = {"agents": None, "robot": None, "sim_time": None}
    history = defaultdict(list)  # name -> [(sim time, x, y)]

    def on_clock(msg):
        # speeds are measured in SIM time: headless Gazebo can run faster (or slower) than real time
        state["sim_time"] = msg.clock.sec + msg.clock.nanosec * 1e-9

    def on_agents(msg):
        state["agents"] = msg.agents
        now = state["sim_time"]
        if now is None:
            return
        for a in msg.agents:
            h = history[a.name]
            h.append((now, a.position.position.x, a.position.position.y))
            while h and now - h[0][0] > args.stuck_after:
                h.pop(0)

    def on_odom(msg):
        state["robot"] = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    node.create_subscription(Clock, "/clock", on_clock, qos_profile_sensor_data)
    node.create_subscription(Agents, f"{args.ns}/human_states", on_agents, qos_profile_sensor_data)
    node.create_subscription(Odometry, f"{args.ns}/{args.robot}/odom", on_odom, qos_profile_sensor_data)

    t0 = last = time.monotonic()
    rtf_mark = (time.monotonic(), None)  # (wall, sim) at the previous printout
    while time.monotonic() - t0 < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
        if time.monotonic() - last < args.period:
            continue
        last = time.monotonic()
        rtf = None
        if state["sim_time"] is not None and rtf_mark[1] is not None and last > rtf_mark[0]:
            rtf = (state["sim_time"] - rtf_mark[1]) / (last - rtf_mark[0])
        rtf_mark = (last, state["sim_time"])
        agents = state["agents"]
        if not agents:
            print(f"[{last - t0:5.0f}s] no pedestrians reported yet on {args.ns}/human_states", flush=True)
            continue

        by_class = defaultdict(list)
        for a in agents:
            by_class[agent_class(a.name)].append(a)
        parts = []
        def speed(name):
            # HuNav's Agent.velocity field is not a reliable speed, so measure displacement over the history window
            h = history[name]
            return math.dist(h[0][1:], h[-1][1:]) / (h[-1][0] - h[0][0]) if len(h) > 1 and h[-1][0] > h[0][0] else 0.0

        for cls, members in sorted(by_class.items()):
            speeds = [speed(a.name) for a in members]
            moving = sum(s > 0.1 for s in speeds)
            desired = sum(a.desired_velocity for a in members) / len(members)
            parts.append(f"{cls} {moving}/{len(members)} @ {sum(speeds) / len(speeds):.2f}/{desired:.2f}")

        pos = {a.name: (a.position.position.x, a.position.position.y) for a in agents}
        gaps = [(math.dist(pos[a], pos[b]), a, b) for a, b in itertools.combinations(pos, 2)]
        min_gap = min(gaps) if gaps else None
        robot = state["robot"]
        near_robot = min((math.dist(xy, robot), n) for n, xy in pos.items()) if robot else None
        stuck = [n for n, h in history.items()
                 if len(h) > 1 and h[-1][0] - h[0][0] > 0.8 * args.stuck_after
                 and math.dist(h[0][1:], h[-1][1:]) < 0.5]

        print(f"[{last - t0:5.0f}s] {len(agents)} peds" + (f" RTF {rtf:.2f}" if rtf else "") + " | " + " | ".join(parts), flush=True)
        print(f"         min ped-ped gap {min_gap[0]:.2f} m ({min_gap[1]} / {min_gap[2]})"
              + (f" | min ped-robot {near_robot[0]:.2f} m ({near_robot[1]})" if near_robot else "")
              + f" | stuck: {', '.join(stuck) if stuck else 'none'}", flush=True)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
