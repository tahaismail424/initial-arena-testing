"""
Watch a running Arena episode: where the robot BELIEVES it is vs where it REALLY is.

  believed pose = TF map -> <robot>/base_link   (what Nav2 plans with)
  true pose     = Gazebo model pose             (from `gz model -m <robot> -p`)
  error         = distance between the two      (~0 with localization:=ground_truth)

It also prints Nav2's goal status and stops when the goal is reached (or on timeout).

Run it in a SECOND terminal while a simulation is up. It needs Arena's ROS environment
(not this repo's uv venv), because it uses rclpy/tf2:

  cd ~/arena_ws && source arena.bash && unset FASTRTPS_DEFAULT_PROFILES_FILE
  python ~/Documents/Lyu_Lab/initial-arena-testing/scripts/check_run.py --timeout 900
"""

import argparse
import math
import re
import subprocess
import time

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

STATUS = {GoalStatus.STATUS_EXECUTING: "executing", GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
          GoalStatus.STATUS_ABORTED: "ABORTED", GoalStatus.STATUS_CANCELED: "canceled"}


def gazebo_pose(model: str):
    """True (x, y, yaw) of a model, parsed from the gz CLI. None if unavailable."""
    try:
        out = subprocess.run(["gz", "model", "-m", model, "-p"], capture_output=True, text=True, timeout=10).stdout
    except subprocess.TimeoutExpired:
        return None
    # output contains "Model: [95]" etc. too; keep only 3-number rows: [x y z] then [roll pitch yaw]
    rows = [r for r in re.findall(r"\[([-\d.e ]+)\]", out) if len(r.split()) == 3]
    if len(rows) < 2:
        return None
    x, y, _ = map(float, rows[0].split())
    yaw = float(rows[1].split()[2])
    return x, y, yaw


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--robot", default="jackal")
    p.add_argument("--ns", default="/task_generator_node", help="task generator namespace")
    p.add_argument("--period", type=float, default=10.0, help="seconds between printouts")
    p.add_argument("--timeout", type=float, default=900.0, help="give up after this many wall seconds")
    p.add_argument("--keep-going", action="store_true",
                   help="don't exit at the first reached goal (watch several episodes)")
    args = p.parse_args()

    rclpy.init()
    node = rclpy.create_node("check_run")
    node.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
    tf_buffer = Buffer()
    TransformListener(tf_buffer, node)

    status = {"value": None}

    def on_status(msg):
        if msg.status_list:
            status["value"] = msg.status_list[-1].status

    node.create_subscription(GoalStatusArray, f"{args.ns}/{args.robot}/navigate_to_pose/_action/status", on_status, 10)

    t0 = last = time.monotonic()
    while time.monotonic() - t0 < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
        if time.monotonic() - last < args.period:
            continue
        last = time.monotonic()

        try:
            tf = tf_buffer.lookup_transform("map", f"{args.robot}/base_link", Time(), Duration(seconds=1))
            t, q = tf.transform.translation, tf.transform.rotation
            believed = (t.x, t.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        except Exception as e:  # TF not ready yet
            believed = None
            print(f"[{last - t0:5.0f}s] no map->{args.robot}/base_link yet ({type(e).__name__})", flush=True)
        true = gazebo_pose(args.robot)

        if believed and true:
            err = math.hypot(believed[0] - true[0], believed[1] - true[1])
            dyaw = math.degrees(math.atan2(math.sin(believed[2] - true[2]), math.cos(believed[2] - true[2])))
            print(f"[{last - t0:5.0f}s] true x={true[0]:6.2f} y={true[1]:5.2f} | believed x={believed[0]:6.2f} "
                  f"y={believed[1]:5.2f} | error {err:5.2f} m, {dyaw:+5.1f} deg | nav: "
                  f"{STATUS.get(status['value'], status['value'])}", flush=True)

        if status["value"] == GoalStatus.STATUS_SUCCEEDED and not args.keep_going:
            print("GOAL REACHED", flush=True)
            break
    else:
        print("TIMEOUT", flush=True)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
