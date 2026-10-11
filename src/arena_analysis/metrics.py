"""Episode metrics. Everything is computed from the recorded positions, so every planner is judged the same way.

Conventions
  * distance between robot and pedestrian = centre distance - robot radius - pedestrian radius ("clearance").
    clearance < 0 means the two discs overlap: a collision.
  * robot radius comes from the episode metadata (recorder --robot-radius, default 0.27 m for the Jackal).
  * pedestrian radius: the `radius` HuNav reports for each agent (from the scenario JSON, 0.35 m in hallway_tame),
    unless you pass human_radius=... to override it for everyone.
  * the recording rate is 10 Hz: a pedestrian at 1.3 m/s moves 13 cm between samples, so a very short graze can
    fall between two samples. Good enough for comparing planners; don't read too much into a single 2 cm overlap.
"""

import numpy as np
import pandas as pd

PERSONAL_SPACE = 0.5   # m of clearance; "intrusion" = closer than this (a common crowd-nav comfort threshold)


def robot_human_distances(robot: pd.DataFrame, humans: pd.DataFrame, robot_radius: float,
                          human_radius: float | None = None) -> pd.DataFrame:
    """Join every pedestrian sample with the robot sample of the same tick.

    Returns one row per (t, pedestrian) with columns:
      t, id, name, center_dist (m), clearance (m, = center_dist - radii), rx, ry (robot), hx, hy (pedestrian).
    """
    if humans.empty or robot.empty:
        return pd.DataFrame(columns=["t", "id", "name", "center_dist", "clearance", "rx", "ry", "hx", "hy"])
    # The recorder writes robot and pedestrian rows of one tick with the same t, so an exact merge works.
    # merge_asof (nearest) also tolerates files written by other tools.
    h = humans.sort_values("t")[["t", "id", "name", "x", "y", "radius"]].rename(columns={"x": "hx", "y": "hy"})
    r = robot.sort_values("t")[["t", "x", "y"]].rename(columns={"x": "rx", "y": "ry"})
    d = pd.merge_asof(h, r, on="t", direction="nearest", tolerance=0.06)
    d = d.dropna(subset=["rx"])
    rad = d["radius"] if human_radius is None else human_radius
    d["center_dist"] = np.hypot(d["hx"] - d["rx"], d["hy"] - d["ry"])
    d["clearance"] = d["center_dist"] - robot_radius - rad
    return d[["t", "id", "name", "center_dist", "clearance", "rx", "ry", "hx", "hy"]].reset_index(drop=True)


def collisions(dist: pd.DataFrame, dt_merge: float = 0.25) -> pd.DataFrame:
    """Contacts = runs of consecutive samples with clearance < 0 for the same pedestrian.

    Samples of the same pedestrian less than `dt_merge` s apart belong to one contact.
    Returns one row per contact: id, name, t_start, t_end, duration, min_clearance (most negative = deepest)."""
    hits = dist[dist["clearance"] < 0].sort_values(["id", "t"])
    rows = []
    for (pid, name), g in hits.groupby(["id", "name"]):
        new_contact = g["t"].diff().fillna(np.inf) > dt_merge
        for _, c in g.groupby(new_contact.cumsum()):
            rows.append({"id": pid, "name": name, "t_start": c["t"].iloc[0], "t_end": c["t"].iloc[-1],
                         "duration": c["t"].iloc[-1] - c["t"].iloc[0], "min_clearance": c["clearance"].min()})
    return pd.DataFrame(rows, columns=["id", "name", "t_start", "t_end", "duration", "min_clearance"])


def path_length(x, y) -> float:
    x, y = np.asarray(x), np.asarray(y)
    return float(np.hypot(np.diff(x), np.diff(y)).sum()) if len(x) > 1 else 0.0


def summarize(ep, human_radius: float | None = None) -> dict:
    """The numbers we compare planners on. Keys:

    episode, planner, scenario, outcome   from the metadata (outcome: success / timeout / aborted / reset / incomplete)
    success          outcome == "success" (Nav2 reported the goal reached)
    reached_goal     final robot position within goal_tolerance of the goal (independent of Nav2)
    duration         sim seconds from episode start to end
    time_to_goal     first sim time (since start) the robot was within goal_tolerance; NaN if never
    path_length      metres driven (sum of sample-to-sample distances)
    path_efficiency  straight-line start->goal distance / path_length (1 = perfectly straight); NaN if not reached
    mean_speed       path_length / duration; max_speed from finite differences
    final_goal_dist  metres from the goal at the last sample
    n_humans         pedestrians seen during the episode
    min_clearance    smallest robot-pedestrian clearance (m, negative = overlap); NaN without pedestrians
    min_center_dist  smallest centre-to-centre robot-pedestrian distance (m)
    collisions       number of robot-pedestrian contacts (see collisions())
    collision_time   total seconds in contact
    intrusion_time   seconds with any pedestrian closer than PERSONAL_SPACE (0.5 m clearance)
    min_wall_clearance  smallest robot-to-wall clearance (m, from walls.yaml); wall_collision = it went below 0
    cmd_tracking_err mean |cmd_vel_nav - cmd_vel| linear speed (how much the velocity smoother changed commands)
    """
    m, robot = ep.meta, ep.robot
    goal = ep.goal
    tol = float(m.get("goal_tolerance") or 0.5)
    out = {
        "episode": m.get("episode"),
        "planner": m.get("local_planner"),
        "scenario": m.get("scenario"),
        "outcome": m.get("outcome"),
        "success": m.get("outcome") == "success",
    }
    duration = m.get("duration")
    if duration is None and len(robot):
        duration = float(robot["t"].iloc[-1] - robot["t"].iloc[0])
    out["duration"] = duration

    length = path_length(robot["x"], robot["y"])
    out["path_length"] = length
    if goal is not None and len(robot):
        straight = float(np.hypot(goal[0] - robot["x"].iloc[0], goal[1] - robot["y"].iloc[0]))
        out["final_goal_dist"] = float(robot["goal_dist"].iloc[-1])
        out["reached_goal"] = out["final_goal_dist"] <= tol
        out["path_efficiency"] = straight / length if length > 0 and out["reached_goal"] else np.nan
        inside = robot[robot["goal_dist"] <= tol]
        out["time_to_goal"] = float(inside["t_rel"].iloc[0]) if len(inside) else np.nan
    else:
        out.update(path_efficiency=np.nan, final_goal_dist=np.nan, reached_goal=False, time_to_goal=np.nan)
    out["mean_speed"] = length / duration if duration else np.nan
    out["max_speed"] = float(robot["speed"].max()) if len(robot) else np.nan

    dist = ep.distances(human_radius)
    contacts = collisions(dist)
    out["n_humans"] = int(ep.humans["id"].nunique()) if len(ep.humans) else 0
    out["min_clearance"] = float(dist["clearance"].min()) if len(dist) else np.nan
    out["min_center_dist"] = float(dist["center_dist"].min()) if len(dist) else np.nan
    out["collisions"] = len(contacts)
    out["collision_time"] = _time_where(dist, dist["clearance"] < 0) if len(dist) else 0.0
    out["intrusion_time"] = _time_where(dist, dist["clearance"] < PERSONAL_SPACE) if len(dist) else 0.0

    wall = ep.world.wall_distance(robot["x"], robot["y"]) - ep.robot_radius
    out["min_wall_clearance"] = float(np.nanmin(wall)) if len(robot) and not np.all(np.isnan(wall)) else np.nan
    out["wall_collision"] = bool(out["min_wall_clearance"] < 0) if not np.isnan(out["min_wall_clearance"]) else False

    out["cmd_tracking_err"] = float((robot["cmd_nav_vx"] - robot["cmd_vx"]).abs().mean()) if len(robot) else np.nan
    return out


def _time_where(dist: pd.DataFrame, mask: pd.Series) -> float:
    """Seconds during which `mask` holds for at least one pedestrian (count of distinct ticks x sample period)."""
    ticks = np.sort(dist["t"].unique())
    if len(ticks) < 2:
        return 0.0
    period = float(np.median(np.diff(ticks)))
    return float(dist.loc[mask, "t"].nunique() * period)
