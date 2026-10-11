"""Social-navigation metrics for recorded Arena episodes (first experiment: DS-RNN vs DWB).

Builds on `arena_analysis` (docs/12): `ep.robot`, `ep.humans`, `ep.distances()`, `ep.summary()`.
Everything here is computed from recorded positions, so every planner is measured the same way.

Metric families (the usual ones in crowd-navigation papers):
  * task       success, timeouts, time to goal, path length / efficiency          (from ep.summary())
  * safety     contacts (count, when, where, who was moving), minimum clearance, time-to-collision (TTC)
  * comfort    proxemic zones (time spent in pedestrians' intimate / personal space)
  * freezing   the robot stopping in the crowd (the "freezing robot problem")
  * smoothness acceleration, jerk, turning, oscillation (how jerky / hesitant the motion is)

Caveats for this experiment:
  * HuNav pedestrians do NOT react to the robot (robot is invisible to them), so "pedestrian disturbance" metrics
    (how much people had to slow down or detour) would be meaningless here and are left out.
  * Right after each reset HuNav pedestrians jump for ~0.3 s (docs/13); SKIP_START seconds are ignored for
    TTC / freezing so that artifact can't create fake events.
"""
from __future__ import annotations

import numpy as np
import pandas as pd

from arena_analysis.metrics import collisions

SKIP_START = 1.0          # s ignored at the episode start (reset artifacts, robot accelerating from rest)
FREEZE_SPEED = 0.05       # m/s: slower than this counts as standing still
FREEZE_MIN = 1.0          # s: a stop must last at least this long to count as a freeze
TTC_DANGER = 1.0          # s: a time-to-collision below this counts as an imminent collision
CROWD_DIST = 2.0          # m clearance: a freeze with a pedestrian closer than this is a "crowd freeze"
# Proxemic zones (Hall, 1966), applied to the clearance between robot and pedestrian bodies:
ZONES = [("contact", -np.inf, 0.0), ("intimate", 0.0, 0.45), ("personal", 0.45, 1.2), ("social", 1.2, 3.6)]


def _period(t: pd.Series) -> float:
    """Sample period (s) of a time column (the recorder samples at 10 Hz)."""
    return float(np.median(np.diff(np.sort(t.unique())))) if t.nunique() > 1 else 0.1


def contact_events(ep, human_radius: float | None = None) -> pd.DataFrame:
    """One row per robot-pedestrian contact, with context: when (t_rel), where (x, y), how fast each party was
    moving at the start of the contact, and whether the robot was moving into the person.

    robot_toward / human_toward: velocity component (m/s) along the line from that party to the other at contact
    start. Positive = moving toward the other. A contact with robot_toward > 0.1 is one the robot actively drove
    into; with robot_toward <= 0.1 the robot was (nearly) stationary or moving away and got hit.
    """
    dist = ep.distances(human_radius)
    c = collisions(dist)
    if c.empty:
        extra = {k: pd.Series(dtype=float) for k in
                 ["t_rel", "x", "y", "robot_speed", "human_speed", "robot_toward", "human_toward"]}
        return c.assign(**extra, robot_active=pd.Series(dtype=bool))
    robot = ep.robot.set_index("t")
    humans = ep.humans
    rows = []
    for _, e in c.iterrows():
        r = robot.iloc[robot.index.get_indexer([e["t_start"]], method="nearest")[0]]
        h = humans[humans["id"] == e["id"]]
        h = h.iloc[(h["t"] - e["t_start"]).abs().argmin()]
        to_h = np.array([h["x"] - r["x"], h["y"] - r["y"]])
        to_h /= max(np.linalg.norm(to_h), 1e-6)
        rv, hv = np.array([r["vx"], r["vy"]]), np.array([h["vx_fd"], h["vy_fd"]])
        rows.append({"t_rel": e["t_start"] - ep.meta["t_start"], "x": r["x"], "y": r["y"],
                     "robot_speed": float(np.linalg.norm(rv)), "human_speed": float(np.linalg.norm(hv)),
                     "robot_toward": float(rv @ to_h), "human_toward": float(hv @ -to_h)})
    out = pd.concat([c.reset_index(drop=True), pd.DataFrame(rows)], axis=1)
    out["robot_active"] = out["robot_toward"] > 0.1
    return out


def min_ttc(ep, human_radius: float | None = None) -> pd.Series:
    """Per tick: the smallest time-to-collision (s) over all pedestrians, assuming everyone keeps their current
    velocity. 0 = already touching; inf = nobody on a collision course. Indexed by t_rel.

    TTC solves |p + v*t| = R for the first t >= 0, with p = pedestrian minus robot position, v = pedestrian minus
    robot velocity, R = sum of radii.
    """
    r = ep.robot[["t", "x", "y", "vx", "vy"]].rename(columns={"x": "rx", "y": "ry", "vx": "rvx", "vy": "rvy"})
    h = ep.humans[["t", "id", "x", "y", "vx_fd", "vy_fd", "radius"]]
    d = pd.merge_asof(h.sort_values("t"), r.sort_values("t"), on="t", direction="nearest", tolerance=0.06)
    d = d.dropna(subset=["rx"])
    R = ep.robot_radius + (d["radius"] if human_radius is None else human_radius)
    px, py = d["x"] - d["rx"], d["y"] - d["ry"]
    vx, vy = d["vx_fd"] - d["rvx"], d["vy_fd"] - d["rvy"]
    a = vx ** 2 + vy ** 2
    b = 2 * (px * vx + py * vy)
    c = px ** 2 + py ** 2 - R ** 2
    disc = b ** 2 - 4 * a * c
    with np.errstate(invalid="ignore", divide="ignore"):
        t_hit = (-b - np.sqrt(disc)) / (2 * a)          # first root = when the bodies first touch
    ttc = np.where(c <= 0, 0.0, np.where((disc >= 0) & (a > 1e-9) & (t_hit >= 0), t_hit, np.inf))
    d = d.assign(ttc=ttc)
    s = d.groupby("t")["ttc"].min()
    s.index = s.index - ep.meta["t_start"]
    s.index.name = "t_rel"
    return s[s.index >= SKIP_START]


def freezes(ep, speed: float = FREEZE_SPEED, min_duration: float = FREEZE_MIN) -> pd.DataFrame:
    """Intervals where the robot stood still (speed < `speed`) for at least `min_duration` s while NOT at the goal.
    Only stops AFTER the robot first started moving count: after a reset the robot waits ~1-2 s for Nav2's plan,
    which is not a freeze. One row per freeze: t_rel start, duration, position, nearest pedestrian clearance at the
    start, and `near_people` (clearance < CROWD_DIST). A freeze next to people is the classic "freezing robot
    problem"; one far from people is something else (e.g. pinned against a wall, a planner failure)."""
    r = ep.robot
    tol = float(ep.meta.get("goal_tolerance") or 0.5)
    moving = r["speed"] >= speed
    started = moving.cummax()                          # True from the first moment the robot moved
    still = (r["speed"] < speed) & (r["goal_dist"] > tol) & started
    rows = []
    groups = (still != still.shift()).cumsum()
    near = ep.distances().groupby("t")["clearance"].min()
    for _, g in r[still].groupby(groups[still]):
        dur = g["t"].iloc[-1] - g["t"].iloc[0] + _period(r["t"])
        if dur >= min_duration:
            t0 = g["t"].iloc[0]
            rows.append({"t_rel": g["t_rel"].iloc[0], "duration": dur, "x": g["x"].iloc[0], "y": g["y"].iloc[0],
                         "nearest_clearance": float(near.iloc[near.index.get_indexer([t0], method="nearest")[0]])
                         if len(near) else np.nan})
    out = pd.DataFrame(rows, columns=["t_rel", "duration", "x", "y", "nearest_clearance"])
    out["near_people"] = out["nearest_clearance"] < CROWD_DIST
    return out


def proxemics(ep, human_radius: float | None = None) -> dict:
    """Seconds the robot spent with its CLOSEST pedestrian in each proxemic zone (by body clearance)."""
    dist = ep.distances(human_radius)
    if dist.empty:
        return {f"time_{name}": 0.0 for name, _, _ in ZONES}
    closest = dist.groupby("t")["clearance"].min()
    dt = _period(dist["t"])
    out = {f"time_{name}": float(((closest >= lo) & (closest < hi)).sum() * dt) for name, lo, hi in ZONES}
    # the same as fractions of the episode: a slower robot spends more SECONDS near people just by taking longer
    total = len(closest) * dt
    out.update({f"frac_{name}": out[f"time_{name}"] / total if total else np.nan for name, _, _ in ZONES})
    return out


def smoothness(ep) -> dict:
    """How smooth / decisive the motion was (lower = smoother):
    mean_abs_accel  mean |d speed / dt|            (m/s^2)
    mean_abs_jerk   mean |d accel / dt|            (m/s^3)
    mean_abs_omega  mean |turn rate|               (rad/s, from the true heading)
    heading_per_m   total |heading change| / path length  (rad/m; 0 = dead straight, "path irregularity")
    omega_flips_per_min  sign changes of the policy's turn command (|w| > 0.1) per minute (oscillation / weaving)
    """
    r = ep.robot[ep.robot["t_rel"] >= SKIP_START]
    if len(r) < 4:
        return {}
    dt = np.diff(r["t"].to_numpy())
    acc = np.diff(r["speed"].to_numpy()) / dt
    jerk = np.diff(acc) / dt[1:]
    yaw = np.unwrap(r["yaw"].to_numpy())
    dyaw = np.diff(yaw)
    length = float(np.hypot(np.diff(r["x"]), np.diff(r["y"])).sum())
    w = r["cmd_nav_wz"].to_numpy()
    s = np.sign(w[np.abs(w) > 0.1])
    minutes = (r["t"].iloc[-1] - r["t"].iloc[0]) / 60
    return {"mean_abs_accel": float(np.mean(np.abs(acc))), "mean_abs_jerk": float(np.mean(np.abs(jerk))),
            "mean_abs_omega": float(np.mean(np.abs(dyaw / dt))),
            "heading_per_m": float(np.abs(dyaw).sum() / length) if length > 0 else np.nan,
            "omega_flips_per_min": float((np.diff(s) != 0).sum() / minutes) if minutes > 0 else np.nan}


def episode_metrics(ep, human_radius: float | None = None) -> dict:
    """All metrics for one episode as a flat dict (one row of the experiment table)."""
    m = ep.summary(human_radius) if human_radius is not None else ep.summary()
    contacts = contact_events(ep, human_radius)
    ttc = min_ttc(ep, human_radius)
    fr = freezes(ep)
    dt = _period(ep.robot["t"])
    m.update({
        "collided": len(contacts) > 0,
        "first_contact_t": float(contacts["t_rel"].min()) if len(contacts) else np.nan,
        "robot_active_contacts": int(contacts["robot_active"].sum()) if len(contacts) else 0,
        "min_ttc": float(ttc.min()) if len(ttc) else np.inf,
        "time_ttc_danger": float((ttc < TTC_DANGER).sum() * dt),
        "n_freezes": len(fr),
        "freeze_time": float(fr["duration"].sum()) if len(fr) else 0.0,
        "froze": len(fr) > 0,
        "crowd_freezes": int(fr["near_people"].sum()) if len(fr) else 0,
        "froze_in_crowd": bool(fr["near_people"].any()) if len(fr) else False,
    })
    m.update(proxemics(ep, human_radius))
    m.update(smoothness(ep))
    return m


# ----------------------------------------------------------------------------------------------- aggregation

def wilson(k: int, n: int, z: float = 1.96) -> tuple[float, float]:
    """95% Wilson confidence interval for a rate k/n (better than +-1.96*sqrt(p(1-p)/n) for small n or p near 0/1)."""
    if n == 0:
        return (np.nan, np.nan)
    p = k / n
    centre = (p + z * z / (2 * n)) / (1 + z * z / n)
    half = z * np.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / (1 + z * z / n)
    return (max(0.0, centre - half), min(1.0, centre + half))


def bootstrap_ci(x, stat=np.mean, n_boot: int = 5000, seed: int = 0) -> tuple[float, float]:
    """95% bootstrap confidence interval of `stat` (resample episodes with replacement)."""
    x = np.asarray(pd.Series(x).dropna(), dtype=float)
    x = x[np.isfinite(x)]
    if len(x) < 2:
        return (np.nan, np.nan)
    rng = np.random.default_rng(seed)
    boots = [stat(rng.choice(x, len(x))) for _ in range(n_boot)]
    return tuple(np.percentile(boots, [2.5, 97.5]))


RATES = ["success", "collided", "froze", "froze_in_crowd", "wall_collision"]
MEANS = ["time_to_goal", "path_length", "path_efficiency", "mean_speed", "collisions", "min_clearance", "min_ttc",
         "time_ttc_danger", "time_intimate", "time_personal", "frac_intimate", "frac_personal", "freeze_time", "mean_abs_jerk", "heading_per_m",
         "omega_flips_per_min"]


def compare(table: pd.DataFrame, by: str = "planner") -> pd.DataFrame:
    """Planner-by-planner summary: rates as 'p [lo, hi]' (Wilson) and means as 'mean [lo, hi]' (bootstrap)."""
    out = {}
    for planner, g in table.groupby(by):
        col = {"episodes": str(len(g))}
        for k in RATES:
            if k in g:
                hits = int(g[k].sum())
                lo, hi = wilson(hits, len(g))
                col[f"{k} rate"] = f"{hits / len(g):.2f} [{lo:.2f}, {hi:.2f}]"
        for k in MEANS:
            if k in g:
                v = g[k].replace([np.inf, -np.inf], np.nan)
                lo, hi = bootstrap_ci(v)
                col[k] = f"{v.mean():.2f} [{lo:.2f}, {hi:.2f}]" if v.notna().any() else "n/a"
        out[planner] = col
    return pd.DataFrame(out)
