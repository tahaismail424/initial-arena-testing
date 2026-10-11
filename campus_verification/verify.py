"""Verification metrics for recorded campus episodes (docs/14 §8): are the pedestrians behaving, does the stress
happen while the robot is there, and did the robot fail because of the policy or because of the scenario?

    uv run campus_verification/verify.py campus_verification/data            # every run under that folder
    uv run campus_verification/verify.py ~/arena_runs/<run> --out table.csv

One row per episode (outcome 'incomplete' skipped). Columns:
  robot      outcome, success, duration, time_to_goal, contacts (robot-person), min_clearance,
             min_obstacle_clear (robot centre to walls AND panels, minus robot radius; < 0 = touching),
             start_err (recorded start vs the scenario's start), freeze_time (social_metrics)
  humans     h_inside (samples with a pedestrian centre inside a wall/panel), h_max_pen (deepest body overlap
             with a wall/panel, m), pp_events / pp_min (pedestrian pairs with centre distance < 0.4 m: count of
             contacts, closest distance), stuck (pedestrians standing still >= 10 s away from their waypoints and
             not blocked by the robot), speed_ratio (median walking speed / desired_velocity)
  stress     per zone the robot passes (corridor / corner / intersection / door / plaza): how long the robot was
             in the zone and what the crowd did meanwhile (head-on passes, people coming round the corner,
             stream crossings, door passages, people within 2 m)
The first SKIP s of every episode are ignored for the pedestrian checks (HuNav's post-reset jump, docs/13).
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import shapely
import yaml
from scipy.spatial import cKDTree

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "scripts"))
sys.path.insert(0, str(REPO / "first_experiment"))
from generate_campus_scenario import Campus, free_space  # noqa: E402

import social_metrics  # noqa: E402
from arena_analysis import load_run  # noqa: E402

SKIP = 1.0                  # s ignored at the episode start for pedestrian checks
PP_CONTACT = 0.4            # m centre distance: pedestrian-pedestrian interpenetration (bodies are 2 x 0.35)
STUCK_WINDOW = 5.0          # s
STUCK_DISP = 0.25           # m moved within STUCK_WINDOW -> standing still
STUCK_MIN = 10.0            # s standing still to count as stuck
STANDERS = ("waiting", "standing")   # classes that stand still on purpose

# Robot windows per zone: (x0, y0, x1, y1) the robot must be inside.
WINDOWS = {
    "corridor": (6.0, 0.0, 22.0, 5.0),           # the narrowed section
    "corner": (24.0, 0.0, 35.0, 8.0),           # approaching and turning the blind corner
    "intersection": (30.0, 15.0, 35.0, 25.0),
    "door": (30.0, 25.0, 35.0, 33.0),
    "plaza": (20.0, 31.0, 48.0, 52.0),
}
TEMPLATE_ZONES = {"corridor_counterflow": ["corridor"], "blind_corner": ["corner"],
                  "intersection": ["intersection"], "door_exit": ["door"], "plaza_dense": ["plaza"],
                  "gauntlet": list(WINDOWS)}


def campus():
    return Campus(yaml.safe_load((REPO / "worlds" / "campus" / "zones.yaml").read_text()))


def scenario_of(ep):
    """The scenario JSON this episode ran: worlds/campus/scenarios/<meta scenario>, else the run's copy."""
    name = ep.meta.get("scenario") or ""
    p = REPO / "worlds" / "campus" / "scenarios" / name
    if p.is_file():
        return name, json.loads(p.read_text())
    return name, json.loads((ep.path.parent / "scenario.json").read_text())


def template_of(name):
    for t in TEMPLATE_ZONES:
        if name.startswith(t):
            return t
    return "other"


def crossings(a, b):
    """Indices i where a[i] < 0 <= a[i+1] ('up') or a[i] >= 0 > a[i+1] ('down') for one agent's series a - b."""
    d = np.asarray(a) - b
    s = np.sign(d)
    up = np.where((s[:-1] < 0) & (s[1:] >= 0))[0]
    down = np.where((s[:-1] >= 0) & (s[1:] < 0))[0]
    return up, down


def robot_window(robot, rect):
    x0, y0, x1, y1 = rect
    m = (robot["x"] >= x0) & (robot["x"] <= x1) & (robot["y"] >= y0) & (robot["y"] <= y1)
    if not m.any():
        return None
    t = robot.loc[m, "t"]
    return float(t.min()), float(t.max())


def human_checks(ep, scen, cmp):
    h = ep.humans
    h = h[h["t_rel"] >= SKIP].copy()
    out = {}
    if h.empty:
        return out, h
    h["cls"] = h["name"].str.rsplit("_", n=1).str[0]
    free = free_space(scen, cmp)
    pts = shapely.points(h["x"].to_numpy(), h["y"].to_numpy())
    inside = shapely.contains_xy(free, h["x"].to_numpy(), h["y"].to_numpy())
    dist = shapely.distance(free.boundary, pts)
    pen = np.where(inside, h["radius"].to_numpy() - dist, h["radius"].to_numpy() + dist)
    out["h_inside"] = int((~inside).sum())
    out["h_inside_agents"] = ",".join(sorted(h.loc[~inside, "name"].unique()))
    out["h_max_pen"] = float(pen.max())
    out["h_pen_gt_0.15"] = int((pen > 0.15).sum())

    # pedestrian-pedestrian interpenetration (per sample, pairs closer than PP_CONTACT)
    pairs, dmin = {}, np.inf
    for t, g in h.groupby("t"):
        xy = g[["x", "y"]].to_numpy()
        if len(xy) < 2:
            continue
        tree = cKDTree(xy)
        for i, j in tree.query_pairs(PP_CONTACT):
            key = tuple(sorted((g["name"].iloc[i], g["name"].iloc[j])))
            d = float(np.hypot(*(xy[i] - xy[j])))
            dmin = min(dmin, d)
            pairs.setdefault(key, []).append(t)
        dd, _ = tree.query(xy, k=2)
        dmin = min(dmin, float(dd[:, 1].min()))
    events = 0
    for ts in pairs.values():
        ts = np.sort(ts)
        events += 1 + int((np.diff(ts) > 0.5).sum())
    out["pp_events"] = events
    out["pp_seconds"] = float(sum(len(v) for v in pairs.values()) * 0.1)
    out["pp_min"] = dmin

    # stuck pedestrians and speeds
    wps = {a["name"]: np.array([w[:2] for w in a["waypoints"]]) for a in scen["obstacles"]["dynamic"]}
    desired = {a["name"]: a["desired_velocity"] for a in scen["obstacles"]["dynamic"]}
    r = ep.robot[["t", "x", "y"]].rename(columns={"x": "rx", "y": "ry"})
    stuck, ratios, speeds = [], [], []
    for name, g in h.groupby("name"):
        g = g.drop_duplicates("t").sort_values("t")
        if g["cls"].iloc[0] in STANDERS or len(g) < 20:
            continue
        sp = np.hypot(g["vx_fd"], g["vy_fd"]).to_numpy()
        moving = sp > 0.2
        if moving.sum() > 10:
            ratios.append(np.median(sp[moving]) / desired.get(name, np.nan))
            speeds.append(np.median(sp[moving]))
        # displacement over the past STUCK_WINDOW
        tt, xx, yy = g["t"].to_numpy(), g["x"].to_numpy(), g["y"].to_numpy()
        j = np.searchsorted(tt, tt - STUCK_WINDOW)
        disp = np.hypot(xx - xx[j], yy - yy[j])
        still = (disp < STUCK_DISP) & (tt - tt[j] >= STUCK_WINDOW - 0.3)
        if not still.any():
            continue
        grp = np.cumsum(np.r_[True, still[1:] != still[:-1]])
        gm = pd.merge_asof(g[["t", "x", "y"]], r.sort_values("t"), on="t", direction="nearest")
        for k in np.unique(grp[still]):
            idx = np.where((grp == k) & still)[0]
            dur = tt[idx[-1]] - tt[idx[0]] + STUCK_WINDOW
            if dur < STUCK_MIN:
                continue
            x, y = xx[idx].mean(), yy[idx].mean()
            near_wp = name in wps and np.min(np.hypot(*(wps[name] - [x, y]).T)) < 1.0
            near_robot = float(np.min(np.hypot(gm["x"].iloc[idx] - gm["rx"].iloc[idx],
                                               gm["y"].iloc[idx] - gm["ry"].iloc[idx]))) < 1.2
            stuck.append({"name": name, "t_rel": tt[idx[0]] - STUCK_WINDOW - ep.meta["t_start"], "dur": dur,
                          "x": round(x, 2), "y": round(y, 2), "near_wp": near_wp, "near_robot": near_robot})
    out["stuck"] = sum(1 for s in stuck if not s["near_robot"])
    out["stuck_by_robot"] = sum(1 for s in stuck if s["near_robot"])
    out["stuck_detail"] = "; ".join(f"{s['name']}@({s['x']},{s['y']}) t={s['t_rel']:.0f}s {s['dur']:.0f}s"
                                    f"{' wp' if s['near_wp'] else ''}{' robot' if s['near_robot'] else ''}"
                                    for s in stuck)
    out["speed_ratio"] = float(np.median(ratios)) if ratios else np.nan
    out["speed_ratio_p10"] = float(np.percentile(ratios, 10)) if ratios else np.nan
    out["walk_speed"] = float(np.median(speeds)) if speeds else np.nan
    return out, h


def zone_stress(ep, h, zone):
    """What the crowd did while the robot was in `zone`'s window."""
    r = ep.robot
    w = robot_window(r, WINDOWS[zone])
    out = {f"{zone}_time": np.nan}
    if w is None:
        return out
    t0, t1 = w
    out[f"{zone}_time"] = t1 - t0
    hw = h[(h["t"] >= t0) & (h["t"] <= t1)]
    rr = r[(r["t"] >= t0) & (r["t"] <= t1)][["t", "x", "y"]].rename(columns={"x": "rx", "y": "ry"})
    m = pd.merge_asof(hw.sort_values("t"), rr.sort_values("t"), on="t", direction="nearest", tolerance=0.06)
    m = m.dropna(subset=["rx"])
    near = m[np.hypot(m["x"] - m["rx"], m["y"] - m["ry"]) < 2.0 + 0.35 + ep.robot_radius]
    out[f"{zone}_people_within_2m"] = int(near["name"].nunique())
    if zone == "corridor":           # head-on passes: a person in the corridor goes from ahead of to behind the robot
        heads, overtakes = 0, 0
        for _, g in m[m["y"] < 5.2].groupby("name"):
            g = g.sort_values("t")
            up, down = crossings(g["x"].to_numpy(), g["rx"].to_numpy())
            for i in down:
                heads += g["vx_fd"].iloc[i] < 0
            overtakes += len(up)
        out["corridor_headon"] = int(heads)
        out["corridor_overtaken"] = int(overtakes)
    elif zone == "corner":           # people coming out of the riser round the corner (crossing y = 5 going down)
        n = 0
        for _, g in m.groupby("name"):
            g = g.sort_values("t")
            _, down = crossings(g["y"].to_numpy(), 5.0)
            n += sum(1 for i in down if 30.0 <= g["x"].iloc[i] <= 35.0)
        out["corner_arrivals"] = n
        out["corner_min_dist"] = float(np.hypot(m["x"] - m["rx"], m["y"] - m["ry"]).min()) if len(m) else np.nan
    elif zone == "intersection":     # stream crossings of the riser's centre line x = 32.5 inside the crossing zone
        n = 0
        for _, g in m[(m["y"] > 18) & (m["y"] < 23)].groupby("name"):
            g = g.sort_values("t")
            up, down = crossings(g["x"].to_numpy(), 32.5)
            n += len(up) + len(down)
        out["intersection_crossings"] = n
        out["intersection_rate_per_min"] = n / max(t1 - t0, 1e-6) * 60
    elif zone == "door":             # door passages (y = 30 inside the opening) while the robot was near the door
        ex, en = 0, 0
        for _, g in m[(m["x"] > 29.5) & (m["x"] < 35.5)].groupby("name"):
            g = g.sort_values("t")
            up, down = crossings(g["y"].to_numpy(), 30.0)
            ex += len(down)
            en += len(up)
        out["door_exits"] = ex
        out["door_entries"] = en
    elif zone == "plaza":
        per_t = near.groupby("t")["name"].nunique()
        out["plaza_mean_within_2m"] = float(per_t.reindex(m["t"].unique(), fill_value=0).mean()) if len(m) else 0
    return out


def episode_row(ep, cmp):
    name, scen = scenario_of(ep)
    tmpl = template_of(name)
    row = {"run": ep.path.parent.name, "episode": ep.index, "planner": ep.meta.get("local_planner"),
           "scenario": name, "template": tmpl}
    s = ep.summary()
    row.update({k: s.get(k) for k in ("outcome", "success", "reached_goal", "duration", "time_to_goal",
                                      "path_length", "collisions", "min_clearance", "n_humans")})
    r = ep.robot
    st = scen["robots"][0]["start"]
    row["start_err"] = float(math.dist(r[["x", "y"]].iloc[0], st[:2]))
    row["goal_err"] = float(math.dist(ep.goal[:2], scen["robots"][0]["goal"][:2])) if ep.goal else np.nan
    free = free_space(scen, cmp)
    d = shapely.distance(free.boundary, shapely.points(r["x"].to_numpy(), r["y"].to_numpy()))
    inside = shapely.contains_xy(free, r["x"].to_numpy(), r["y"].to_numpy())
    clear = np.where(inside, d, -d) - ep.robot_radius
    row["min_obstacle_clear"] = float(clear.min())
    row["obstacle_contact_s"] = float((clear < 0).sum() * 0.1)
    fr = social_metrics.freezes(ep)
    row["freeze_time"] = float(fr["duration"].sum()) if len(fr) else 0.0
    hc, h = human_checks(ep, scen, cmp)
    row.update(hc)
    if not h.empty:
        for z in TEMPLATE_ZONES.get(tmpl, []):
            row.update(zone_stress(ep, h, z))
    return row


def find_runs(paths):
    for p in paths:
        p = Path(p).expanduser()
        if (p / "run.json").is_file():
            yield p
        else:
            yield from sorted(q.parent for q in p.rglob("run.json"))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("paths", nargs="+", help="run folders or folders containing runs")
    ap.add_argument("--out", help="write the table as CSV")
    args = ap.parse_args()
    cmp = campus()
    rows = []
    for run_dir in find_runs(args.paths):
        run = load_run(run_dir)
        for ep in run.episodes:
            if ep.outcome == "incomplete":
                continue
            try:
                rows.append(episode_row(ep, cmp))
            except Exception as e:      # keep going; report the broken episode
                rows.append({"run": run_dir.name, "episode": ep.index, "error": repr(e)})
                print(f"{run_dir.name}/{ep.index}: {e!r}", file=sys.stderr)
    df = pd.DataFrame(rows)
    if args.out:
        df.to_csv(args.out, index=False)
    with pd.option_context("display.max_columns", None, "display.width", 250, "display.max_colwidth", 80):
        cols = [c for c in ["scenario", "planner", "episode", "outcome", "duration", "collisions", "min_clearance",
                            "min_obstacle_clear", "freeze_time", "h_inside", "h_max_pen", "pp_events", "pp_min",
                            "stuck", "stuck_by_robot", "speed_ratio"] if c in df]
        print(df[cols].round(2).to_string(index=False))
    return df


if __name__ == "__main__":
    main()
