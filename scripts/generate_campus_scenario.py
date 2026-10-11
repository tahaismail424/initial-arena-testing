"""
Procedurally generate stress-test scenarios for the campus world (scripts/generate_campus.py).
Guide: docs/14-campus-world.md. This file is boilerplate: the plumbing is done, and ONE template
(corridor_counterflow) is worked out as an example. The other templates are TODOs for you.

A scenario = TEMPLATE + PARAMETERS + SEED:
  * a template is a function that builds one kind of stress situation (counterflow corridor, doorway, ...):
    the robot's task, the pedestrians and their routes, and wall panels that adjust the geometry;
  * its parameters (corridor width, crowd size, speeds, ...) are sampled from ranges in the spec YAML;
  * the seed makes everything reproducible: same spec + same seed -> identical scenario JSON.

  spec   : scenarios/campus/<spec>.yaml           which templates, parameter ranges, HuNav defaults
  world  : worlds/campus/zones.yaml               zone rectangles, door, anchor points (from generate_campus.py)
  out    : worlds/campus/scenarios/<template>_s<seed>.json      <- what Arena loads (one per template x seed)
           worlds/campus/previews/<template>_s<seed>.png         <- check by eye
           worlds/campus/scenarios/manifest.csv                 <- the sampled parameters of every scenario,
                                                                   join it with recorded outcomes for analysis

Usage (from the repo root):
  uv run scripts/generate_campus_scenario.py scenarios/campus/stress.yaml                  # seed from the spec
  uv run scripts/generate_campus_scenario.py scenarios/campus/stress.yaml --seeds 0 29     # 30 seeds per template
  uv run scripts/generate_campus_scenario.py scenarios/campus/stress.yaml --only corridor_counterflow --link
"""

import argparse
import copy
import csv
import json
import math
import random
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import yaml  # noqa: E402
from PIL import Image  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
from generate_hallway import REPO_ROOT, link_into_arena  # noqa: E402

# Thin wall panels for per-scenario geometry (simulation-setup/entities/obstacles/static/panel_*; docs/14 §5):
# 0.2 m thick, 2 m tall, origin at the footprint centre. panel_x<L> is L m long along x, panel_y<L> along y.
# NEVER rotate a panel (pos yaw must stay 0): the HuNav plugin uses the UNROTATED box, so a rotated panel
# becomes a phantom obstacle that traps pedestrians. Pick the x or y variant instead (wall() does).
PANEL_THICKNESS = 0.2
PANEL_LENGTHS = {4.0: "4", 2.0: "2", 1.0: "1", 0.5: "05"}      # length (m) -> model name suffix


# ============================================================================================ building blocks

class Campus:
    """The layout, loaded from zones.yaml. zone("corridor") -> (x0, y0, x1, y1); anchor("plaza_far") -> (x, y)."""

    def __init__(self, data):
        self.zones = {k: tuple(v["rect"]) for k, v in data["zones"].items()}
        self.door = data["door"]
        self.anchors = {k: tuple(v) for k, v in data["anchors"].items()}

    def zone(self, name):
        return self.zones[name]

    def anchor(self, name):
        return self.anchors[name]


def sample(value, rng):
    """Spec values: [lo, hi] -> uniform float, {"int": [lo, hi]} -> integer, {"choice": [...]} -> one of them,
    anything else -> used as is."""
    if isinstance(value, list) and len(value) == 2 and all(isinstance(v, (int, float)) for v in value):
        return rng.uniform(*value)
    if isinstance(value, dict) and "int" in value:
        return rng.randint(*value["int"])
    if isinstance(value, dict) and "choice" in value:
        return rng.choice(value["choice"])
    return value


def deep_merge(base, override):
    out = copy.deepcopy(base)
    for k, v in (override or {}).items():
        out[k] = deep_merge(out.get(k, {}), v) if isinstance(v, dict) else v
    return out


class Builder:
    """Collects one scenario's robot task, pedestrians and panels. Templates call its methods."""

    def __init__(self, spec, rng):
        self.spec, self.rng = spec, rng
        self.robot = None
        self.agents, self.static = [], []
        self.next_group = 1          # HuNav group ids: -1 = alone, >= 0 = group

    def set_robot(self, start, goal, yaw=0.0):
        """Robot start/goal (x, y). Keep the goal >= 3 m from walls (the DS-RNN end-wall trap)."""
        self.robot = {"start": [round(start[0], 2), round(start[1], 2), yaw],
                      "goal": [round(goal[0], 2), round(goal[1], 2), 0.0]}

    def add_pedestrian(self, cls, start, waypoints, speed, group=-1, hunav=None):
        """One HuNav pedestrian walking `waypoints` (looping back to the first with cyclic_goals).
        cls: a short class name (ends up in the agent's name, e.g. 'oncoming_3'; handy in analysis).
        hunav: per-pedestrian overrides of the spec's `defaults` (e.g. {"behavior": {"type": 6}})."""
        fields = deep_merge(self.spec["defaults"], hunav)
        # Arena's HuNav adapter asserts float types (an int force factor in the YAML, e.g. 12, makes it drop the
        # agent with "must be of type 'float'": the episode silently runs without that pedestrian)
        for k in ("radius", "goal_radius"):
            if k in fields:
                fields[k] = float(fields[k])
        fields["behavior"] = {k: float(v) if k.endswith("_factor") or k in ("vel", "dist") else v
                              for k, v in fields.get("behavior", {}).items()}
        n = sum(a["class"] == cls for a in self.agents)
        self.agents.append({
            "name": f"{cls}_{n}",
            "model": self.spec.get("model", "actor1"),
            "type": 1,                                   # hunav_msgs/Agent PERSON (must be the integer 1)
            "pos": [round(start[0], 2), round(start[1], 2), 0.0],
            "waypoints": [[round(x, 2), round(y, 2), 0.0] for x, y in waypoints],
            "desired_velocity": round(speed, 2),
            "group_id": group,
            "class": cls,                                # ignored by Arena, used by our analysis
            **fields,
        })

    def new_group(self):
        g, self.next_group = self.next_group, self.next_group + 1
        return g

    def wall(self, x0, y0, x1, y1):
        """A thin wall (0.2 m) along the axis-aligned line (x0, y0) -> (x1, y1), centred on it, built from panels.
        Long walls use 4 m panels, short ones shorter panels; panels may overlap (and may overlap shell walls)."""
        horizontal = abs(x1 - x0) >= abs(y1 - y0)
        if abs(x1 - x0) > 1e-6 and abs(y1 - y0) > 1e-6:
            raise ValueError("wall() must be horizontal or vertical (panels can't be rotated)")
        a, c = (min(x0, x1), max(x0, x1)) if horizontal else (min(y0, y1), max(y0, y1))
        length = max(c - a, 0.5)
        size = max([L for L in PANEL_LENGTHS if L <= length] or [0.5])
        n = math.ceil(length / size - 1e-9)
        model = f"panel_{'x' if horizontal else 'y'}{PANEL_LENGTHS[size]}"
        centres = [(a + c) / 2] if n == 1 else [a + size / 2 + i * (c - a - size) / (n - 1) for i in range(n)]
        for m in centres:
            x, y = (m, y0) if horizontal else (x0, m)
            # Name by MODEL: across scenario switches Arena only moves an obstacle whose name it already knows,
            # it never re-spawns it, so a name reused with a different model would keep the old geometry.
            k = sum(o["model"] == model for o in self.static)
            self.static.append({"name": f"{model}_{k}", "model": model, "pos": [round(x, 3), round(y, 3), 0.0]})

    def block_rect(self, x0, y0, x1, y1):
        """Block off the axis-aligned rectangle (nobody can enter it): its outline in panels, drawn just INSIDE
        the rectangle so the free space ends exactly at its edges. Use it to narrow a corridor (a strip along a
        wall) or to make a door (one block on each side of the gap). A rectangle thinner than one panel (0.2 m)
        becomes a single wall. Blocks may overlap the shell walls."""
        t = PANEL_THICKNESS / 2
        if x1 - x0 <= PANEL_THICKNESS:
            self.wall((x0 + x1) / 2, y0, (x0 + x1) / 2, y1)
        elif y1 - y0 <= PANEL_THICKNESS:
            self.wall(x0, (y0 + y1) / 2, x1, (y0 + y1) / 2)
        else:
            self.wall(x0, y0 + t, x1, y0 + t)          # bottom
            self.wall(x0, y1 - t, x1, y1 - t)          # top
            self.wall(x0 + t, y0, x0 + t, y1)          # left
            self.wall(x1 - t, y0, x1 - t, y1)          # right

    def jitter(self, pt, amount=0.4, axis="xy"):
        """Random offset so no two pedestrians aim at exactly the same waypoint (they would overlap: Arena
        hard-codes HuNav's goal force to 20, much stronger than the social force). axis="x" or "y" jitters only
        along that axis, e.g. along a corridor, so lanes don't drift into its walls."""
        dx = self.rng.uniform(-amount, amount) if "x" in axis else 0.0
        dy = self.rng.uniform(-amount, amount) if "y" in axis else 0.0
        return (pt[0] + dx, pt[1] + dy)

    # ----- placing people on closed routes -------------------------------------------------------------------

    def is_clear(self, pt, min_sep=0.9, robot_sep=1.5):
        """True if pt is >= min_sep from every pedestrian start so far and >= robot_sep from the robot's start
        and goal (nobody spawns on the robot or parks on its goal)."""
        if any(math.dist(pt, a["pos"][:2]) < min_sep for a in self.agents):
            return False
        if self.robot and any(math.dist(pt, self.robot[k][:2]) < robot_sep for k in ("start", "goal")):
            return False
        return True

    @staticmethod
    def point_on_loop(route, i, t):
        """The point a fraction t along segment i of the closed route (segment i: route[i] -> route[i+1])."""
        (xa, ya), (xb, yb) = route[i], route[(i + 1) % len(route)]
        return (xa + t * (xb - xa), ya + t * (yb - ya))

    @staticmethod
    def rotate_from(route, i):
        """Waypoints in walking order for someone on segment i: next is route[i+1], looping round."""
        return route[i + 1:] + route[:i + 1]

    def phase_on_loop(self, route, segments=None, min_sep=0.9, check=None, tries=300):
        """A uniformly random position (by length) along the closed route, or along only the given segment
        indices. Populations on loops are then spread evenly from t = 0, so a zone is busy at ANY time the robot
        arrives, not only at the start. Returns (i, t) for point_on_loop / rotate_from. The point must pass
        is_clear(); `check(i, t) -> bool` can add a veto (e.g. for the other members of a group)."""
        segs = list(range(len(route))) if segments is None else list(segments)
        lengths = [math.dist(route[k], route[(k + 1) % len(route)]) for k in segs]
        for _ in range(tries):
            s = self.rng.uniform(0, sum(lengths))
            for i, L in zip(segs, lengths):
                if s <= L:
                    break
                s -= L
            t = min(s / L, 1.0) if L > 0 else 0.0
            if self.is_clear(self.point_on_loop(route, i, t), min_sep) and (check is None or check(i, t)):
                return i, t
        raise RuntimeError("couldn't place a pedestrian on its loop (too crowded?)")

    def add_on_loop(self, cls, route, speed, group=-1, hunav=None, phase=None):
        """add_pedestrian() at a random phase of the closed route (or at phase=(i, t)). Returns (i, t)."""
        i, t = phase or self.phase_on_loop(route)
        self.add_pedestrian(cls, self.point_on_loop(route, i, t), self.rotate_from(route, i), speed, group, hunav)
        return i, t

    def scenario(self):
        if self.robot is None:
            raise ValueError("template didn't call set_robot()")
        return {"robots": [self.robot],
                "obstacles": {"static": self.static, "dynamic": self.agents, "interactive": []}}


# ================================================================================================ templates
# Each template: f(campus, b, p) where b is a Builder and p the SAMPLED parameters (a dict). It fills b.
# Register it in TEMPLATES at the bottom. Keep pedestrian routes inside their zone so templates don't interfere.

def corridor_counterflow(campus, b, p):
    """Narrow corridor with two-way pedestrian traffic: people coming at the robot (counterflow) and people
    overtaking it.

    Geometry: the shell corridor is 5 m wide (y 0..5). Panels along both long walls between x = 6 and 22 leave a
    clear width p["width"] in the middle (the robot's start x = 2 and goal x = 26 are in the wide part).
    Robot: corridor_west (2, 2.5) -> corridor_east (26, 2.5), i.e. through the whole narrow section.
    Crowd: p["n_oncoming"] people who start on their westbound leg (coming at the robot) and p["n_with"] who start
    on their eastbound leg; everyone loops between a west turnaround (x 4.5..6) and an east one (x 22.5..24.5),
    randomised per person so they don't pile up. People keep right: westbound in the north half, eastbound in the
    south half (lanes >= 0.6 m apart, so oncoming people pass instead of meeting exactly head-on).
    Timing: initial positions are spread uniformly along the starting leg, so oncoming people meet the robot from
    the first seconds on, and with looping routes the corridor stays busy for the whole episode.
    """
    x0, y0, x1, y1 = campus.zone("corridor")
    narrow_x0, narrow_x1 = 6.0, 22.0
    mid = (y0 + y1) / 2
    gap = min(p["width"], (y1 - y0) - 0.2)
    side = ((y1 - y0) - gap) / 2                                   # how much each side gets narrowed
    if side > 0.05:
        b.block_rect(narrow_x0, y0, narrow_x1, y0 + side)        # a strip along the south wall
        b.block_rect(narrow_x0, y1 - side, narrow_x1, y1)        # ... and along the north wall

    b.set_robot(campus.anchor("corridor_west"), campus.anchor("corridor_east"))

    lane_lo, lane_hi = mid - gap / 2 + 0.45, mid + gap / 2 - 0.45  # body radius 0.35 + 0.1 off the panels
    if lane_hi - mid < 0.3:
        raise ValueError("corridor_counterflow: width < 1.5 m leaves no room for two lanes")

    def lanes():
        # keep right: westbound (facing -x) walks the north half, eastbound the south half. Lanes >= 0.6 m apart
        # also in the narrowest corridor: two people on the SAME line meet exactly head-on, the social force then
        # has no sideways component and (goal force 20 >> social) they walk through each other.
        return b.rng.uniform(mid + 0.3, lane_hi), b.rng.uniform(lane_lo, mid - 0.3)

    for cls, n, first_leg in (("oncoming", p["n_oncoming"], "west"), ("with_robot", p["n_with"], "east")):
        for _ in range(n):
            speed = b.rng.uniform(*p["speed"])
            y_w, y_e = lanes()
            lo, hi = b.rng.uniform(4.5, 6.0), b.rng.uniform(narrow_x1 + 0.5, narrow_x1 + 2.5)
            route = [(lo, y_w), (lo, y_e), (hi, y_e), (hi, y_w)]   # west leg = segment 3, east leg = segment 1
            seg = 3 if first_leg == "west" else 1
            b.add_on_loop(cls, route, speed, phase=b.phase_on_loop(route, segments=[seg]))


def door_exit(campus, b, p):
    """Doorway against an exiting crowd: can the robot hold back while people come through a narrow door, then go?

    Geometry: the shell opening between the riser and the plaza (x 30..35 at y = 30) is closed down to a door of
    width p["door_width"] centred at x = 32.5 + p["door_offset"], with one 1 m-deep panel block on each side
    (y 29.5..30.5; clipped so each block is >= 0.5 m).
    Robot: the riser at y = 24.5 (6 m below the door) -> plaza_center (34, 41), i.e. up and through the door.
    Crowd, p["wave"] = true (standalone): p["n_exiting"] people start at homes spread over the southern plaza
    (1.8-11 m from the door) and all head for the door at t = 0: down through the door (waypoints above, in, and
    below the door, so nobody catches on the frame), down the riser past the robot, through the intersection and
    out along the crossing corridor to a spot 4-18 m east or west, where they mill around (10 points within 2 m,
    ~20 m of walking), then walk back up through the door to their home. The exit wave fills the door for
    ~10-30 s: DS-RNN (1 m/s) meets it in the door, DWB (0.26 m/s, ~23 s to the door) meets its tail plus everyone
    walking down the riser at it. The return wave reaches the door after ~45 s and goes the robot's way. Exits and
    returns are separated in time on purpose: head-on traffic in a single-file door makes HuNav people walk
    through each other (they cannot take turns). (cyclic_goals false, i.e. stopping at the destination, crashes
    the HuNav Gazebo plugin once an agent has reached its last goal, so everyone loops.)
    Crowd, p["wave"] = false (gauntlet): everyone loops plaza home -> door -> riser below the door -> door ->
    home, at random phases, so the door has two-way traffic whenever the robot arrives. This needs two lanes
    (door_width >= 1.7 m: lanes w - 1.0 >= 0.7 m apart, out on the -x side, in on the +x side).
    p["n_waiting"] people stand beside the door on the plaza side (clutter at the door).
    """
    rx0, ry0, rx1, ry1 = campus.zone("riser")
    zx0, zy0, zx1, zy1 = campus.zone("crossing")
    dcx, dcy = campus.door["center"]
    shell = campus.door["shell_width"]
    wave = p.get("wave", True)
    w = max(1.2, min(p["door_width"], shell - 1.0))
    if not wave and w < 1.7:
        raise ValueError("door_exit with wave: false (two-way traffic) needs door_width >= 1.7")
    cx = dcx + p.get("door_offset", 0.0)
    cx = min(max(cx, dcx - shell / 2 + 0.5 + w / 2), dcx + shell / 2 - 0.5 - w / 2)   # >= 0.5 m of block each side
    b.block_rect(dcx - shell / 2, dcy - 0.5, cx - w / 2, dcy + 0.5)
    b.block_rect(cx + w / 2, dcy - 0.5, dcx + shell / 2, dcy + 0.5)

    b.set_robot((dcx, dcy - 5.5), campus.anchor("plaza_center"))

    y_above, y_below = dcy + 1.4, dcy - 1.4
    px0, py0, px1, py1 = campus.zone("plaza")
    off = w / 2 - 0.5 if w >= 1.7 else 0.0
    x_out, x_in = cx - off, cx + off          # going out (facing -y) keep right = -x side
    jit = 0.03 if w >= 1.7 else min(0.1, (w - 1.0) / 2)

    def home():
        for _ in range(500):
            q = (b.rng.uniform(px0 + 2.0, px1 - 2.0), b.rng.uniform(py0 + 1.8, py0 + (11.0 if wave else 8.5)))
            if b.is_clear(q, robot_sep=2.0):
                return q
        raise RuntimeError("no free home in the plaza")

    dests = []                                 # wave: final standing spots in the crossing corridor, >= 1 m apart

    def dest():
        for _ in range(500):
            side = b.rng.choice([-1, 1])
            q = (dcx + side * b.rng.uniform(4.0, 18.0), b.rng.uniform(zy0 + 0.8, zy1 - 0.8))
            if zx0 + 1.0 <= q[0] <= zx1 - 1.0 and all(math.dist(q, d) >= 1.0 for d in dests):
                dests.append(q)
                return q
        raise RuntimeError("no free destination in the crossing corridor")

    def sink():
        return (b.rng.uniform(rx0 + 0.8, rx1 - 0.8), b.rng.uniform(ry1 - 6.2, ry1 - 3.0))

    for _ in range(p["n_exiting"]):
        speed = b.rng.uniform(*p["speed"])
        h = home()
        xo = x_out + b.rng.uniform(-jit, jit)
        out_leg = [(xo, y_above), (xo, dcy), (xo, y_below)]
        if wave:
            def mid():                                                                   # in the intersection
                return (b.rng.uniform(rx0 + 1.0, rx1 - 1.0), b.rng.uniform(zy0 + 1.5, zy1 - 1.5))
            d = dest()
            mill = [(min(max(d[0] + b.rng.uniform(-2, 2), zx0 + 1.0), zx1 - 1.0),
                     b.rng.uniform(zy0 + 0.8, zy1 - 0.8)) for _ in range(10)]
            xi = x_in + b.rng.uniform(-jit, jit)
            back = [mid(), (xi, y_below), (xi, dcy), (xi, y_above), h]
            b.add_pedestrian("exiting", h, out_leg + [mid(), d] + mill + back, speed)
        else:
            s1 = sink()
            s2 = next(q for q in (sink() for _ in range(100)) if math.dist(q, s1) > 1.0)
            xi = x_in + b.rng.uniform(-jit, jit)
            route = out_leg + [s1, s2, (xi, y_below), (xi, dcy), (xi, y_above), h]
            b.add_on_loop("exiting", route, speed)

    for _ in range(p.get("n_waiting", 0)):
        for _ in range(200):
            side = b.rng.choice([-1, 1])
            q = (cx + side * b.rng.uniform(w / 2 + 0.9, w / 2 + 2.2), b.rng.uniform(dcy + 1.3, dcy + 2.3))
            if b.is_clear(q, min_sep=1.0):
                break
        b.add_pedestrian("waiting", q, [(q[0] + 0.3, q[1]), q], 0.5)


def blind_corner(campus, b, p):
    """Blind corner: people come round the L-turn between the corridor and the riser with little warning.

    Geometry: the corridor's east end (x 30..35, y 0..5) turns north into the riser; the wall corner at (30, 5)
    hides the riser from a robot in the corridor (and vice versa). p["riser_width"] < 4.5 narrows the riser above
    the corner (panels along its east wall, y 5.5..16.5); 4.5..5 leaves it at the full 5 m.
    Robot: corridor_mid (18, 2.5) -> the riser at y = 12 (centred in the possibly narrowed riser).
    Crowd: p["n"] people on a loop: down the riser on its west side (keep right), round the INSIDE of the corner
    (1 m from the hidden wall corner, exactly where the robot turns), west along the corridor's north half to
    x 24..27, back east along the south half, up the riser's east side, cross over at the top (y 14..17, >= 2 m
    above the robot's goal). Speeds p["speed"] up to 2 m/s = little reaction time.
    Timing: start positions are uniform along the loop, so someone comes round the corner every few seconds for
    the whole episode (loop ~40 m, i.e. one person per ~40 / (n * speed) s).
    """
    rx0, ry0, rx1, ry1 = campus.zone("riser")
    cx0, cy0, cx1, cy1 = campus.zone("corridor")
    w = min(p.get("riser_width", rx1 - rx0), rx1 - rx0)
    y_narrow_top = 16.5
    if (rx1 - rx0) - w < 0.5:          # a strip thinner than 0.5 m changes nothing: keep the full width
        w = rx1 - rx0
    else:
        b.block_rect(rx0 + w, cy1 + 0.5, rx1, y_narrow_top)

    goal_y = campus.anchor("riser_mid")[1]
    b.set_robot(campus.anchor("corridor_mid"), (rx0 + w / 2, goal_y))

    for _ in range(p["n"]):
        speed = b.rng.uniform(*p["speed"])
        x_d = b.rng.uniform(rx0 + 0.7, rx0 + w / 2 - 0.25)        # down lane (west side of the riser)
        x_u = b.rng.uniform(rx0 + w / 2 + 0.25, rx0 + w - 0.7)    # up lane (east side)
        y_top = b.rng.uniform(goal_y + 2.0, y_narrow_top + 0.5)
        y_w = b.rng.uniform(cy1 - 1.6, cy1 - 0.8)                 # westbound lane (north half of the corridor)
        y_e = b.rng.uniform(cy0 + 0.8, cy0 + 1.6)                 # eastbound lane (south half)
        x_west = b.rng.uniform(24.0, 27.0)
        route = [(x_d, y_top), (x_d, y_w), (x_west, y_w), (x_west, y_e), (x_u, y_e), (x_u, y_top)]
        b.add_on_loop("corner", route, speed)


def intersection(campus, b, p):
    """Intersection with crossing streams: find a gap in perpendicular traffic.

    Geometry: the riser crosses the east-west "crossing" corridor at x 30..35, y 18..23 (no panels).
    Robot: riser_mid (32.5, 12) -> below_door (32.5, 26): straight through the intersection, perpendicular to the
    streams.
    Crowd: p["n_east"] people start walking +x, p["n_west"] -x. Everyone loops along the crossing corridor between
    turnarounds at x 17..24 and 41..48 (randomised), keeping right (eastbound in the south half, westbound in the
    north half), so each person crosses the riser twice per loop. A fraction p["group_frac"] of people walk in
    pairs or triples (shared group_id, side by side on parallel lanes 0.7 m apart).
    Timing: initial positions are uniform along each person's starting leg (~20 m on either side of the
    intersection); with short loops (~45 m) a person crosses the riser every ~1-3 s for the whole episode.
    """
    zx0, zy0, zx1, zy1 = campus.zone("crossing")
    b.set_robot(campus.anchor("riser_mid"), campus.anchor("below_door"))
    low = (zy0 + 0.7, (zy0 + zy1) / 2 - 0.5)          # south-half lane band
    high = ((zy0 + zy1) / 2 + 0.5, zy1 - 0.7)         # north-half lane band
    spacing = 0.7

    for cls, n, d in (("eastbound", p["n_east"], +1), ("westbound", p["n_west"], -1)):
        left = n
        while left > 0:
            size = 1
            if left >= 2 and b.rng.random() < p.get("group_frac", 0.0):
                size = min(left, b.rng.choice([2, 2, 3]))
            left -= size
            speed = b.rng.uniform(*p["speed"])
            spread = spacing * (size - 1)
            lo, hi = b.rng.uniform(17.0, 24.0), b.rng.uniform(41.0, 48.0)
            go_band, back_band = (low, high) if d > 0 else (high, low)
            s = 1 if d > 0 else -1                     # members spread toward the corridor's middle
            if s > 0:
                y_go = b.rng.uniform(go_band[0], max(go_band[0], go_band[1] - spread))
                y_back = b.rng.uniform(min(back_band[1], back_band[0] + spread), back_band[1])
            else:
                y_go = b.rng.uniform(min(go_band[1], go_band[0] + spread), go_band[1])
                y_back = b.rng.uniform(back_band[0], max(back_band[0], back_band[1] - spread))

            def route_for(m):
                dy = s * spacing * m
                if d > 0:
                    return [(hi, y_go + dy), (hi, y_back - dy), (lo, y_back - dy), (lo, y_go + dy)]
                return [(lo, y_go + dy), (lo, y_back - dy), (hi, y_back - dy), (hi, y_go + dy)]

            routes = [route_for(m) for m in range(size)]
            group = b.new_group() if size > 1 else -1
            # the go leg is segment 3 for both directions
            phase = b.phase_on_loop(routes[0], segments=[3],
                                    check=lambda i, t: all(b.is_clear(b.point_on_loop(r, i, t)) for r in routes))
            for r in routes:
                b.add_on_loop(cls if size == 1 else f"{cls}_group", r, speed, group=group, phase=phase)


def plaza_dense(campus, b, p):
    """Dense plaza: crowd density beyond what DS-RNN was trained on (5 people), people milling in every direction.

    Geometry: the plaza (x 20..48, y 30..52); no panels (the shell's 5 m opening to the riser stays open).
    Robot: above_door (32.5, 34) -> plaza_far (44, 48), a ~18 m diagonal.
    Crowd: p["n"] people milling around the robot's path: each walks a loop of 3-5 random points in a band of
    half-width p["band"] around the start-goal line (inside the plaza, >= 1.5 m off its walls, >= 2 m from the
    robot's start and goal), so the density is p["n"] / (~18 m x 2 * band). A fraction p["frac_standing"] stand
    still (chatting), p["frac_impassive"] get HuNav behaviour 2 (IMPASSIVE) instead of 1 (REGULAR).
    Timing: everyone is on their loop from t = 0 at a random phase; the band stays equally crowded throughout.
    """
    a, g = campus.anchor("above_door"), campus.anchor("plaza_far")
    b.set_robot(a, g)
    px0, py0, px1, py1 = campus.zone("plaza")
    ux, uy = g[0] - a[0], g[1] - a[1]
    L = math.hypot(ux, uy)
    ux, uy = ux / L, uy / L
    half = p.get("band", 4.0)

    def point():
        for _ in range(1000):
            s, v = b.rng.uniform(-1.0, L + 1.0), b.rng.uniform(-half, half)
            q = (a[0] + s * ux - v * uy, a[1] + s * uy + v * ux)
            if px0 + 1.5 <= q[0] <= px1 - 1.5 and py0 + 1.5 <= q[1] <= py1 - 1.5 \
                    and math.dist(q, a) >= 2.0 and math.dist(q, g) >= 2.0:
                return q
        raise RuntimeError("no milling point found")

    n_stand = round(p["n"] * p.get("frac_standing", 0.0))
    for k in range(p["n"]):
        hunav = {"behavior": {"type": 2}} if b.rng.random() < p.get("frac_impassive", 0.0) else None
        if k < n_stand:
            for _ in range(200):
                q = point()
                if b.is_clear(q, min_sep=1.0, robot_sep=2.5):
                    break
            b.add_pedestrian("standing", q, [(q[0] + 0.3, q[1]), q], 0.5, hunav=hunav)
            continue
        route = [point() for _ in range(b.rng.randint(3, 5))]
        b.add_on_loop("milling", route, b.rng.uniform(*p["speed"]), hunav=hunav)


def gauntlet(campus, b, p):
    """The long run through every zone: robot corridor_west (2, 2.5) -> plaza_far (44, 48), ~80 m.

    Builds every other template with its own parameter block (p["corridor_counterflow"], p["blind_corner"], ...;
    smaller crowds than the standalone ranges so the total stays ~30-55 people), then overrides the robot task.
    Each zone's population lives on closed loops inside its zone with random phases, so every zone is busy
    whenever the robot gets there (DS-RNN reaches the plaza after ~1.5 min, DWB after ~5 min).
    """
    for name in ("corridor_counterflow", "blind_corner", "intersection", "door_exit", "plaza_dense"):
        TEMPLATES[name](campus, b, p[name])
    b.set_robot(campus.anchor("corridor_west"), campus.anchor("plaza_far"))


TEMPLATES = {
    "corridor_counterflow": corridor_counterflow,
    "door_exit": door_exit,
    "blind_corner": blind_corner,
    "intersection": intersection,
    "plaza_dense": plaza_dense,
    "gauntlet": gauntlet,
}


# ================================================================================================= plumbing

def sample_params(raw, rng):
    """Sample every parameter of one template (nested dicts are sampled recursively; ranges that are meant
    to stay ranges, like a per-pedestrian speed range, are written as {"range": [lo, hi]})."""
    out = {}
    for k, v in (raw or {}).items():
        if isinstance(v, dict) and "range" in v:
            out[k] = v["range"]
        elif isinstance(v, dict) and not ({"int", "choice"} & set(v)):
            out[k] = sample_params(v, rng)
        else:
            out[k] = sample(v, rng)
    return out


def generate(spec, campus, template, seed):
    """One scenario: (scenario dict, sampled params). The RNG is seeded from (seed, template) so adding a template
    to the spec doesn't change the scenarios of the others."""
    rng = random.Random(f"{seed}-{template}")
    params = sample_params(spec["templates"][template], rng)
    b = Builder(spec, rng)
    TEMPLATES[template](campus, b, params)
    return b.scenario(), params


PED_CLEARANCE = 0.45      # waypoints/starts must be this far inside the free space (body radius 0.35 + 0.1)
SIGHT_CLEARANCE = 0.3     # each leg of a route must stay this far from walls/panels (line of sight with a body)
ROBOT_CLEARANCE = 0.5     # robot start/goal this far from walls/panels


def panel_footprint(s):
    """shapely box of a static panel entry (model panel_x<L> / panel_y<L>, origin at the footprint centre)."""
    from shapely.geometry import box
    model = s["model"]
    L = {suf: length for length, suf in PANEL_LENGTHS.items()}[model[len("panel_x"):]]
    w, h = (L, PANEL_THICKNESS) if model.startswith("panel_x") else (PANEL_THICKNESS, L)
    x, y = s["pos"][:2]
    return box(x - w / 2, y - h / 2, x + w / 2, y + h / 2)


def free_space(scenario, campus):
    """The walkable floor of one scenario: union of the zones minus every panel footprint."""
    import shapely
    from shapely.geometry import box
    free = shapely.union_all([box(*r) for r in campus.zones.values()])
    panels = [panel_footprint(s) for s in scenario["obstacles"]["static"]]
    return free.difference(shapely.union_all(panels)) if panels else free


def check_scenario(scenario, campus):
    """Static sanity checks; returns a list of problems (empty = OK):
    every pedestrian start/waypoint inside the free space (with body clearance), every leg of every closed route
    in line of sight (HuNav has no path planning), robot start/goal clear, panels unrotated and uniquely named."""
    from shapely.geometry import LineString, Point
    free = free_space(scenario, campus)
    pt_ok, leg_ok, robot_ok = free.buffer(-PED_CLEARANCE), free.buffer(-SIGHT_CLEARANCE), free.buffer(-ROBOT_CLEARANCE)
    problems = []
    names = [s["name"] for s in scenario["obstacles"]["static"]]
    if len(names) != len(set(names)):
        problems.append("duplicate panel names")
    problems += [f"{s['name']} is rotated" for s in scenario["obstacles"]["static"] if abs(s["pos"][2]) > 1e-9]
    for k in ("start", "goal"):
        if not robot_ok.contains(Point(scenario["robots"][0][k][:2])):
            problems.append(f"robot {k} {scenario['robots'][0][k][:2]} too close to a wall/panel")
    for a in scenario["obstacles"]["dynamic"]:
        if not a.get("cyclic_goals", True):
            problems.append(f"{a['name']}: cyclic_goals false crashes the HuNav Gazebo plugin at the last goal")
        bad = [k for k, v in a.get("behavior", {}).items() if k.endswith("_factor") and not isinstance(v, float)]
        if bad or not isinstance(a.get("desired_velocity"), float):
            problems.append(f"{a['name']}: non-float {bad or 'desired_velocity'} (Arena drops such agents)")
        pts = [tuple(a["pos"][:2])] + [tuple(w[:2]) for w in a["waypoints"]]
        for q in pts:
            if not pt_ok.contains(Point(q)):
                problems.append(f"{a['name']}: point {q} inside/too close to a wall or panel")
        legs = list(zip(pts, pts[1:]))                                 # start -> wp0 -> ... -> wp_last
        if a.get("cyclic_goals", True):
            legs.append((pts[-1], pts[1]))                             # ... -> wp0 again
        for q0, q1 in legs:
            if q0 != q1 and not leg_ok.contains(LineString([q0, q1])):
                problems.append(f"{a['name']}: no line of sight {q0} -> {q1}")
    return problems


def write_map(world_dir, name, scenario):
    """Per-scenario Nav2 map with the panels rasterised in (scripts/scenario_maps.py, owned by the infra
    work). Sets scenario["map"]. Returns the map path, or None if scenario_maps.py isn't available."""
    try:
        from scenario_maps import write_scenario_map
    except ImportError:
        return None
    return write_scenario_map(world_dir, name, scenario)


def save_preview(scenario, world_dir, out_png, title):
    m = yaml.safe_load((world_dir / "map" / "map.yaml").read_text())
    img = Image.open(world_dir / "map" / m["image"])
    ox, oy, _ = m["origin"]
    extent = [ox, ox + img.width * m["resolution"], oy, oy + img.height * m["resolution"]]
    fig, ax = plt.subplots(figsize=(12, 6))
    ax.imshow(img, cmap="gray", extent=extent, origin="upper")
    sizes = {f"panel_{ax_}{suf}": (L, PANEL_THICKNESS) if ax_ == "x" else (PANEL_THICKNESS, L)
             for L, suf in PANEL_LENGTHS.items() for ax_ in "xy"}
    for s in scenario["obstacles"]["static"]:
        x, y = s["pos"][:2]
        w, h = sizes.get(s["model"], (1.0, 1.0))
        ax.add_patch(plt.Rectangle((x - w / 2, y - h / 2), w, h, color="saddlebrown", alpha=0.8))
    classes = sorted({a["class"] for a in scenario["obstacles"]["dynamic"]})
    colors = dict(zip(classes, plt.cm.tab10.colors))
    for a in scenario["obstacles"]["dynamic"]:
        pts = [a["pos"]] + a["waypoints"] + ([a["waypoints"][0]] if a.get("cyclic_goals", True) else [])
        ax.plot([q[0] for q in pts], [q[1] for q in pts], color=colors[a["class"]], alpha=0.35, lw=1)
        ax.plot(*a["pos"][:2], "o", color=colors[a["class"]], ms=5)
    for c in classes:
        ax.plot([], [], "o-", color=colors[c], label=c)
    r = scenario["robots"][0]
    ax.annotate("", xy=r["goal"][:2], xytext=r["start"][:2], arrowprops=dict(arrowstyle="->", color="blue", lw=2))
    ax.plot(*r["start"][:2], "bs", ms=8, label="robot start -> goal")
    # zoom to what this scenario uses (robot task, routes, panels) plus a margin
    xs = [r["start"][0], r["goal"][0]] + [q[0] for a in scenario["obstacles"]["dynamic"] for q in a["waypoints"]] \
        + [s["pos"][0] for s in scenario["obstacles"]["static"]]
    ys = [r["start"][1], r["goal"][1]] + [q[1] for a in scenario["obstacles"]["dynamic"] for q in a["waypoints"]] \
        + [s["pos"][1] for s in scenario["obstacles"]["static"]]
    ax.set_xlim(min(xs) - 4, max(xs) + 4)
    ax.set_ylim(min(ys) - 4, max(ys) + 4)
    ax.set_aspect("equal")
    ax.set_title(title, fontsize=9)
    ax.legend(loc="upper left", fontsize=7)
    fig.tight_layout()
    fig.savefig(out_png, dpi=90)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description="Generate campus stress-test scenarios from a spec")
    ap.add_argument("spec", help="spec YAML, e.g. scenarios/campus/stress.yaml")
    ap.add_argument("--world", default="campus", help="world folder under <repo>/worlds")
    ap.add_argument("--seeds", type=int, nargs=2, metavar=("FIRST", "LAST"),
                    help="generate every seed in FIRST..LAST (default: the spec's seed)")
    ap.add_argument("--only", nargs="+", help="only these templates (default: all templates in the spec)")
    ap.add_argument("--no-preview", action="store_true", help="skip preview PNGs (faster for many seeds)")
    ap.add_argument("--link", action="store_true", help="also (re)link the world into Arena")
    ap.add_argument("--no-map", action="store_true",
                    help="don't write per-scenario Nav2 maps (default: write them, panels rasterised in)")
    args = ap.parse_args()

    spec = yaml.safe_load(Path(args.spec).read_text())
    world_dir = REPO_ROOT / "worlds" / args.world
    campus = Campus(yaml.safe_load((world_dir / "zones.yaml").read_text()))
    seeds = range(args.seeds[0], args.seeds[1] + 1) if args.seeds else [spec.get("seed", 0)]
    templates = args.only or list(spec["templates"])
    (world_dir / "previews").mkdir(exist_ok=True)

    manifest = world_dir / "scenarios" / "manifest.csv"
    rows = {}
    if manifest.exists():
        with open(manifest) as f:
            rows = {r["scenario"]: r for r in csv.DictReader(f)}

    warned_map = False
    for template in templates:
        for seed in seeds:
            name = f"{template}_s{seed}"
            scenario, params = generate(spec, campus, template, seed)
            problems = check_scenario(scenario, campus)
            if problems:
                raise ValueError(f"{name}: " + "; ".join(problems))
            if not args.no_map and write_map(world_dir, name, scenario) is None and not warned_map:
                print("NOTE: scripts/scenario_maps.py not found - no per-scenario maps (Nav2 uses map.png)")
                warned_map = True
            (world_dir / "scenarios" / f"{name}.json").write_text(json.dumps(scenario, indent=2))
            if not args.no_preview:
                short = ", ".join(f"{k}={v:.2f}" if isinstance(v, float) else f"{k}={v}" for k, v in params.items()
                                  if not isinstance(v, dict))
                save_preview(scenario, world_dir, world_dir / "previews" / f"{name}.png", f"{name}: {short}")
            rows[f"{name}.json"] = {"scenario": f"{name}.json", "template": template, "seed": seed,
                                    "spec": spec.get("name"), "params": json.dumps(params)}
            print(f"{name}: {len(scenario['obstacles']['dynamic'])} pedestrians, "
                  f"{len(scenario['obstacles']['static'])} panels")

    with open(manifest, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["scenario", "template", "seed", "spec", "params"])
        w.writeheader()
        w.writerows(rows.values())
    print(f"manifest: {manifest} ({len(rows)} scenarios)")
    if args.link:
        link_into_arena(world_dir)


if __name__ == "__main__":
    main()
