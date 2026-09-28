"""
Generate a pedestrian scenario (Arena scenario JSON) for a hallway world from a YAML spec.

  spec  : scenarios/<something>.yaml            (agent classes, counts, speeds, behaviours, seed)
  world : worlds/<world>/world_params.yaml      (written by generate_hallway.py; the layout)
  out   : worlds/<world>/scenarios/<spec name>.json   <- what Arena loads
          worlds/<world>/preview_<spec name>.png    <- start positions + routes, for checking by eye

Usage (from the repo root):
  uv run scripts/generate_scenario.py scenarios/hallway_tame.yaml
  uv run scripts/generate_scenario.py scenarios/hallway_tame.yaml --world school_hallway --seed 3 --link

Then launch with:  bash scripts/arena-launch.sh school_hallway scenario scenario:=hallway_tame.json

How the scenario JSON is consumed (read from Arena's code, see docs/09-hallway-scenarios.md):
  * arena_simulation_setup reads name/model/pos/waypoints and keeps the WHOLE entry as `extra`
  * Arena's HuNav adapter reads desired_velocity, radius, goal_radius, cyclic_goals, group_id, type,
    behavior{...} from that `extra`; anything missing comes from hunav_agents/default.yaml (agent1)
  * pedestrians follow `waypoints` in order; with cyclic_goals they loop back to the first one, walking a
    straight line (with obstacle avoidance), so every route here is a closed loop that stays in free space
"""

import argparse
import copy
import json
import math
import random
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import yaml
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parent))
from generate_hallway import REPO_ROOT, link_into_arena  # noqa: E402

END_MARGIN = 5.0   # routes turn around at least this far from the hallway's end walls (m); keeps the
                   # robot's start (x=1) and goal (x=L-1) areas clear
END_SPREAD = 3.0   # ...plus a random 0..END_SPREAD per agent, so turnarounds don't all land on one spot.
                   # (With every lane turning at exactly x=L-2, 10 pedestrians jammed against the end wall.)
SIDE_MARGIN = 1.0  # keep lanes this far from the long walls (m)
DOOR_FRACTION = 0.5  # "door" waypoint sits this fraction of the way into a side corridor. Deeper than ~half,
                     # the dead end's three walls push people back before they reach the waypoint, and they
                     # stop there forever (seen at 1 m from the end of a 3 m-wide, 4 m-deep corridor).
JITTER = 0.4       # random offset added to every waypoint (m). Arena hard-codes HuNav's goal force to 20
                   # (vs social force ~5), so two people aiming at the SAME point overlap instead of sharing it.
GROUP_SPACING = 0.8  # friends walk side by side on parallel lanes this far apart (m)


class Hallway:
    """Layout helpers built from world_params.yaml."""

    def __init__(self, params):
        self.L = params["hall_length"]
        self.W = params["hall_width"]
        self.w = params["corridor_width"]
        self.s = params["corridor_length"]
        self.corridors = params["corridor_pos"]
        self.robot_start = (1.0, self.W / 2)
        self.robot_goal = (self.L - 1.0, self.W / 2)

    def lane_y(self, rng, side):
        """A y inside one half of the hallway. People keep right: +x walkers use 'low', -x walkers 'high'."""
        if side == "low":
            return rng.uniform(SIDE_MARGIN, self.W / 2 - 0.5)
        return rng.uniform(self.W / 2 + 0.5, self.W - SIDE_MARGIN)

    def doors(self):
        """All door points: (corridor x, top/bottom)."""
        out = []
        for c in self.corridors:
            out.append((c, self.W + self.s * DOOR_FRACTION))  # top stub
            out.append((c, -self.s * DOOR_FRACTION))          # bottom stub
        return out

    def turnarounds(self, rng):
        """(lo, hi) x positions where one agent turns around, randomised per agent."""
        return (END_MARGIN + rng.uniform(0, END_SPREAD), self.L - END_MARGIN - rng.uniform(0, END_SPREAD))


# --- route patterns: each returns (start_xy, [waypoint_xy, ...]) forming a closed loop -----------------

def route_lane(hall, rng, direction, y_go=None, y_back=None):
    """Walk to the far end on your side, cross over, walk back on the other side, cross over, repeat.
    y_go / y_back can be forced (used to put group members on parallel lanes)."""
    lo, hi = hall.turnarounds(rng)
    if direction > 0:
        y_go = hall.lane_y(rng, "low") if y_go is None else y_go
        y_back = hall.lane_y(rng, "high") if y_back is None else y_back
        route = [(hi, y_go), (hi, y_back), (lo, y_back), (lo, y_go)]
    else:
        y_go = hall.lane_y(rng, "high") if y_go is None else y_go
        y_back = hall.lane_y(rng, "low") if y_back is None else y_back
        route = [(lo, y_go), (lo, y_back), (hi, y_back), (hi, y_go)]
    start = (rng.uniform(lo + 2, hi - 2), y_go)
    return start, route


def jitter(route, rng, hall, keep_x=()):
    """Nudge every waypoint by up to JITTER so no two agents aim at exactly the same point.
    Indices in keep_x only move in y (door points must stay centred in their corridor)."""
    out = []
    for i, (x, y) in enumerate(route):
        dx = 0.0 if i in keep_x else rng.uniform(-JITTER, JITTER)
        dy = rng.uniform(-JITTER, JITTER)
        out.append((x + dx, min(max(y + dy, -hall.s + 0.5), hall.W + hall.s - 0.5)))
    return out


def route_zigzag(hall, rng, direction, step):
    """Weave from side to side every `step` metres, end to end and back (a distracted walker)."""
    lo, hi = hall.turnarounds(rng)
    xs = [lo + i * step for i in range(int((hi - lo) // step) + 1)] + [hi]
    ys = [hall.lane_y(rng, "low" if i % 2 == 0 else "high") for i in range(len(xs))]
    forward = list(zip(xs, ys))
    backward = [(x, hall.W - y) for x, y in reversed(forward)]  # mirrored, so the return leg weaves too
    loop = forward + backward
    if direction < 0:
        loop = backward + forward
    # start somewhere along the first leg, heading for the next zig point
    k = rng.randrange(1, len(xs) - 1)
    (x0, y0), (x1, y1) = loop[k - 1], loop[k]
    t = rng.uniform(0.2, 0.8)
    start = (x0 + t * (x1 - x0), y0 + t * (y1 - y0))
    return start, loop[k:] + loop[:k]


def route_door_loop(hall, rng, door_a, door_b):
    """Come out of door A, join the traffic on your side, leave through door B, then come back the other way."""
    (xa, ya), (xb, yb) = door_a, door_b
    side_go, side_back = ("low", "high") if xb > xa else ("high", "low")
    y_go, y_back = hall.lane_y(rng, side_go), hall.lane_y(rng, side_back)
    route = [(xa, y_go), (xb, y_go), (xb, yb), (xb, y_back), (xa, y_back), (xa, ya)]
    return (xa, ya), route


# --- generation ----------------------------------------------------------------------------------------

def deep_merge(base, override):
    out = copy.deepcopy(base)
    for k, v in (override or {}).items():
        out[k] = deep_merge(out.get(k, {}), v) if isinstance(v, dict) else v
    return out


def directions(spec_dir, n):
    """'+x' / '-x' / 'both' -> list of +1/-1, alternating for 'both'."""
    if spec_dir == "+x":
        return [1] * n
    if spec_dir == "-x":
        return [-1] * n
    return [1 if i % 2 == 0 else -1 for i in range(n)]


def build(spec, hall, seed):
    rng = random.Random(seed)
    starts = []   # (x, y, group_id) of everyone placed so far, for spacing checks
    agents = []
    next_group = 1  # HuNav: -1 = no group, >= 0 = group id

    def free_spot(propose, group=-1, tries=500):
        """Re-sample a start until it is far enough from the robot and from everyone else."""
        for _ in range(tries):
            start, route = propose()
            if math.dist(start, hall.robot_start) < spec["robot_clearance"]:
                continue
            gap = [math.dist(start, (x, y)) for x, y, g in starts if not (group >= 0 and g == group)]
            if all(d >= spec["min_spacing"] for d in gap):
                return start, route
        raise RuntimeError("couldn't place an agent - lower counts or min_spacing")

    def add(cls, start, route, speed, group=-1):
        hunav = deep_merge(spec["defaults"], cls.get("hunav"))
        agents.append({
            "name": f"{cls['name']}_{sum(a['name'].startswith(cls['name'] + '_') for a in agents)}",
            "model": spec["model"],
            "type": 1,  # hunav_msgs/Agent PERSON
            "pos": [round(start[0], 2), round(start[1], 2), 0.0],
            "waypoints": [[round(x, 2), round(y, 2), 0.0] for x, y in route],
            "desired_velocity": round(speed, 2),
            "group_id": group,
            "class": cls["name"],  # ignored by Arena, handy for analysis
            **hunav,
        })
        starts.append((start[0], start[1], group))

    doors = hall.doors()
    rng.shuffle(doors)

    for cls in spec["classes"]:
        n, pattern = cls["count"], cls["pattern"]
        dirs = directions(cls.get("direction", "both"), n)
        for i in range(n):
            speed = rng.uniform(*cls["speed"])
            if pattern == "lane" and "group_size" in cls:
                # friends walk side by side: every member gets its own parallel lane GROUP_SPACING apart
                # (sharing one route made them converge onto the same waypoints and overlap)
                group, size = next_group, cls["group_size"][i]
                next_group += 1
                leader, route = free_spot(lambda: route_lane(hall, rng, dirs[i]))
                y_go, y_back = route[0][1], route[1][1]
                side = 1 if y_go < hall.W / 2 else -1  # spread toward the hallway middle
                for m in range(size):
                    dy = side * GROUP_SPACING * m
                    member_route = [(x, y_go + dy if y == y_go else y_back - dy) for x, y in route]
                    add(cls, (leader[0], leader[1] + dy), member_route, speed, group)
            elif pattern == "lane":
                start, route = free_spot(lambda: route_lane(hall, rng, dirs[i]))
                add(cls, start, jitter(route, rng, hall), speed)
            elif pattern == "zigzag":
                start, route = free_spot(lambda: route_zigzag(hall, rng, dirs[i], cls.get("zigzag_step", 8.0)))
                add(cls, start, jitter(route, rng, hall), speed)
            elif pattern == "door_loop":
                # every corridor walker gets its own entry AND exit door, so none share a door waypoint
                door_a = doors.pop()
                door_b = next((d for d in doors if d[0] != door_a[0]), None) or \
                    rng.choice([d for d in hall.doors() if d[0] != door_a[0]])
                if door_b in doors:
                    doors.remove(door_b)
                start, route = free_spot(lambda: route_door_loop(hall, rng, door_a, door_b))
                add(cls, start, jitter(route, rng, hall, keep_x=(2, 5)), speed)  # 2, 5 = door waypoints
            else:
                raise ValueError(f"unknown pattern {pattern!r}")

    return {
        "robots": [{"start": [*hall.robot_start, 0.0], "goal": [*hall.robot_goal, 0.0]}],
        "obstacles": {"static": [], "dynamic": agents, "interactive": []},
    }


def save_preview(scenario, world_dir, out_png):
    """Map + robot route + every pedestrian's start (dot) and loop (line), coloured by class."""
    m = yaml.safe_load((world_dir / "map" / "map.yaml").read_text())
    img = Image.open(world_dir / "map" / m["image"])
    ox, oy, _ = m["origin"]
    extent = [ox, ox + img.width * m["resolution"], oy, oy + img.height * m["resolution"]]

    fig, ax = plt.subplots(figsize=(16, 16 * img.height / img.width + 2))
    ax.imshow(img, cmap="gray", extent=extent, origin="upper")
    classes = sorted({a["class"] for a in scenario["obstacles"]["dynamic"]})
    colors = dict(zip(classes, plt.cm.tab10.colors))
    for a in scenario["obstacles"]["dynamic"]:
        pts = [a["pos"]] + a["waypoints"] + [a["waypoints"][0]]
        ax.plot([p[0] for p in pts], [p[1] for p in pts], color=colors[a["class"]], alpha=0.35, lw=1)
        ax.plot(*a["pos"][:2], "o", color=colors[a["class"]], ms=7)
    for c in classes:
        ax.plot([], [], "o-", color=colors[c], label=c)
    r = scenario["robots"][0]
    ax.annotate("", xy=r["goal"][:2], xytext=r["start"][:2], arrowprops=dict(arrowstyle="->", color="blue", lw=2))
    ax.plot(*r["start"][:2], "bs", ms=10, label="robot start → goal")
    ax.set_aspect("equal")
    ax.set_title(f"{out_png.stem}: {len(scenario['obstacles']['dynamic'])} pedestrians (dots = starts, lines = loops)")
    ax.legend(loc="upper left", bbox_to_anchor=(1.01, 1))
    fig.tight_layout()
    fig.savefig(out_png, dpi=110)
    plt.close(fig)


def main():
    p = argparse.ArgumentParser(description="Generate a hallway pedestrian scenario from a YAML spec")
    p.add_argument("spec", help="scenario spec YAML, e.g. scenarios/hallway_tame.yaml")
    p.add_argument("--world", default="school_hallway", help="world folder under <repo>/worlds")
    p.add_argument("--seed", type=int, default=None, help="override the spec's seed")
    p.add_argument("--name", default=None, help="override the output name (default: spec name)")
    p.add_argument("--link", action="store_true", help="also (re)link the world into Arena")
    args = p.parse_args()

    spec = yaml.safe_load(Path(args.spec).read_text())
    seed = spec.get("seed", 0) if args.seed is None else args.seed
    name = args.name or spec["name"]
    world_dir = REPO_ROOT / "worlds" / args.world
    hall = Hallway(yaml.safe_load((world_dir / "world_params.yaml").read_text()))

    scenario = build(spec, hall, seed)
    out_json = world_dir / "scenarios" / f"{name}.json"
    out_json.write_text(json.dumps(scenario, indent=2))
    save_preview(scenario, world_dir, world_dir / f"preview_{name}.png")

    counts = {}
    for a in scenario["obstacles"]["dynamic"]:
        counts[a["class"]] = counts.get(a["class"], 0) + 1
    print(f"wrote {out_json}  (seed {seed}): {sum(counts.values())} pedestrians {counts}")

    if args.link:
        link_into_arena(world_dir)


if __name__ == "__main__":
    main()
