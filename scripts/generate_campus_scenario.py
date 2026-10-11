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

PANEL_MODEL = "cuboid"   # Arena's stock static model: a 1 x 1 x 2.2 m box (entities/obstacles/static/cuboid).
PANEL_SIZE = 1.0         # its footprint edge (m). See block_rect().


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

    def block_rect(self, x0, y0, x1, y1, tag="panel"):
        """Fill the axis-aligned rectangle with static boxes (geometry adjustments: narrow a corridor, make a
        door). The stock `cuboid` is 1 x 1 m, so the rectangle is covered by a grid of boxes that may overlap;
        a side shorter than 1 m gets one box centred on it, i.e. the box sticks out ~equally on both sides. To
        make a thin ledge along a wall, extend the rectangle INTO the wall (boxes may overlap walls).
        TODO(after verification, docs/14 §5): may switch to dedicated thin panel models."""
        def centres(a, b):
            n = max(1, math.ceil((b - a) / PANEL_SIZE - 1e-9))
            if n == 1:
                return [(a + b) / 2]
            step = (b - a - PANEL_SIZE) / (n - 1)
            return [a + PANEL_SIZE / 2 + i * step for i in range(n)]
        for cx in centres(x0, x1):
            for cy in centres(y0, y1):
                self.static.append({"name": f"{tag}_{len(self.static)}", "model": PANEL_MODEL,
                                    "pos": [round(cx, 3), round(cy, 3), 0.0]})

    def jitter(self, pt, amount=0.4):
        """Random offset so no two pedestrians aim at exactly the same waypoint (they would overlap: Arena
        hard-codes HuNav's goal force to 20, much stronger than the social force)."""
        return (pt[0] + self.rng.uniform(-amount, amount), pt[1] + self.rng.uniform(-amount, amount))

    def scenario(self):
        if self.robot is None:
            raise ValueError("template didn't call set_robot()")
        return {"robots": [self.robot],
                "obstacles": {"static": self.static, "dynamic": self.agents, "interactive": []}}


# ================================================================================================ templates
# Each template: f(campus, b, p) where b is a Builder and p the SAMPLED parameters (a dict). It fills b.
# Register it in TEMPLATES at the bottom. Keep pedestrian routes inside their zone so templates don't interfere.

def corridor_counterflow(campus, b, p):
    """WORKED EXAMPLE. Narrow corridor with a stream of people coming the other way.

    Geometry: the shell corridor is 5 m wide (y 0..5). Panels along both long walls, between x = narrow_x0 and
    narrow_x1, leave a clear width p["width"] in the middle.
    Robot: west end -> just before the blind corner, i.e. it must get through the whole narrow section.
    Crowd: p["n_oncoming"] people walking -x through the narrow part (looping back on the far side), plus
    p["n_with"] walking +x like the robot.
    """
    x0, y0, x1, y1 = campus.zone("corridor")
    narrow_x0, narrow_x1 = 6.0, 26.0
    mid = (y0 + y1) / 2
    gap = min(p["width"], (y1 - y0) - 0.2)
    side = ((y1 - y0) - gap) / 2                                   # how much each side gets narrowed
    if side > 0.05:
        b.block_rect(narrow_x0, y0 - 0.5, narrow_x1, y0 + side, tag="narrow_s")    # extends into the wall:
        b.block_rect(narrow_x0, y1 - side, narrow_x1, y1 + 0.5, tag="narrow_n")    # see block_rect()

    b.set_robot(campus.anchor("corridor_west"), campus.anchor("corridor_east"))

    lane_lo, lane_hi = mid - gap / 2 + 0.4, mid + gap / 2 - 0.4    # stay 0.4 m off the panels
    for i in range(p["n_oncoming"]):
        speed = b.rng.uniform(*p["speed"])
        y = b.rng.uniform(lane_lo, lane_hi) if lane_hi > lane_lo else mid
        start = (b.rng.uniform(narrow_x0, x1 - 6.0), y)               # spread along the corridor
        route = [b.jitter((narrow_x0 - 2.0, y)), b.jitter((x1 - 6.0, y))]   # -x first, then back
        b.add_pedestrian("oncoming", start, route, speed)
    for i in range(p["n_with"]):
        speed = b.rng.uniform(*p["speed"])
        y = b.rng.uniform(lane_lo, lane_hi) if lane_hi > lane_lo else mid
        start = (b.rng.uniform(narrow_x0 + 2.0, narrow_x1), y)
        route = [b.jitter((x1 - 6.0, y)), b.jitter((narrow_x0 - 2.0, y))]   # +x first
        b.add_pedestrian("with_robot", start, route, speed)


def door_exit(campus, b, p):
    """TODO. Doorway against an exiting crowd (docs/14 §6.2).
    Geometry: the door is campus.door (centre, shell_width 5 m along x at y = 30). Close it down to p["door_width"]
    with two block_rect()s, one on each side of the gap, ~1 m deep (y 29.5..30.5 or so).
    Robot: from campus.anchor("below_door") (or "intersection") -> into the plaza (e.g. "plaza_center").
    Crowd: p["n_exiting"] people starting in the plaza near the door, walking out through the door into the riser,
    then back (cyclic). Most should be in the plaza when the robot arrives: start them spread over the plaza's
    southern part. Optional: a few people waiting/standing near the door (a route with two very close points).
    Watch for: HuNav pedestrians and the door's corners (pedestrians may get stuck on the panels' corners: put a
    waypoint exactly in the door's centre, like DOOR_FRACTION in generate_scenario.py).
    """
    raise NotImplementedError("TODO: door_exit (docs/14 §6.2)")


def blind_corner(campus, b, p):
    """TODO. Blind corner (docs/14 §6.3).
    Geometry: the L-turn where the corridor (east end, x 30..35) meets the riser. No panels needed, the walls
    themselves block line of sight; optionally narrow the riser with p["width"].
    Robot: corridor (x ~ 20) -> riser ("riser_mid"), so it has to turn the corner.
    Crowd: p["n"] people walking DOWN the riser and turning WEST into the corridor (and back), so they appear
    around the corner just as the robot gets there. Timing matters: start them at different heights in the riser
    (spread over y 6..16) so someone is always about to come around. Faster walkers (p["speed"]) = less reaction
    time. This is where lidar-based planners (DWB) and ground-truth-perception planners (DS-RNN) differ most.
    """
    raise NotImplementedError("TODO: blind_corner (docs/14 §6.3)")


def intersection(campus, b, p):
    """TODO. Intersection with crossing streams (docs/14 §6.4).
    Geometry: the riser crosses the east-west "crossing" zone at x 30..35, y 18..23.
    Robot: "riser_mid" -> "below_door", i.e. straight through the intersection, perpendicular to the streams.
    Crowd: two streams along the crossing corridor: p["n_east"] walking +x, p["n_west"] walking -x, looping between
    "crossing_west" and "crossing_east" (keep-right lanes, like route_lane in generate_scenario.py). Optionally
    p["group_frac"] of them in pairs/triples (use b.new_group() and parallel lanes ~0.8 m apart).
    """
    raise NotImplementedError("TODO: intersection (docs/14 §6.4)")


def plaza_dense(campus, b, p):
    """TODO. Dense plaza (docs/14 §6.5).
    Geometry: the plaza zone, no panels (optionally a few static "pillars": block_rect of 1 x 1 m).
    Robot: "above_door" -> "plaza_far".
    Crowd: p["n"] people milling: each gets a loop of 3-5 random points inside the plaza (keep 1.5 m off the
    walls), random speeds from p["speed"]. Mix in behaviours via p["frac_impassive"] etc. (HuNav behavior.type:
    1 regular, 2 impassive, 5 curious, 6 threatening; 3-6 need HuNav robot-awareness, see docs/14 §8).
    This is the density stress test: DS-RNN only attends to the 5 nearest people.
    """
    raise NotImplementedError("TODO: plaza_dense (docs/14 §6.5)")


def gauntlet(campus, b, p):
    """TODO (last). The long run through every zone: robot "corridor_west" -> "plaza_far".
    Easiest: call the other templates with their parameter dicts (p["corridor_counterflow"], p["door_exit"], ...),
    then overwrite the robot task with b.set_robot(...) at the end (each template sets its own robot task)."""
    raise NotImplementedError("TODO: gauntlet (docs/14 §6.6)")


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


def save_preview(scenario, world_dir, out_png, title):
    m = yaml.safe_load((world_dir / "map" / "map.yaml").read_text())
    img = Image.open(world_dir / "map" / m["image"])
    ox, oy, _ = m["origin"]
    extent = [ox, ox + img.width * m["resolution"], oy, oy + img.height * m["resolution"]]
    fig, ax = plt.subplots(figsize=(12, 6))
    ax.imshow(img, cmap="gray", extent=extent, origin="upper")
    for s in scenario["obstacles"]["static"]:
        x, y = s["pos"][:2]
        ax.add_patch(plt.Rectangle((x - PANEL_SIZE / 2, y - PANEL_SIZE / 2), PANEL_SIZE, PANEL_SIZE,
                                   color="saddlebrown", alpha=0.6))
    classes = sorted({a["class"] for a in scenario["obstacles"]["dynamic"]})
    colors = dict(zip(classes, plt.cm.tab10.colors))
    for a in scenario["obstacles"]["dynamic"]:
        pts = [a["pos"]] + a["waypoints"] + [a["waypoints"][0]]
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

    for template in templates:
        for seed in seeds:
            name = f"{template}_s{seed}"
            scenario, params = generate(spec, campus, template, seed)
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
