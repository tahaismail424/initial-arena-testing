"""
Generate the "campus" stress-test world for Arena: ONE static world shell that contains every stress zone.
Guide: docs/14-campus-world.md (read it first; this file is boilerplate with TODOs for you to fill in).

The shell is deliberately GENEROUS (wide corridors, wide openings). Scenarios narrow things down per run by
spawning wall panels as static obstacles (scripts/generate_campus_scenario.py), so one world serves every
corridor width / door width without rebuilding.

Proposed layout (all units metres; edit ZONES below to change it):

                    y
                   52 ┌──────────────────────────────┐
                      │           PLAZA              │   P: dense crowd, milling
                      │                              │
                   30 └─────────┐  DOOR   ┌──────────┘   D: opening in the plaza's south wall
                                │    ↑    │                 (panels make it 0.9-2 m wide)
     ┌─────────────────────────┐│    │    │┌──────────────┐
  23 │ EAST-WEST CORRIDOR      ││    │    ││              │   X: intersection (crossing streams)
  18 │  ← ← ←      ⊕ X → → →   ││    │    ││              │
     └─────────────────────────┘│    │    │└──────────────┘
                                │ RISER   │                 B: vertical corridor ("riser")
     ┌──────────────────────────┘    │    │
   5 │ NARROW CORRIDOR (counterflow) →┘    │                 C: counterflow corridor;  K: blind corner at
   0 └────────────────────────────────────┘                    its east end (an L-turn, walls hide people)
     0                            30    35         x

The robot's longest task ("gauntlet") runs START (2, 2.5) -> corridor -> blind corner -> riser -> intersection
-> door -> plaza GOAL. Each zone also gets its own short tasks for focused, factorised tests.

Outputs (same files Arena reads as for the hallway; see generate_hallway.py for what each is):
  worlds/campus/map/map.png, map/map.yaml, map/walls.yaml, scenarios/default.json
  worlds/campus/zones.yaml     <- zone geometry + named anchor points for the scenario generator (not linked)
  worlds/campus/preview.png    <- check by eye: map, walls, zone labels, anchors

Usage (from the repo root):
  uv run scripts/generate_campus.py              # writes worlds/campus/
  uv run scripts/generate_campus.py --link       # ...and makes it visible to Arena (then rebuild, see below)
"""

import argparse
import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import shapely  # noqa: E402
import yaml  # noqa: E402
from PIL import Image  # noqa: E402
from shapely.geometry import Polygon, box  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
# Reuse the hallway generator's proven pieces: map frame, map.png/map.yaml writers, walls.yaml writer, linking.
from generate_hallway import (REPO_ROOT, WALL_THICKNESS, gen_walls, link_into_arena,  # noqa: E402,F401
                              map_frame, save_map_png, save_map_yaml, save_walls)

# --------------------------------------------------------------------------------------------------- layout
# Every zone is an axis-aligned rectangle (x0, y0, x1, y1) of FREE floor. Their union is the walkable space.
# Rectangles that touch/overlap are connected; the shared edge becomes an opening (no wall there).
# Keep everything axis-aligned: HuNav's obstacle forces use axis-aligned bounding boxes (docs/09 phantom walls).
ZONES = {
    "corridor":   {"rect": (0.0, 0.0, 35.0, 5.0),   "role": "counterflow corridor; its east end is the blind corner"},
    "riser":      {"rect": (30.0, 0.0, 35.0, 30.0), "role": "vertical corridor joining corridor -> intersection -> door"},
    "crossing":   {"rect": (12.0, 18.0, 53.0, 23.0), "role": "east-west corridor; crosses the riser = intersection"},
    "plaza":      {"rect": (20.0, 30.0, 48.0, 52.0), "role": "open plaza; its south wall has the door"},
}
# The door: where the riser meets the plaza (y = 30, x 30..35). The shell opening is the riser's full 5 m width;
# scenarios close it down to a door with panels. Recorded in zones.yaml so the scenario generator can find it.
DOOR = {"center": (32.5, 30.0), "axis": "x", "shell_width": 5.0}

# Named points the scenario generator uses (robot starts/goals, crowd sources/sinks). Add your own.
ANCHORS = {
    "corridor_west": (2.0, 2.5),
    "corridor_mid": (18.0, 2.5),           # blind_corner start: 12 m before the corner
    "corridor_east": (26.0, 2.5),          # corridor_counterflow goal: past the narrow section, before the corner
    "corner": (32.5, 2.5),
    "riser_mid": (32.5, 12.0),
    "intersection": (32.5, 20.5),
    "crossing_west": (14.0, 20.5),
    "crossing_east": (51.0, 20.5),
    "below_door": (32.5, 26.0),
    "above_door": (32.5, 34.0),
    "plaza_center": (34.0, 41.0),
    "plaza_far": (44.0, 48.0),
}


def write_campus_world(name="campus", location=REPO_ROOT / "worlds", resolution=0.05, margin=1.0):
    world_dir = Path(location) / name
    (world_dir / "map").mkdir(parents=True, exist_ok=True)
    (world_dir / "scenarios").mkdir(parents=True, exist_ok=True)

    free = build_free_space(ZONES)
    walls = build_walls(free)
    origin, size_px = map_frame(free, resolution, margin)

    save_walls(walls, world_dir)
    save_map_png_with_holes(free, origin, size_px, resolution, world_dir)
    save_map_yaml(origin, resolution, world_dir)
    save_default_scenario(world_dir)
    save_zones(world_dir)
    save_preview(free, walls, origin, size_px, resolution, world_dir)

    print(f"wrote {world_dir}  ({size_px[0]}x{size_px[1]} px, {len(walls)} wall segments, "
          f"{free.area:.0f} m^2 of floor)")
    return world_dir


# --------------------------------------------------------------------------------------------- your TODOs

def build_free_space(zones):
    """The walkable floor as ONE shapely Polygon: the union of the zone rectangles.

    Raises if a zone doesn't touch the rest (MultiPolygon) or if the zones enclose a hole (a loop), because
    build_walls / the map writer handle only a single outer ring."""
    rects = {name: box(*z["rect"]) for name, z in zones.items()}
    free = shapely.union_all(list(rects.values()))
    if not isinstance(free, Polygon):
        pieces = list(free.geoms)
        main = max(pieces, key=lambda g: g.area)
        loose = [n for n, r in rects.items() if not r.intersects(main)]
        raise ValueError(f"zones are not one connected area; disconnected: {loose}")
    if free.interiors:
        raise ValueError(f"zones enclose {len(free.interiors)} hole(s) (a loop); build_walls only handles the "
                         "outer boundary - see its docstring")
    return free


def build_walls(free):
    """Wall segments for walls.yaml, as [[[x1, y1], [x2, y2]], ...]: the outer boundary pushed out by half a wall
    thickness (generate_hallway.gen_walls). Only layouts without holes are supported (build_free_space checks)."""
    if free.interiors:
        raise ValueError("free space has holes; walls for interior rings are not implemented")
    walls = gen_walls(free)
    for (x1, y1), (x2, y2) in walls:      # HuNav turns every segment into an axis-aligned box
        if abs(x1 - x2) > 1e-6 and abs(y1 - y2) > 1e-6:
            raise ValueError(f"wall segment {(x1, y1)}->{(x2, y2)} is not axis-aligned")
    return walls


def save_map_png_with_holes(free, origin, size_px, resolution, world_dir):
    """map.png for Nav2: free floor white on a black canvas (generate_hallway.save_map_png). No holes allowed."""
    if free.interiors:
        raise ValueError("free space has holes; map.png would need them painted black")
    save_map_png(free, origin, size_px, resolution, world_dir)


def save_default_scenario(world_dir):
    """scenarios/default.json: the gauntlet task (corridor_west -> plaza_far) with no pedestrians or panels."""
    (sx, sy), (gx, gy) = ANCHORS["corridor_west"], ANCHORS["plaza_far"]
    scenario = {"robots": [{"start": [sx, sy, 0.0], "goal": [gx, gy, 0.0]}],
                "obstacles": {"static": [], "dynamic": [], "interactive": []}}
    with open(world_dir / "scenarios" / "default.json", "w") as f:
        json.dump(scenario, f, indent=2)
    # warmup.json: a 4 m drive, nobody else. Launch with it, then switch to the real scenarios (docs/14 §9): the
    # first reset after a launch makes HuNav pedestrians jump several metres (docs/14 §10), the later ones don't.
    warmup = {"robots": [{"start": [sx, sy, 0.0], "goal": [sx + 4.0, sy, 0.0]}],
              "obstacles": {"static": [], "dynamic": [], "interactive": []}}
    with open(world_dir / "scenarios" / "warmup.json", "w") as f:
        json.dump(warmup, f, indent=2)


# ----------------------------------------------------------------------------------------- done for you

def save_zones(world_dir):
    """zones.yaml: everything the scenario generator needs to know about the layout."""
    data = {
        "zones": {k: {"rect": list(v["rect"]), "role": v["role"]} for k, v in ZONES.items()},
        "door": {"center": list(DOOR["center"]), "axis": DOOR["axis"], "shell_width": DOOR["shell_width"]},
        "anchors": {k: list(v) for k, v in ANCHORS.items()},
        "wall_thickness": WALL_THICKNESS,
    }
    with open(world_dir / "zones.yaml", "w") as f:
        yaml.safe_dump(data, f, default_flow_style=None, sort_keys=False)


def save_preview(free, walls, origin, size_px, resolution, world_dir):
    """Map + walls + zone names + anchors, to check the layout by eye before launching anything."""
    img = np.array(Image.open(world_dir / "map" / "map.png"))
    extent = [origin[0], origin[0] + size_px[0] * resolution, origin[1], origin[1] + size_px[1] * resolution]
    fig, ax = plt.subplots(figsize=(12, 12 * size_px[1] / size_px[0] + 1))
    ax.imshow(img, cmap="gray", extent=extent, origin="upper")
    for (x1, y1), (x2, y2) in walls:
        ax.plot([x1, x2], [y1, y2], color="red", lw=1)
    for name, z in ZONES.items():
        x0, y0, x1, y1 = z["rect"]
        ax.text((x0 + x1) / 2, (y0 + y1) / 2, name, ha="center", va="center", color="tab:blue", alpha=0.5,
                fontsize=11, fontweight="bold")
    for name, (x, y) in ANCHORS.items():
        ax.plot(x, y, "o", color="tab:green", ms=4)
        ax.annotate(name, (x, y), fontsize=7, color="tab:green", xytext=(3, 3), textcoords="offset points")
    dx, dy = DOOR["center"]
    ax.plot([dx - DOOR["shell_width"] / 2, dx + DOOR["shell_width"] / 2], [dy, dy], color="orange", lw=3,
            label="door opening (shell)")
    try:
        sc = json.loads((world_dir / "scenarios" / "default.json").read_text())
        r = sc["robots"][0]
        ax.plot(*r["start"][:2], "bs", ms=8, label="gauntlet start")
        ax.plot(*r["goal"][:2], "b*", ms=14, label="gauntlet goal")
    except (FileNotFoundError, KeyError, IndexError):
        pass
    ax.set_aspect("equal")
    ax.set_title(f"{world_dir.name}: map.png (grey), walls.yaml (red), zones, anchors")
    ax.legend(loc="upper left", fontsize=8)
    fig.tight_layout()
    fig.savefig(world_dir / "preview.png", dpi=110)
    plt.close(fig)


if __name__ == "__main__":
    p = argparse.ArgumentParser(description="Generate the campus stress-test world for Arena")
    p.add_argument("-n", "--name", default="campus", help="world name (folder under worlds/)")
    p.add_argument("-r", "--resolution", type=float, default=0.05, help="map metres per pixel")
    p.add_argument("--link", action="store_true", help="also symlink the world into Arena's simulation-setup")
    args = p.parse_args()
    world_dir = write_campus_world(name=args.name, resolution=args.resolution)
    if args.link:
        link_into_arena(world_dir)
