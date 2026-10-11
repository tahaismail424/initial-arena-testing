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
    "corridor_east": (27.0, 2.5),          # just before the blind corner (the corner itself is x 30-35)
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
    """TODO (step 1 in docs/14): the walkable floor as ONE shapely Polygon.

    Hints:
      * shapely.geometry.box(x0, y0, x1, y1) makes a rectangle; shapely.union_all([...]) merges a list of them
        (see gen_blocks + write_hallway_world in generate_hallway.py).
      * Check the result is ONE connected piece: union_all returns a Polygon if it is, a MultiPolygon if some
        zone doesn't touch the rest. Raise a clear error in that case (which zone is disconnected?).
      * If your layout has a LOOP (zones connected in a ring), the polygon has a HOLE (free.interiors is not
        empty). The proposed layout is a tree (no loops), but if you add one, build_walls and the map writer
        must handle holes too - see their docstrings.
    """
    raise NotImplementedError("TODO step 1: union the zone rectangles into one Polygon (docs/14 §3)")


def build_walls(free):
    """TODO (step 2): wall segments for walls.yaml, as [[[x1, y1], [x2, y2]], ...].

    generate_hallway.gen_walls(free) already does this for the polygon's OUTER boundary (it pushes the outline
    out by half a wall thickness so corridor widths stay exact, and merges collinear points). Start by calling it.

    Then: does it cover holes? (free.interiors) For each interior ring you'd need the same treatment, but pushed
    INTO the hole (buffer the hole polygon by -WALL_THICKNESS/2). Not needed for the proposed tree layout; leave a
    clear error if free.interiors is non-empty, so you notice if you ever add a loop.
    """
    raise NotImplementedError("TODO step 2: wall segments (start with generate_hallway.gen_walls)")


def save_map_png_with_holes(free, origin, size_px, resolution, world_dir):
    """TODO (step 2b): map.png for Nav2. generate_hallway.save_map_png(free, ...) paints the outer boundary white
    on a black canvas. That's all you need for a layout without holes, so to start just call it. (With holes:
    also paint every interior ring black afterwards.)"""
    raise NotImplementedError("TODO step 2b: write map.png (start with generate_hallway.save_map_png)")


def save_default_scenario(world_dir):
    """TODO (step 3): scenarios/default.json - the gauntlet task with no pedestrians, so the world can be
    launched and driven before any crowd exists. Same JSON shape as generate_hallway.save_default_scenario:
        {"robots": [{"start": [x, y, yaw], "goal": [x, y, yaw]}],
         "obstacles": {"static": [], "dynamic": [], "interactive": []}}
    Use ANCHORS["corridor_west"] as the start and ANCHORS["plaza_far"] as the goal. Keep goals >= 3 m from any
    wall (the DS-RNN end-wall trap, docs/10 §8)."""
    raise NotImplementedError("TODO step 3: default.json with the gauntlet start/goal")


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
