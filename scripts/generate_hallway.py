"""
Generate a parametric "school hallway" world for Arena (map-first format).

Layout: one long main hallway along +x, with cross corridors that stick out
above and below it. The dead ends of those corridors are the "doors" that
pedestrians will later walk in and out of.

         door          door          door
          ║             ║             ║
  ════════╝═════════════╝═════════════╝════════   y = W
  START →            main hallway            → GOAL
  ════════╗═════════════╗═════════════╗════════   y = 0
          ║             ║             ║
         door          door          door
  x = 0                                          x = L

Everything is derived from ONE shapely polygon (the free floor space), so the
three files Arena reads can never disagree with each other:

  <world>/map/map.png      occupancy image for Nav2  (white = free, black = wall)
  <world>/map/map.yaml     tells Nav2 the image's scale and where it sits in the world
  <world>/map/walls.yaml   line segments -> Gazebo wall boxes + HuNav wall forces
  <world>/scenarios/default.json   robot start/goal only (no pedestrians yet)
  <world>/preview.png      overlay of all of the above, for checking by eye
  <world>/world_params.yaml  the layout parameters (read by generate_scenario.py; not linked into Arena)

Usage (from the repo root):
  uv run scripts/generate_hallway.py -n school_hallway
  uv run scripts/generate_hallway.py -n hall_narrow -W 2 -c 10 20 30 40 50
  uv run scripts/generate_hallway.py -n school_hallway --link   # also make it visible to Arena
"""

import argparse
import json
import math
from glob import glob
from pathlib import Path

import matplotlib

matplotlib.use("Agg")  # render to file; no display needed
import matplotlib.pyplot as plt
import numpy as np
import shapely
import yaml
from PIL import Image, ImageDraw
from shapely.geometry import Polygon, box

REPO_ROOT = Path(__file__).resolve().parent.parent
ARENA_WORLDS = Path.home() / "arena_ws/src/arena/simulation-setup/worlds"

# Arena's Gazebo backend builds every wall as a box 0.2 m thick, centred on the
# segment (see gazebo_simulator.py: spawn_walls). We place segments half a
# thickness OUTSIDE the free space so the clear width is exactly what you asked for.
WALL_THICKNESS = 0.2


def write_hallway_world(
    hall_length=60.0,
    hall_width=12.0,
    corridor_width=3.0,
    corridor_length=4.0,
    corridor_pos=(15.0, 30.0, 45.0),
    name=None,
    location=REPO_ROOT / "worlds",
    resolution=0.05,
    margin=1.0,
):
    world_dir = resolve_path(name, location)  # resolve ONCE so every file lands in the same folder

    blocks = gen_blocks(hall_length, hall_width, corridor_width, corridor_length, corridor_pos)
    free = shapely.union_all(blocks)  # the walkable floor
    if not isinstance(free, Polygon):
        raise ValueError("layout is not one connected area - check corridor positions/widths")

    walls = gen_walls(free)
    origin, size_px = map_frame(free, resolution, margin)

    save_walls(walls, world_dir)
    save_map_png(free, origin, size_px, resolution, world_dir)
    save_map_yaml(origin, resolution, world_dir)
    save_default_scenario(hall_length, hall_width, world_dir)
    save_preview(free, walls, origin, size_px, resolution, world_dir)
    save_world_params(world_dir, hall_length=hall_length, hall_width=hall_width, corridor_width=corridor_width,
                      corridor_length=corridor_length, corridor_pos=list(corridor_pos))

    print(f"wrote {world_dir}  ({size_px[0]}x{size_px[1]} px, {len(walls)} wall segments)")
    return world_dir


def gen_blocks(hall_length, hall_width, corridor_width, corridor_length, corridor_pos):
    """Free-space rectangles: the hallway plus a top and bottom stub per corridor."""
    blocks = [box(0, 0, hall_length, hall_width)]
    for center in corridor_pos:
        x0, x1 = center - corridor_width / 2, center + corridor_width / 2
        if x0 < 0 or x1 > hall_length:
            raise ValueError(f"corridor at x={center} sticks out past the hallway ends")
        blocks.append(box(x0, hall_width, x1, hall_width + corridor_length))  # top stub
        blocks.append(box(x0, -corridor_length, x1, 0))  # bottom stub
    return blocks


def gen_walls(free):
    """Outline of the free space, pushed out by half a wall thickness, as [[x1,y1],[x2,y2]] segments."""
    # mitre joins keep corners square (the default round join would add dozens of tiny segments)
    outline = free.buffer(WALL_THICKNESS / 2, join_style="mitre").exterior
    outline = shapely.simplify(outline, 0)  # drop redundant collinear vertices
    coords = [(round(x, 3), round(y, 3)) for x, y in outline.coords]
    return [[list(a), list(b)] for a, b in zip(coords, coords[1:])]


def map_frame(free, resolution, margin):
    """Image origin (world coords of the bottom-left pixel) and size in pixels."""
    min_x, min_y, max_x, max_y = free.bounds
    origin = (min_x - margin, min_y - margin)
    width_px = math.ceil((max_x - min_x + 2 * margin) / resolution)
    height_px = math.ceil((max_y - min_y + 2 * margin) / resolution)
    return origin, (width_px, height_px)


def world_to_px(x, y, origin, height_px, resolution):
    """World metres -> image pixel. Images count rows from the TOP, the map counts y from the BOTTOM."""
    col = (x - origin[0]) / resolution
    row = height_px - (y - origin[1]) / resolution
    return col, row


def save_walls(walls, world_dir):
    with open(world_dir / "map" / "walls.yaml", "w") as f:
        # default_flow_style=None keeps each point on one line: - [[x1, y1], [x2, y2]]
        yaml.safe_dump({"walls": walls}, f, default_flow_style=None)


def save_map_png(free, origin, size_px, resolution, world_dir):
    """Black canvas (occupied), free floor painted white. The walls are the black band left around it."""
    img = Image.new("L", size_px, 0)
    pts = [world_to_px(x, y, origin, size_px[1], resolution) for x, y in free.exterior.coords]
    ImageDraw.Draw(img).polygon(pts, fill=255)
    img.save(world_dir / "map" / "map.png")


def save_map_yaml(origin, resolution, world_dir):
    meta = {
        "image": "map.png",
        "resolution": resolution,
        "origin": [round(origin[0], 3), round(origin[1], 3), 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.196,
    }
    with open(world_dir / "map" / "map.yaml", "w") as f:
        yaml.safe_dump(meta, f, default_flow_style=None, sort_keys=False)


def save_default_scenario(hall_length, hall_width, world_dir):
    """Robot drives the length of the hallway down its centreline; no pedestrians yet."""
    scenario = {
        "robots": [{"start": [1.0, hall_width / 2, 0.0], "goal": [hall_length - 1.0, hall_width / 2, 0.0]}],
        "obstacles": {"static": [], "dynamic": [], "interactive": []},
    }
    with open(world_dir / "scenarios" / "default.json", "w") as f:
        json.dump(scenario, f, indent=2)


def save_world_params(world_dir, **params):
    """Record the layout parameters so scripts/generate_scenario.py can place pedestrians to match.
    Lives at the world root, so it is NOT linked into Arena (Arena doesn't need it)."""
    with open(world_dir / "world_params.yaml", "w") as f:
        yaml.safe_dump({k: float(v) if not isinstance(v, list) else [float(x) for x in v] for k, v in params.items()},
                       f, default_flow_style=None, sort_keys=False)


def save_preview(free, walls, origin, size_px, resolution, world_dir):
    """Map image drawn in world coordinates, with walls.yaml on top. If the red lines don't hug the
    black band, or the robot markers aren't in the hallway, map.yaml / walls.yaml disagree."""
    img = np.array(Image.open(world_dir / "map" / "map.png"))
    extent = [origin[0], origin[0] + size_px[0] * resolution, origin[1], origin[1] + size_px[1] * resolution]
    scenario = json.loads((world_dir / "scenarios" / "default.json").read_text())

    fig, ax = plt.subplots(figsize=(14, 14 * size_px[1] / size_px[0] + 1.5))
    ax.imshow(img, cmap="gray", extent=extent, origin="upper")
    for (x1, y1), (x2, y2) in walls:
        ax.plot([x1, x2], [y1, y2], color="red", lw=1)
    for robot in scenario["robots"]:
        ax.plot(*robot["start"][:2], "bs", label="start")
        ax.plot(*robot["goal"][:2], "b*", ms=12, label="goal")
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title(f"{world_dir.name}: map.png (grey) + walls.yaml (red)")
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(world_dir / "preview.png", dpi=120)
    plt.close(fig)


def resolve_path(name, location):
    """Create <location>/<name>/{map,scenarios} and return <location>/<name>."""
    parent_dir = Path(location)
    if not parent_dir.is_dir():
        raise FileNotFoundError(f"output location {parent_dir} doesn't exist")

    if not name:  # auto-name: school_hallway_v0, _v1, ...
        name = f"school_hallway_v{len(glob(str(parent_dir / 'school_hallway*')))}"

    world_dir = parent_dir / name
    (world_dir / "map").mkdir(parents=True, exist_ok=True)
    (world_dir / "scenarios").mkdir(parents=True, exist_ok=True)
    return world_dir


def link_into_arena(world_dir, arena_worlds=ARENA_WORLDS):
    """Make the world visible to Arena by symlinking each FILE into simulation-setup/worlds/<name>/.

    Arena's setup.py collects worlds with os.walk, which does not descend into symlinked
    directories - so linking the whole folder would silently install nothing. Real directories
    with per-file symlinks work, and edits here show up in Arena without re-linking.
    New files still need: colcon build --symlink-install --packages-select arena_simulation_setup
    """
    target_root = Path(arena_worlds) / world_dir.name
    for src in sorted(world_dir.rglob("*")):
        rel = src.relative_to(world_dir)
        if src.is_dir() or rel.parts[0] not in ("map", "scenarios"):
            continue  # preview.png etc. stay in this repo only
        dst = target_root / rel
        dst.parent.mkdir(parents=True, exist_ok=True)
        if dst.is_symlink():
            dst.unlink()
        elif dst.exists():
            raise FileExistsError(f"{dst} is a real file, not one of our links - not overwriting")
        dst.symlink_to(src.resolve())
    print(f"linked into {target_root}")
    print("next: cd ~/arena_ws && source arena.bash && "
          "colcon build --symlink-install --packages-select arena_simulation_setup")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate a school-hallway world for Arena")
    parser.add_argument("-L", "--length-hallway", type=float, default=60.0, help="main hallway length (m)")
    parser.add_argument("-W", "--width-hallway", type=float, default=12.0, help="main hallway clear width (m)")
    parser.add_argument("-w", "--width-corridor", type=float, default=3.0, help="cross corridor clear width (m)")
    parser.add_argument("-l", "--length-corridor", type=float, default=4.0,
                        help="how far each corridor sticks out from the hallway, per side (m)")
    parser.add_argument("-c", "--corridors", type=float, nargs="+", default=[15.0, 30.0, 45.0],
                        help="x positions of the corridor centrelines (m)")
    parser.add_argument("-r", "--resolution", type=float, default=0.05, help="map metres per pixel")
    parser.add_argument("-n", "--name", type=str, default=None, help="world name (default: school_hallway_vN)")
    parser.add_argument("-o", "--out-location", type=str, default=str(REPO_ROOT / "worlds"),
                        help="folder to create the world in (default: <repo>/worlds)")
    parser.add_argument("--link", action="store_true",
                        help="also symlink the world into Arena's simulation-setup/worlds")
    parser.add_argument("--arena-worlds", type=str, default=str(ARENA_WORLDS),
                        help="Arena worlds folder used by --link")

    args = parser.parse_args()
    world_dir = write_hallway_world(
        hall_length=args.length_hallway,
        hall_width=args.width_hallway,
        corridor_width=args.width_corridor,
        corridor_length=args.length_corridor,
        corridor_pos=args.corridors,
        name=args.name,
        location=args.out_location,
        resolution=args.resolution,
    )
    if args.link:
        link_into_arena(world_dir, args.arena_worlds)
