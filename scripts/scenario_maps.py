"""Per-scenario Nav2 maps: rasterize a scenario's static obstacles (wall panels) into a copy of the world's map.

Why: Nav2's global planner plans on the world's map.png, which knows nothing about the panels a scenario spawns.
Without a per-scenario map the plan goes straight through a panel wall until the lidar sees it (docs/14 §11).
Arena (our arena-rosnav patch, docs/14 §9) loads the map named in the scenario's optional top-level field

    "map": "scenario_maps/<scenario_stem>.yaml"        # relative to worlds/<world>/map/

at every reset that applies the scenario (before the robot is placed and the goal is sent), and restores the
world's own map.yaml for scenarios without the field.

Usage from a generator (pure Python: numpy / Pillow / PyYAML / shapely, runs in this repo's uv env):

    from scenario_maps import write_scenario_map
    scenario = builder.scenario()                         # the dict you are about to json.dump
    write_scenario_map(world_dir, "door_exit_s3", scenario)
    # -> writes worlds/campus/map/scenario_maps/door_exit_s3.{png,yaml}
    #    sets scenario["map"] = "scenario_maps/door_exit_s3.yaml" and returns that string
    json.dump(scenario, open(world_dir / "scenarios" / "door_exit_s3.json", "w"))

Then link and rebuild as for any new world file (new files need it, edits don't):
    uv run scripts/generate_campus_scenario.py ... --link      (link_into_arena() also links map/scenario_maps/)
    cd ~/arena_ws && source arena.bash && colcon build --symlink-install --packages-select arena_simulation_setup

CLI (add maps to existing scenario files in place):
    uv run scripts/scenario_maps.py worlds/school_hallway worlds/school_hallway/scenarios/spike_doorway.json
    uv run scripts/scenario_maps.py --self-test

What is rasterized (cells set to occupied = 0 in a copy of the base image; same size, origin, resolution):
  * panel_x<L> / panel_y<L> (L = 05 -> 0.5 m, 1, 2, 4): L x 0.2 m (or 0.2 x L m), origin = footprint centre.
  * cuboid: a 1 x 1 m box whose SDF link sits at (+0.1, +0.5) from the model origin (rotated with the model's yaw);
    docs/14 §11. Other models are skipped with a warning (their footprint is unknown here).
  * obstacles.static and obstacles.interactive (Arena treats both as static).
The footprint is rotated by the obstacle's yaw (pos[2]) exactly as Gazebo does. A cell is occupied when the
footprint overlaps it (so a 0.2 m panel on cell boundaries is exactly 4 cells at 0.05 m). Panels with yaw != 0
get a warning: Gazebo/lidar/the map see the rotated box, but HuNav pedestrians use the UNROTATED one (docs/14 §5).
"""
from __future__ import annotations

import argparse
import json
import math
import re
import sys
import warnings
from pathlib import Path

import numpy as np
import yaml
from PIL import Image

PANEL_THICKNESS = 0.2
SCENARIO_MAP_DIR = "scenario_maps"


def footprint(model: str) -> tuple[float, float, float, float] | None:
    """(size_x, size_y, offset_x, offset_y) of a static model's footprint in its own frame, or None if unknown.
    offset = position of the box centre relative to the model origin (before applying the model's yaw)."""
    m = re.fullmatch(r"panel_([xy])(\d+)", model)
    if m:
        digits = m.group(2)
        length = float("0." + digits[1:]) if digits.startswith("0") else float(digits)
        return (length, PANEL_THICKNESS, 0.0, 0.0) if m.group(1) == "x" else (PANEL_THICKNESS, length, 0.0, 0.0)
    if model == "cuboid":
        return (1.0, 1.0, 0.1, 0.5)     # simulation-setup entities/obstacles/static/cuboid: <pose>0.1 0.5 ...
    return None


def box_corners(x: float, y: float, yaw: float, fp: tuple[float, float, float, float]) -> np.ndarray:
    """4x2 corners (world frame) of the footprint `fp` of a model placed at (x, y, yaw)."""
    sx, sy, ox, oy = fp
    local = np.array([[ox - sx / 2, oy - sy / 2], [ox + sx / 2, oy - sy / 2],
                      [ox + sx / 2, oy + sy / 2], [ox - sx / 2, oy + sy / 2]])
    c, s = math.cos(yaw), math.sin(yaw)
    return local @ np.array([[c, s], [-s, c]]) + [x, y]


def rasterize(img: np.ndarray, origin: tuple[float, float], res: float, corners: np.ndarray, value: int = 0):
    """Set every cell of `img` (row 0 = top, ROS map_server convention) that the polygon `corners` overlaps."""
    h, w = img.shape
    ox, oy = origin
    eps = 1e-6 * res
    xmin, ymin = corners.min(axis=0)
    xmax, ymax = corners.max(axis=0)
    # column / "row from the bottom" index ranges of the bounding box
    c0, c1 = math.floor((xmin - ox) / res + 1e-6), math.ceil((xmax - ox) / res - 1e-6)
    b0, b1 = math.floor((ymin - oy) / res + 1e-6), math.ceil((ymax - oy) / res - 1e-6)
    c0, c1, b0, b1 = max(c0, 0), min(c1, w), max(b0, 0), min(b1, h)
    if c0 >= c1 or b0 >= b1:
        return 0
    axis_aligned = all(min(abs(px - xmin), abs(px - xmax)) < eps and min(abs(py - ymin), abs(py - ymax)) < eps
                       for px, py in corners)
    if axis_aligned:
        mask = np.ones((b1 - b0, c1 - c0), bool)
    else:
        import shapely
        poly = shapely.Polygon(corners)
        cols, rows = np.meshgrid(np.arange(c0, c1), np.arange(b0, b1))
        cells = shapely.box(ox + cols * res, oy + rows * res, ox + (cols + 1) * res, oy + (rows + 1) * res)
        mask = shapely.area(shapely.intersection(cells, poly)) > eps * res
    # mask row k = "row from the bottom" b0 + k; image rows run top-down, so flip it
    img[h - b1:h - b0, c0:c1][mask[::-1]] = value
    return int(mask.sum())


def write_scenario_map(world_dir: Path, scenario_stem: str, scenario: dict) -> str:
    """Write <world_dir>/map/scenario_maps/<scenario_stem>.{png,yaml}: the world's base map plus the scenario's
    static obstacles. Sets scenario["map"] to the path relative to <world_dir>/map/ and returns it.

    world_dir: this repo's worlds/<world> (contains map/map.yaml + its image). The output has the same size,
    origin, resolution and thresholds as the base map, so it can replace it in map_server one-to-one.

    >>> import tempfile, json
    >>> d = Path(tempfile.mkdtemp()); (d / "map").mkdir()
    >>> Image.fromarray(np.full((40, 100), 255, np.uint8)).save(d / "map" / "map.png")   # 5 x 2 m at 0.05 m
    >>> _ = (d / "map" / "map.yaml").write_text("image: map.png\\nresolution: 0.05\\norigin: [0.0, 0.0, 0.0]\\n"
    ...                                         "negate: 0\\noccupied_thresh: 0.65\\nfree_thresh: 0.196\\n")
    >>> sc = {"obstacles": {"static": [{"name": "p", "model": "panel_y1", "pos": [2.0, 1.0, 0.0]}]}}
    >>> write_scenario_map(d, "test", sc)
    'scenario_maps/test.yaml'
    >>> img = np.array(Image.open(d / "map" / "scenario_maps" / "test.png"))
    >>> int((img == 0).sum()), sorted(set(np.nonzero(img == 0)[1].tolist()))   # 0.2 x 1 m = 4 x 20 cells, x 1.9..2.1
    (80, [38, 39, 40, 41])
    """
    world_dir = Path(world_dir)
    map_dir = world_dir / "map"
    base = yaml.safe_load((map_dir / "map.yaml").read_text())
    img = np.array(Image.open(map_dir / base["image"]).convert("L"))
    res = float(base["resolution"])
    origin = (float(base["origin"][0]), float(base["origin"][1]))
    occupied = 255 if base.get("negate", 0) else 0

    obstacles = scenario.get("obstacles", {})
    n_cells, n_obs = 0, 0
    for obs in list(obstacles.get("static", [])) + list(obstacles.get("interactive", [])):
        model = obs.get("model", "")
        fp = footprint(model)
        if fp is None:
            warnings.warn(f"{scenario_stem}: static obstacle {obs.get('name')!r} has model {model!r} with unknown "
                          "footprint; not in the scenario map")
            continue
        x, y = obs["pos"][:2]
        yaw = float(obs["pos"][2]) if len(obs["pos"]) > 2 else 0.0
        if model.startswith("panel_") and abs(math.remainder(yaw, 2 * math.pi)) > 1e-6:
            warnings.warn(f"{scenario_stem}: panel {obs.get('name')!r} is rotated (yaw {yaw:.3f}); the map shows the "
                          "rotated box like Gazebo, but HuNav pedestrians see the unrotated one (docs/14 §5)")
        n_cells += rasterize(img, origin, res, box_corners(float(x), float(y), yaw, fp), occupied)
        n_obs += 1

    out_dir = map_dir / SCENARIO_MAP_DIR
    out_dir.mkdir(parents=True, exist_ok=True)
    Image.fromarray(img).save(out_dir / f"{scenario_stem}.png")
    out_yaml = {**base, "image": f"{scenario_stem}.png"}
    (out_dir / f"{scenario_stem}.yaml").write_text(
        f"# generated by scripts/scenario_maps.py: base map + {n_obs} static obstacles ({n_cells} cells) of "
        f"scenario {scenario_stem}\n" + yaml.safe_dump(out_yaml, sort_keys=False, default_flow_style=None))
    rel = f"{SCENARIO_MAP_DIR}/{scenario_stem}.yaml"
    scenario["map"] = rel
    return rel


def main():
    ap = argparse.ArgumentParser(description="Add per-scenario maps (static panels rasterized) to scenario JSONs, "
                                             "in place. See the module docstring.")
    ap.add_argument("world_dir", nargs="?", type=Path, help="this repo's worlds/<world>")
    ap.add_argument("scenarios", nargs="*", type=Path, help="scenario JSON files to update")
    ap.add_argument("--self-test", action="store_true", help="run the docstring example and exit")
    args = ap.parse_args()
    if args.self_test:
        import doctest
        failures, tests = doctest.testmod(verbose=False)
        print(f"self-test: {tests - failures}/{tests} ok")
        sys.exit(1 if failures else 0)
    if not args.world_dir or not args.scenarios:
        ap.error("give a world dir and scenario files (or --self-test)")
    for f in args.scenarios:
        scenario = json.loads(f.read_text())
        rel = write_scenario_map(args.world_dir, f.stem, scenario)
        f.write_text(json.dumps(scenario, indent=2) + "\n")
        print(f"{f.name}: map -> {rel}")


if __name__ == "__main__":
    main()
