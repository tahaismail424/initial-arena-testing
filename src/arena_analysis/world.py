"""Static world geometry for plotting and wall-distance checks: wall segments (walls.yaml) and the occupancy map."""

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml

REPO = Path(__file__).resolve().parents[2]


@dataclass
class World:
    name: str
    walls: list = field(default_factory=list)   # [[(x0, y0), (x1, y1)], ...] in the map frame (metres)
    map_image: np.ndarray | None = None         # occupancy image, row 0 = top, as loaded from map.png
    map_extent: tuple | None = None             # (xmin, xmax, ymin, ymax) of the image in metres

    @classmethod
    def load(cls, name: str, *search_dirs: Path) -> "World":
        """Find `<dir>/walls.yaml`, `<dir>/map.yaml` in the given dirs, then in this repo's worlds/<name>/map/.

        The recorder copies the world's map/ folder into each run (`<run>/world/`), so a run stays plottable even
        if the world is regenerated later. Missing files are fine: you just get fewer layers in the plot."""
        candidates = [Path(d) for d in search_dirs] + [REPO / "worlds" / name / "map"]
        world = cls(name=name)
        for d in candidates:
            if (d / "walls.yaml").is_file() and not world.walls:
                with open(d / "walls.yaml") as f:
                    world.walls = [[tuple(p) for p in seg] for seg in yaml.safe_load(f)["walls"]]
            if (d / "map.yaml").is_file() and world.map_image is None:
                world._load_map(d / "map.yaml")
        return world

    def _load_map(self, map_yaml: Path):
        """ROS map_server format: origin = (x, y) of the image's BOTTOM-left pixel, resolution = metres/pixel."""
        import matplotlib.image as mpimg

        with open(map_yaml) as f:
            info = yaml.safe_load(f)
        img = mpimg.imread(map_yaml.parent / info["image"])
        if img.ndim == 3:
            img = img[..., :3].mean(axis=2)
        h, w = img.shape
        res = float(info["resolution"])
        x0, y0 = float(info["origin"][0]), float(info["origin"][1])
        self.map_image = img
        self.map_extent = (x0, x0 + w * res, y0, y0 + h * res)

    @property
    def bounds(self) -> tuple | None:
        """(xmin, xmax, ymin, ymax) of the walls (preferred) or of the map image."""
        if self.walls:
            pts = np.array([p for seg in self.walls for p in seg])
            return pts[:, 0].min(), pts[:, 0].max(), pts[:, 1].min(), pts[:, 1].max()
        return self.map_extent

    def wall_distance(self, x, y) -> np.ndarray:
        """Distance (m) from each point (x[i], y[i]) to the nearest wall segment. NaN if there are no walls."""
        x, y = np.asarray(x, float), np.asarray(y, float)
        if not self.walls:
            return np.full(x.shape, np.nan)
        import shapely

        lines = shapely.multilinestrings([np.asarray(seg, float) for seg in self.walls])
        return shapely.distance(shapely.points(x, y), lines)
