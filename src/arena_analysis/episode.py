"""Loading recorded runs and episodes (the directory layout written by scripts/arena_recorder.py)."""

import json
from functools import cached_property
from pathlib import Path

import numpy as np
import pandas as pd

from arena_analysis import metrics
from arena_analysis.world import World


def _finite_difference(df: pd.DataFrame, by: str | None = None, tcol: str = "t",
                       k: int = 1) -> tuple[np.ndarray, np.ndarray]:
    """World-frame velocity (vx, vy) from x, y samples: (p[i+k] - p[i-k]) / (t[i+k] - t[i-k]), per `by` group.

    k=1 is the usual central difference. A larger k averages over 2k samples, which tames timing noise
    (pedestrian messages arrive in small bursts, so their arrival times jitter by ~1/30 s).
    Rows with the same time `tcol` (one pedestrian message written into two samples) are differenced once
    and share the result, so repeated messages don't produce division by zero."""
    keys = ([by] if by else []) + [tcol]
    uniq = df.drop_duplicates(keys)[keys + ["x", "y"]].sort_values(keys)

    def one(g):
        t, x, y = g[tcol], g["x"], g["y"]
        if len(g) < 2:
            return pd.DataFrame({"vx": 0.0, "vy": 0.0}, index=g.index)
        kk = min(k, (len(g) - 1) // 2) or 1
        # ends of the series: shift() gives NaN there, so fall back to the nearest available sample
        t1, t0 = t.shift(-kk).fillna(t.iloc[-1]), t.shift(kk).fillna(t.iloc[0])
        dt = (t1 - t0).replace(0, np.nan)
        vx = (x.shift(-kk).fillna(x.iloc[-1]) - x.shift(kk).fillna(x.iloc[0])) / dt
        vy = (y.shift(-kk).fillna(y.iloc[-1]) - y.shift(kk).fillna(y.iloc[0])) / dt
        return pd.DataFrame({"vx": vx.fillna(0.0), "vy": vy.fillna(0.0)}, index=g.index)

    groups = [uniq] if by is None else [g for _, g in uniq.groupby(by, sort=False)]
    vel = pd.concat([one(g) for g in groups]) if groups else pd.DataFrame(columns=["vx", "vy"])
    merged = df[keys].merge(uniq[keys].join(vel), on=keys, how="left")
    return merged["vx"].to_numpy(), merged["vy"].to_numpy()


class Episode:
    """One recorded episode. Tables are loaded lazily (on first access) and cached.

    Attributes:
        path   episode directory (…/episode_000)
        meta   dict from meta.json (planner, scenario, start, goal, t_start, t_end, outcome, …)
        robot  DataFrame, one row per sample: t, x, y, yaw, odom/cmd velocities + derived columns
        humans DataFrame, one row per (sample, pedestrian): t, id, name, x, y, vx, vy, radius, …
        plan   DataFrame of global plans: t, plan_idx, i, x, y (one plan_idx per stored plan)
        events DataFrame: t, event, detail (episode start/end, Nav2 goal status changes)
        world  World (walls + map) used for plotting and wall distances
    """

    def __init__(self, path, run: "Run | None" = None):
        self.path = Path(path).expanduser().resolve()
        self.run = run
        with open(self.path / "meta.json") as f:
            self.meta: dict = json.load(f)

    def __repr__(self):
        m = self.meta
        dur = f"{m['duration']:.1f}s" if m.get("duration") is not None else "?"
        return (f"<Episode {m.get('episode')} {m.get('local_planner')} {m.get('scenario')} "
                f"outcome={m.get('outcome')} duration={dur}>")

    # ------------------------------------------------------------------ metadata shortcuts
    @property
    def index(self) -> int:
        return self.meta.get("episode")

    @property
    def outcome(self) -> str:
        return self.meta.get("outcome")

    @property
    def robot_radius(self) -> float:
        return float(self.meta.get("robot_radius") or 0.27)

    @property
    def goal(self) -> tuple | None:
        g = self.meta.get("goal")
        return None if g is None else tuple(g)

    @property
    def start(self) -> tuple | None:
        s = self.meta.get("start")
        return None if s is None else tuple(s)

    # ------------------------------------------------------------------ tables
    @cached_property
    def robot(self) -> pd.DataFrame:
        df = pd.read_csv(self.path / "robot.csv")
        # Derived columns (not in the file):
        #   t_rel        seconds since the episode started
        #   vx, vy       world-frame velocity from finite differences of x, y (independent of odometry conventions)
        #   speed        |(vx, vy)|
        #   goal_dist    distance to the goal (m)
        df["t_rel"] = df["t"] - self.meta.get("t_start", df["t"].iloc[0] if len(df) else 0.0)
        df["vx"], df["vy"] = _finite_difference(df)
        df["speed"] = np.hypot(df["vx"], df["vy"])
        if self.goal is not None:
            df["goal_dist"] = np.hypot(df["x"] - self.goal[0], df["y"] - self.goal[1])
        return df

    @cached_property
    def humans(self) -> pd.DataFrame:
        df = pd.read_csv(self.path / "humans.csv")
        # vx_fd, vy_fd: finite-difference velocity per pedestrian over ~0.4 s, from the message arrival times t_msg.
        # Prefer these over the recorded vx, vy: HuNav's velocity field glitches to ~0.04 in some messages (docs/12).
        df["t_rel"] = df["t"] - self.meta.get("t_start", 0.0)
        if len(df):
            df["vx_fd"], df["vy_fd"] = _finite_difference(df, by="id", tcol="t_msg" if "t_msg" in df else "t", k=2)
        else:
            df["vx_fd"] = df["vy_fd"] = pd.Series(dtype=float)
        return df

    @cached_property
    def plan(self) -> pd.DataFrame:
        return pd.read_csv(self.path / "plan.csv")

    @cached_property
    def events(self) -> pd.DataFrame:
        return pd.read_csv(self.path / "events.csv")

    @cached_property
    def world(self) -> World:
        search = []
        if self.run is not None:
            search.append(self.run.path / "world")
        search.append(self.path.parent / "world")
        return World.load(self.meta.get("world", ""), *search)

    # ------------------------------------------------------------------ analysis
    def distances(self, human_radius: float | None = None) -> pd.DataFrame:
        """Robot-to-pedestrian distance for every (sample, pedestrian); see metrics.robot_human_distances."""
        return metrics.robot_human_distances(self.robot, self.humans, self.robot_radius, human_radius)

    def collisions(self, human_radius: float | None = None) -> pd.DataFrame:
        """One row per robot-pedestrian contact (consecutive overlapping samples merged); see metrics.collisions."""
        return metrics.collisions(self.distances(human_radius))

    def summary(self, human_radius: float | None = None) -> dict:
        """Episode metrics, computed the same way for every planner. See metrics.summarize for each key."""
        return metrics.summarize(self, human_radius=human_radius)

    def plan_at(self, t: float) -> pd.DataFrame:
        """The latest global plan recorded at or before sim time t (empty DataFrame if none yet)."""
        p = self.plan
        if p.empty:
            return p
        earlier = p[p["t"] <= t + 1e-6]
        if earlier.empty:
            return earlier
        return earlier[earlier["plan_idx"] == earlier["plan_idx"].iloc[-1]]

    # ------------------------------------------------------------------ playback (see playback.py)
    def plot(self, t: float | None = None, ax=None, **style):
        """Draw one top-down frame t seconds after the episode start (default: the last sample). Returns the Axes."""
        from arena_analysis.playback import plot_frame
        return plot_frame(self, t, ax=ax, **style)

    def animation(self, **kwargs):
        """matplotlib FuncAnimation of the episode. kwargs: step, fps, speed, view, trail, figsize."""
        from arena_analysis.playback import make_animation
        return make_animation(self, **kwargs)

    def play(self, **kwargs):
        """Inline animation for Jupyter (HTML5 video if ffmpeg is available, else a JS player with a slider)."""
        from arena_analysis.playback import play
        return play(self, **kwargs)

    def save_video(self, filename, **kwargs) -> Path:
        """Write the animation to .mp4 (bundled ffmpeg) or .gif (Pillow). Returns the file actually written."""
        from arena_analysis.playback import save_video
        return save_video(self, filename, **kwargs)


class Run:
    """A run directory: run.json + episode_000/, episode_001/, …"""

    def __init__(self, path):
        self.path = Path(path).expanduser().resolve()
        meta_file = self.path / "run.json"
        self.meta: dict = json.loads(meta_file.read_text()) if meta_file.is_file() else {}
        self.episodes: list[Episode] = [
            Episode(d, run=self) for d in sorted(self.path.glob("episode_*")) if (d / "meta.json").is_file()
        ]

    def __repr__(self):
        return f"<Run {self.path.name}: {len(self.episodes)} episode(s)>"

    def __len__(self):
        return len(self.episodes)

    def __getitem__(self, i) -> Episode:
        return self.episodes[i]

    def summary(self, human_radius: float | None = None) -> pd.DataFrame:
        """One row per episode with the keys of Episode.summary()."""
        return pd.DataFrame([ep.summary(human_radius=human_radius) for ep in self.episodes])


def load_run(path) -> Run:
    """Load a run directory written by arena_recorder.py (e.g. ~/arena_runs/20261011-021500_dsrnn_hallway_tame)."""
    return Run(path)


def load_episode(path) -> Episode:
    """Load a single episode directory (…/episode_000). Its run's world/ copy is used if present."""
    path = Path(path).expanduser().resolve()
    parent = path.parent
    run = Run.__new__(Run)   # light-weight parent: just the path and run.json, without scanning sibling episodes
    run.path = parent
    run.meta = json.loads((parent / "run.json").read_text()) if (parent / "run.json").is_file() else {}
    run.episodes = []
    ep = Episode(path, run=run)
    run.episodes = [ep]
    return ep
