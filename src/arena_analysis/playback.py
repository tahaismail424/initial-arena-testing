"""Top-down 2-D replay of an episode, styled after the DS-RNN / CrowdNav simulator view.

Layers (bottom to top): occupancy map (light grey), walls (black), global plan (green dashes), robot trail,
goal (red star), pedestrians (numbered blue discs + velocity arrows; orange = within 0.5 m, red = overlapping
the robot), robot (yellow disc + red heading arrow), and a time stamp.

    ep.plot(t=30.0)                       # one frame
    ep.play(speed=4)                      # Jupyter: inline video at 4x sim time
    ep.save_video("ep.mp4", speed=4)      # .mp4 (ffmpeg bundled via imageio-ffmpeg) or .gif
"""

import shutil
import warnings
from pathlib import Path

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FFMpegWriter, FuncAnimation, PillowWriter
from matplotlib.patches import Circle, FancyArrow

from arena_analysis.metrics import PERSONAL_SPACE

ROBOT_COLOR = "#f5c518"
TRAIL_COLOR = "#e08a00"
HUMAN_COLOR = "#4a7fc1"
CLOSE_COLOR = "#f28e2b"
HIT_COLOR = "#d62728"
PLAN_COLOR = "#2ca02c"


def ffmpeg_path() -> str | None:
    """Path of an ffmpeg binary: the one bundled with imageio-ffmpeg (in the uv env), else one on PATH."""
    try:
        import imageio_ffmpeg
        return imageio_ffmpeg.get_ffmpeg_exe()
    except Exception:
        return shutil.which("ffmpeg")


class Scene:
    """A figure with the static world drawn once and the moving things redrawn by update(t)."""

    def __init__(self, ep, ax=None, view: str = "full", window: float = 16.0, trail: float | None = None,
                 human_velocity: str = "fd", figsize=None, show_ids: bool = True):
        self.ep = ep
        self.view, self.window, self.trail = view, window, trail
        self.human_velocity, self.show_ids = human_velocity, show_ids
        self.robot = ep.robot.reset_index(drop=True)
        # Pedestrian rows grouped by tick. The recorder writes robot and pedestrian rows of one tick with the same
        # t text, so the parsed floats are identical and can be dict keys. (Don't round them: pandas and Python
        # round some values differently, which silently dropped the pedestrians from ~5% of the frames.)
        self.humans_by_t = {t: g for t, g in ep.humans.groupby("t")}
        self.bounds = self._bounds()

        if ax is None:
            xmin, xmax, ymin, ymax = self._view_box(0)
            aspect = (ymax - ymin) / (xmax - xmin)
            width = 14 if figsize is None else figsize[0]
            figsize = figsize or (width, max(3.0, width * aspect + 0.9))
            fig, ax = plt.subplots(figsize=figsize)
        self.ax, self.fig = ax, ax.figure
        self._dynamic = []
        self._draw_static()

    # ------------------------------------------------------------------ layout
    def _bounds(self):
        b = self.ep.world.bounds
        xs = list(self.robot["x"]) + ([self.ep.goal[0]] if self.ep.goal else [])
        ys = list(self.robot["y"]) + ([self.ep.goal[1]] if self.ep.goal else [])
        if b is None:
            b = (min(xs), max(xs), min(ys), max(ys)) if xs else (0, 10, 0, 10)
        pad = 1.0
        return b[0] - pad, b[1] + pad, b[2] - pad, b[3] + pad

    def _view_box(self, i: int):
        """Axis limits for sample i: the whole world, or a window x (9/16 window) box around the robot."""
        xmin, xmax, ymin, ymax = self.bounds
        if self.view == "follow" and len(self.robot):
            def clamp(c, half, lo, hi):   # keep the box inside the world when the world is bigger than the box
                return min(max(c, lo + half), hi - half) if hi - lo > 2 * half else (lo + hi) / 2
            hx, hy = self.window / 2, self.window * 9 / 32
            cx = clamp(float(self.robot["x"].iloc[i]), hx, xmin, xmax)
            cy = clamp(float(self.robot["y"].iloc[i]), hy, ymin, ymax)
            return cx - hx, cx + hx, cy - hy, cy + hy
        return xmin, xmax, ymin, ymax

    def _draw_static(self):
        ax, w = self.ax, self.ep.world
        if w.map_image is not None:
            # map.png: dark = occupied. Show it faintly under the walls (origin='upper': row 0 is the top).
            ax.imshow(w.map_image, cmap="gray", extent=w.map_extent, origin="upper", alpha=0.25, zorder=0,
                      interpolation="nearest")
        for (x0, y0), (x1, y1) in w.walls:
            ax.plot([x0, x1], [y0, y1], color="black", lw=2.0, solid_capstyle="round", zorder=1)
        if self.ep.goal is not None:
            ax.plot(*self.ep.goal[:2], marker="*", color=HIT_COLOR, markersize=18, markeredgecolor="black",
                    zorder=3, label="goal")
        if self.ep.start is not None:
            ax.plot(*self.ep.start[:2], marker="x", color="grey", markersize=8, zorder=3)
        ax.set_aspect("equal")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        xmin, xmax, ymin, ymax = self._view_box(0)
        ax.set_xlim(xmin, xmax)
        ax.set_ylim(ymin, ymax)

    # ------------------------------------------------------------------ per frame
    def index_at(self, t: float | None) -> int:
        if t is None or not len(self.robot):
            return len(self.robot) - 1
        return int(np.clip(np.searchsorted(self.robot["t"].to_numpy(), t), 0, len(self.robot) - 1))

    def update(self, i: int):
        for artist in self._dynamic:
            artist.remove()
        self._dynamic = []
        if not len(self.robot):
            return []
        ax, add = self.ax, self._dynamic.append
        r = self.robot.iloc[i]
        t = float(r["t"])
        rr = self.ep.robot_radius

        # global plan that was current at time t
        plan = self.ep.plan_at(t)
        if len(plan):
            add(ax.plot(plan["x"], plan["y"], ls="--", color=PLAN_COLOR, lw=1.5, zorder=2)[0])

        # robot trail
        first = 0 if self.trail is None else self.index_at(t - self.trail)
        add(ax.plot(self.robot["x"].iloc[first:i + 1], self.robot["y"].iloc[first:i + 1],
                    color=TRAIL_COLOR, lw=1.5, alpha=0.8, zorder=2)[0])

        # pedestrians
        humans = self.humans_by_t.get(t)
        n_close = 0
        if humans is not None:
            vx_col, vy_col = ("vx", "vy") if self.human_velocity == "recorded" else ("vx_fd", "vy_fd")
            for h in humans.itertuples(index=False):
                clearance = np.hypot(h.x - r["x"], h.y - r["y"]) - rr - h.radius
                color = HIT_COLOR if clearance < 0 else CLOSE_COLOR if clearance < PERSONAL_SPACE else HUMAN_COLOR
                n_close += clearance < PERSONAL_SPACE
                add(ax.add_patch(Circle((h.x, h.y), h.radius, facecolor=color, edgecolor="black", lw=0.8,
                                        alpha=0.75, zorder=4)))
                vx, vy = getattr(h, vx_col), getattr(h, vy_col)
                if np.hypot(vx, vy) > 0.05:
                    add(ax.add_patch(FancyArrow(h.x, h.y, vx, vy, width=0.04, head_width=0.2, head_length=0.2,
                                                length_includes_head=True, color="black", zorder=5)))
                if self.show_ids:
                    add(ax.text(h.x, h.y, str(h.id), ha="center", va="center", fontsize=7, color="white",
                                fontweight="bold", zorder=6))

        # robot: disc + heading arrow
        add(ax.add_patch(Circle((r["x"], r["y"]), rr, facecolor=ROBOT_COLOR, edgecolor="black", lw=1.2, zorder=7)))
        L = max(3 * rr, 0.8)   # long enough to see at whole-hallway zoom
        add(ax.add_patch(FancyArrow(r["x"], r["y"], L * np.cos(r["yaw"]), L * np.sin(r["yaw"]), width=0.05,
                                    head_width=0.22, head_length=0.18, length_includes_head=True,
                                    color=HIT_COLOR, zorder=8)))

        # time stamp + status line
        m = self.ep.meta
        status = f"t = {float(r['t_rel']):6.1f} s   v = {r['speed']:.2f} m/s   cmd = {r['cmd_vx']:.2f} m/s"
        if "goal_dist" in r:
            status += f"   to goal {r['goal_dist']:.1f} m"
        if n_close:
            status += f"   people < {PERSONAL_SPACE} m: {n_close}"
        add(ax.text(0.01, 0.98, status, transform=ax.transAxes, ha="left", va="top", fontsize=10,
                    family="monospace", bbox=dict(facecolor="white", alpha=0.8, edgecolor="none"), zorder=10))
        title = f"episode {m.get('episode')} · {m.get('local_planner')} · {m.get('scenario')}"
        if i == len(self.robot) - 1 and m.get("outcome"):
            title += f" · {m['outcome']}"
        ax.set_title(title)

        if self.view == "follow":
            xmin, xmax, ymin, ymax = self._view_box(i)
            ax.set_xlim(xmin, xmax)
            ax.set_ylim(ymin, ymax)
        return self._dynamic


def plot_frame(ep, t: float | None = None, ax=None, **style):
    """Draw the episode at sim time t (absolute sim seconds, like the `t` column; default: last sample)."""
    scene = Scene(ep, ax=ax, **style)
    scene.update(scene.index_at(t))
    return scene.ax


def make_animation(ep, speed: float = 4.0, fps: int = 20, step: int | None = None, **style) -> FuncAnimation:
    """FuncAnimation over the recorded samples.

    speed  playback speed relative to sim time (4 = a 60 s episode plays in 15 s)
    fps    frames per second of the output
    step   use every `step`-th sample (default: chosen from speed and fps; recording rate is 10 Hz)
    style  passed to Scene: view="full"|"follow", window (m, follow view), trail (s, None = whole path),
           human_velocity="recorded"|"fd", figsize, show_ids
    """
    scene = Scene(ep, **style)
    rate = float((ep.run.meta.get("rate_hz") if ep.run else None) or 10.0)
    if step is None:
        step = max(1, round(speed * rate / fps))
    frames = list(range(0, len(scene.robot), step))
    if frames and frames[-1] != len(scene.robot) - 1:
        frames.append(len(scene.robot) - 1)   # always end on the final state (shows the outcome)
    scene.fig.tight_layout()
    anim = FuncAnimation(scene.fig, scene.update, frames=frames, interval=1000 / fps, blit=False)
    anim._scene = scene   # keep a reference (and let callers reach the figure)
    return anim


def play(ep, **kwargs):
    """Inline animation for Jupyter. HTML5 <video> when ffmpeg is available (small), else a JS player."""
    from IPython.display import HTML

    anim = make_animation(ep, **kwargs)
    plt.close(anim._fig)   # don't also show the static figure below the player
    ff = ffmpeg_path()
    if ff:
        matplotlib.rcParams["animation.ffmpeg_path"] = ff
        return HTML(anim.to_html5_video(embed_limit=200))
    matplotlib.rcParams["animation.embed_limit"] = 200   # MB
    return HTML(anim.to_jshtml())


def save_video(ep, filename, dpi: int = 100, **kwargs) -> Path:
    """Save as .mp4 (needs ffmpeg; bundled via imageio-ffmpeg) or .gif. Falls back to .gif without ffmpeg."""
    filename = Path(filename).expanduser()
    anim = make_animation(ep, **kwargs)
    fps = kwargs.get("fps", 20)
    if filename.suffix.lower() != ".gif":
        ff = ffmpeg_path()
        if ff:
            matplotlib.rcParams["animation.ffmpeg_path"] = ff
            anim.save(filename, writer=FFMpegWriter(fps=fps, bitrate=2400), dpi=dpi)
            plt.close(anim._fig)
            return filename
        warnings.warn("ffmpeg not found (install imageio-ffmpeg); writing a GIF instead")
        filename = filename.with_suffix(".gif")
    anim.save(filename, writer=PillowWriter(fps=fps), dpi=dpi)
    plt.close(anim._fig)
    return filename
