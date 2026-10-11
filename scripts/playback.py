"""
Replay a recorded episode (scripts/arena_recorder.py output) as a top-down 2-D animation.

  uv run scripts/playback.py ~/arena_runs/<run>/episode_000                    # window (needs a display)
  uv run scripts/playback.py ~/arena_runs/<run>/episode_000 --save ep0.mp4     # video (.mp4 or .gif)
  uv run scripts/playback.py ~/arena_runs/<run>/episode_000 --frames 0 30 60 --save-frames /tmp/frames
  uv run scripts/playback.py ~/arena_runs/<run>                                # a run dir: summary table only

Runs in this repo's uv env (pandas/matplotlib/imageio-ffmpeg), no ROS needed. API and file format: docs/12.
"""

import argparse
from pathlib import Path

import pandas as pd

from arena_analysis import load_episode, load_run


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("path", help="episode directory (…/episode_000) or run directory (summary only)")
    p.add_argument("--save", help="write a video instead of opening a window (.mp4, or .gif)")
    p.add_argument("--speed", type=float, default=4.0, help="playback speed vs sim time (default 4x)")
    p.add_argument("--fps", type=int, default=20, help="frames per second (default 20)")
    p.add_argument("--view", choices=["full", "follow"], default="full",
                   help="whole world, or a window that follows the robot (see --window)")
    p.add_argument("--window", type=float, default=16.0, help="width of the follow window (m)")
    p.add_argument("--trail", type=float, default=None, help="seconds of robot trail (default: whole path)")
    p.add_argument("--frames", type=float, nargs="*",
                   help="instead of a video, save PNGs at these times (seconds since episode start)")
    p.add_argument("--save-frames", default=".", help="directory for --frames PNGs (default: current dir)")
    args = p.parse_args()

    path = Path(args.path).expanduser()
    pd.set_option("display.width", 200)
    pd.set_option("display.max_columns", 30)
    if not (path / "meta.json").is_file():
        run = load_run(path)
        print(run)
        print(run.summary().round(3).to_string(index=False))
        return

    ep = load_episode(path)
    print(ep)
    for key, value in ep.summary().items():
        print(f"  {key:20s} {value:.3f}" if isinstance(value, float) else f"  {key:20s} {value}")

    style = dict(view=args.view, window=args.window, trail=args.trail)
    if args.frames is not None:
        import matplotlib.pyplot as plt

        out = Path(args.save_frames).expanduser()
        out.mkdir(parents=True, exist_ok=True)
        t0 = ep.meta.get("t_start", 0.0)
        for t_rel in args.frames:
            ax = ep.plot(t_rel, **style)
            f = out / f"{path.parent.name}_{path.name}_t{t_rel:06.1f}.png"
            ax.figure.tight_layout()
            ax.figure.savefig(f, dpi=100)
            plt.close(ax.figure)
            print(f"wrote {f}")
        return

    if args.save:
        written = ep.save_video(args.save, speed=args.speed, fps=args.fps, **style)
        print(f"wrote {written}")
    else:
        import matplotlib.pyplot as plt

        anim = ep.animation(speed=args.speed, fps=args.fps, **style)  # keep a reference while the window is open
        plt.show()


if __name__ == "__main__":
    main()
