"""
arena_analysis: load, summarize and replay episodes recorded by scripts/arena_recorder.py.

    from arena_analysis import load_run
    run = load_run("~/arena_runs/20261011-021500_dsrnn_hallway_tame")
    run.summary()                 # one row per episode (pandas DataFrame)
    ep = run.episodes[0]
    ep.summary()                  # dict: duration, path length, min distance to people, collisions, success, ...
    ep.robot, ep.humans, ep.plan  # pandas DataFrames, one row per sample (see docs/12 for every column)
    ep.play()                     # inline animation in Jupyter
    ep.save_video("ep0.mp4")      # mp4 (bundled ffmpeg) or .gif

Pure Python (pandas / numpy / matplotlib / shapely); no ROS needed. Full guide: docs/12-recording-and-playback.md.
"""

from arena_analysis.episode import Episode, Run, load_episode, load_run
from arena_analysis.world import World

__all__ = ["Episode", "Run", "World", "load_episode", "load_run"]
