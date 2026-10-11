# 12. Recording episodes, playback and analysis

Goal: every Arena episode ends up as a small folder of CSV files that you can load in a notebook, summarize with
the same metrics for every planner, and replay as a top-down animation (like the DS-RNN simulator view).

```
arena-launch.sh --record ...   ──►  scripts/arena_recorder.py  ──►  ~/arena_runs/<run>/episode_NNN/*.csv
   (ROS side, Arena's python)                                              │
                                                                           ▼
                               uv env:  arena_analysis  (load_run → summary / plot / play / save_video)
                                        scripts/playback.py  (CLI)   notebooks/episode_playback.ipynb
```

## 1. Recording

```bash
cd ~/Documents/Lyu_Lab/initial-arena-testing
bash scripts/arena-launch.sh --headless --record school_hallway scenario scenario:=hallway_tame.json local_planner:=dsrnn
#   --record          -> ~/arena_runs (or $ARENA_RUNS)
#   --record=/some/dir -> there instead
```

The launcher prints `recording episodes under ...`, and the recorder logs one line per episode start/end:

```
[arena_recorder]: recording to ~/arena_runs/20261011-021540_dsrnn_hallway_tame (waiting for the next 'Task Reset!')
[arena_recorder]: episode 0 started at t=24.4s -> episode_000
[arena_recorder]: episode 0 ended: success after 65.6s (657 samples)
```

Stop the simulation as usual (Ctrl+C or `scripts/arena-stop.sh`). The recorder notices that `ros2 launch` is gone,
closes the running episode with outcome `incomplete` and exits.

**Recording by hand** (e.g. a sim you started without `--record`): start it in a second terminal *before* the first
"Task Reset!", because it can only see episodes that start while it runs:

```bash
cd ~/arena_ws && source arena.bash && unset FASTRTPS_DEFAULT_PROFILES_FILE
python ~/Documents/Lyu_Lab/initial-arena-testing/scripts/arena_recorder.py \
    --launch-args local_planner:=dsrnn world:=school_hallway scenario:=hallway_tame.json
```

Useful options (`--help` for all): `--out DIR`, `--rate 10` (samples per sim second), `--robot-radius 0.27`,
`--goal-tolerance 0.5`, `--timeout 600` (must match `timeout` in `task_generator.yaml`, used to label timeouts),
`--plan-topic plan`.

### Why a plain script and not a colcon package

* It only *subscribes*; nothing in Arena depends on it, so there's nothing to build or install. Edit and rerun.
* It lives next to the analysis code it feeds, in this repo, versioned together with the file format.
* Arena's sourced python (3.10 venv) already has `rclpy` and `hunav_msgs`. It has **no pandas/pyarrow**, so the
  recorder uses only the standard library and writes **CSV** (pandas reads it in milliseconds at these sizes:
  a 65 s episode is ~1.5 MB, a 600 s timeout ~12 MB).

### How episodes and outcomes are detected

Read from `task_generator/node.py`, `tasks/task_factory.py` and our `robot_manager.py` patch:

| Signal | Type | Meaning for the recorder |
|---|---|---|
| `/reset_start` | `std_msgs/Empty` | the task generator starts a reset → **close** the running episode (before the robot is teleported) |
| `/task_generator_node/task_reset` | `std_msgs/Int16` | reset done: robot at its start, goal sent → **start** episode `data` (0, 1, 2, …) |
| `/task_generator_node/jackal/navigate_to_pose/_action/status` | `action_msgs/GoalStatusArray` | Nav2 goal states; any goal of *this* episode `SUCCEEDED` → `success` |
| `/task_generator_node/jackal/goal_pose` | `PoseStamped` | the episode's goal (published right before `task_reset`) |

Outcome labels in `meta.json`:

| `outcome` | When |
|---|---|
| `success` | Nav2 reported a goal of this episode as succeeded (this is also what makes Arena reset) |
| `timeout` | the episode lasted ≥ `timeout` − 1 s (Arena resets after `timeout` sim seconds) |
| `aborted` | ended otherwise and Nav2's last goal state was aborted |
| `reset` | ended for another reason (e.g. "Reset Task" clicked in RViz) |
| `incomplete` | the recorder or the simulation stopped mid-episode |

Nav2 shows ~20 "aborted" goals per episode in `events.csv`. That's normal: our `robot_manager` re-sends the goal
every 3 s for the first 60 s (an upstream workaround), and each new goal pre-empts the old one, which Nav2 reports
as aborted. Only the *last* state matters for `aborted`.

`success` relies on Nav2's own goal checker; `reached_goal` in the summary (below) re-checks it from the recorded
positions, so planners that don't use Nav2's goal checker are judged the same way.

### Timing (sim time everywhere)

* Robot rows are taken from `odom` (~30 Hz): the first odometry message after each 0.1 s grid point, and `t` is
  **that message's sim-time stamp**. So `t` spacing is 0.1 s ± 1/30 s and pose and time always match. (A sim-time
  timer was tried first; it fired irregularly and made finite-difference speeds jump to 3 m/s.)
* `human_states`' header stamp is **wall-clock** time (HuNav), so it's unusable. Pedestrian rows reuse the robot
  row's `t` (so they join exactly) and also store `t_msg`, the sim time the pedestrian message arrived.
* `t` is absolute sim seconds since Gazebo started. `meta["t_start"]` is the episode start; the analysis package
  adds `t_rel = t - t_start`.

### Batches: many episodes, many planners (`scripts/arena_batch.py`)

For experiments, don't babysit launches: describe them in a jobs file and let the scheduler run them one after
another (one simulation at a time; each uses ~5.5 of the 8 cores).

```yaml
# my_experiment/jobs.yaml
out: data                        # run folders go here (relative to this file)
defaults: {world: school_hallway, scenario: hallway_tame.json, episodes: 20, headless: true}
jobs:
  - {name: dsrnn, planner: dsrnn}
  - {name: dwb, planner: dwb}
  - {name: dwb_amcl, planner: dwb, args: ["localization:=amcl"]}   # any extra ros2 launch args
```

```bash
uv run scripts/arena_batch.py my_experiment/jobs.yaml --dry-run   # list the jobs + a rough ETA
uv run scripts/arena_batch.py my_experiment/jobs.yaml             # run them (Ctrl+C stops cleanly)
uv run scripts/arena_batch.py my_experiment/jobs.yaml --resume    # after an interruption: skip finished jobs
uv run scripts/arena_batch.py --planner dsrnn dwb --episodes 5 --out ~/arena_runs/quick   # one-off, no file
```

Per job, the scheduler:
1. takes the shared simulator lock `/tmp/arena-sim.lock` (it waits if another batch holds it);
2. launches `arena-launch.sh --headless --record=<out>`;
3. waits until N episodes are complete, printing a live ETA;
4. stops the sim and appends an entry to `<out>/batch_manifest.json` (run folder, episodes, status `complete` /
   `stalled` / `failed`, minutes).

Launch logs go to `<out>/logs/<job>.log`. A job is abandoned as `stalled` if no new episode starts for 15 min.
Run it in the background with `nohup ... > batch.log 2>&1 &` if you want to log out.
See [`first_experiment/`](../first_experiment/) for a complete example (jobs file → data → notebook).

## 2. File format

```
~/arena_runs/<YYYYmmdd-HHMMSS>_<local_planner>_<scenario>/
├── run.json             launch args, planner, world, scenario, robot_radius, rate_hz, repo_commit
├── scenario.json        copy of the scenario JSON that was used (agent classes, waypoints, radii)
├── world/               copy of worlds/<world>/map/ (map.png, map.yaml, walls.yaml) → runs stay plottable
└── episode_000/
    ├── meta.json        episode metadata and outcome
    ├── robot.csv        one row per sample (10 Hz sim time)
    ├── humans.csv       one row per sample × pedestrian
    ├── plan.csv         global plans, only when they change
    └── events.csv       episode start/end and Nav2 goal-state changes
```

Units: metres, radians, seconds, m/s, rad/s. Frame: `map` (= `odom` with `localization:=ground_truth`).

### `robot.csv`

| Column | Meaning |
|---|---|
| `t` | sim time of the odometry message (s) |
| `x`, `y`, `yaw` | true robot pose in the map frame (ground-truth odometry) |
| `odom_vx`, `odom_vy`, `odom_wz` | odometry twist as published: **robot (body) frame**, forward / sideways speed and yaw rate. `odom_vy` is ±0.15 m/s while turning: skid-steer slip in Gazebo's odometry, not real sideways motion |
| `cmd_nav_vx`, `cmd_nav_wz` | latest `cmd_vel_nav`: what the controller / policy asked for |
| `cmd_vx`, `cmd_wz` | latest `cmd_vel`: what reached the robot after Nav2's velocity smoother |

Added by `arena_analysis` when loading: `t_rel`, `vx`, `vy` (world-frame velocity from finite differences),
`speed`, `goal_dist`.

### `humans.csv`

| Column | Meaning |
|---|---|
| `t` | the robot row's `t` this pedestrian state belongs to (join key) |
| `t_msg` | sim time the `human_states` message arrived (≤ 1/30 s from `t`) |
| `id`, `name`, `group_id` | HuNav agent id (1…N), scenario name (e.g. `runner_0`), group (−1 = alone) |
| `x`, `y`, `yaw` | pedestrian position and heading in the map frame |
| `vx`, `vy` | velocity **as reported by HuNav**. ⚠️ In ~40% of messages it glitches to ~0.04 m/s while the person is walking. Use `vx_fd`, `vy_fd` instead |
| `radius` | the agent's radius from the scenario (0.35 m in hallway_tame); used for collision checks |

Added when loading: `t_rel`, `vx_fd`, `vy_fd` (finite differences over ~0.4 s using `t_msg`).

### `plan.csv`

| Column | Meaning |
|---|---|
| `t` | sample time at which this plan was first seen |
| `plan_idx` | 0, 1, 2… one per stored plan |
| `i`, `x`, `y` | point index and position along the plan |

Points are thinned to ≥ 0.25 m spacing (Nav2 puts one every 5 cm). A plan is stored only if it differs from the
previous one by > 5 cm somewhere. `ep.plan_at(t)` returns the plan that was current at time `t`.

### `events.csv`

`t, event, detail` with events `episode_start` (`index=N`), `nav2_goal` (`<goal id>:<state>`), `episode_end`
(`<outcome>`).

### `meta.json`

`local_planner`, `global_planner`, `world`, `scenario`, `robot`, `robot_radius`, `episode` (Arena's episode
index), `episode_dir`, `t_start`, `t_end`, `duration` (sim s), `start` `[x, y, yaw]` (pose at the first sample),
`goal` `[x, y, yaw]`, `goal_tolerance`, `timeout`, `outcome`, `outcome_source` (`nav2_succeeded` / `duration` /
`nav2_aborted` / `unknown` / `recorder`), `final_goal_distance`, `nav2_goals` (goals sent this episode),
`nav2_final_status`, `wall_start`, `samples`, `plans`.

## 3. Analysis API (`arena_analysis`)

A pure-Python package in `src/arena_analysis/`, installed in this repo's uv env (no ROS needed). In a notebook:

```python
from arena_analysis import load_run
run = load_run("~/arena_runs/20261011-021540_dsrnn_hallway_tame")
run.summary()                 # DataFrame, one row per episode
ep = run.episodes[0]          # or run[0]
ep.summary()                  # dict of metrics
ep.play()                     # inline video
```

| Call | Returns |
|---|---|
| `load_run(path)` | `Run`: `.episodes`, `.meta` (run.json), `.summary(human_radius=None)` |
| `load_episode(path)` | one `Episode` from an `episode_NNN` directory |
| `ep.meta`, `ep.outcome`, `ep.start`, `ep.goal`, `ep.robot_radius` | metadata |
| `ep.robot`, `ep.humans`, `ep.plan`, `ep.events` | DataFrames (section 2), loaded on first use |
| `ep.distances(human_radius=None)` | one row per (sample, pedestrian): `t, id, name, center_dist, clearance, rx, ry, hx, hy` |
| `ep.collisions(human_radius=None)` | one row per contact: `id, name, t_start, t_end, duration, min_clearance` |
| `ep.summary(human_radius=None)` | dict of metrics (below) |
| `ep.plan_at(t)` | the global plan current at sim time `t` |
| `ep.world` | `World`: `.walls`, `.map_image`, `.bounds`, `.wall_distance(x, y)` |
| `ep.plot(t=None, view="full")` | draw one frame `t` seconds after the episode start (same clock as the frame's label; default: last sample), returns the Axes |
| `ep.animation(speed=4, fps=20, view="full")` | matplotlib `FuncAnimation` |
| `ep.play(speed=4, fps=20, view="full")` | Jupyter: inline HTML5 video (JS player with slider if no ffmpeg) |
| `ep.save_video("x.mp4", speed=4, fps=20)` | writes .mp4 (ffmpeg) or .gif; returns the path written |

Style options for `plot` / `animation` / `play` / `save_video`: `view="full"|"follow"`, `window=16` (m, follow
view width), `trail=None` (seconds of robot trail, None = whole path), `human_velocity="fd"|"recorded"`,
`figsize`, `show_ids`.

### Metrics (`ep.summary()`)

Collisions are computed from positions, the same way for every planner:
**clearance = centre distance − robot radius − pedestrian radius**, and clearance < 0 is a contact.
Robot radius is `meta["robot_radius"]` (0.27 m), pedestrian radius HuNav's `radius` (0.35 m), or
`human_radius=` to override (e.g. `ep.summary(human_radius=0.3)` for a stricter body-size definition).

| Key | Meaning |
|---|---|
| `outcome`, `success` | from the metadata; `success` = Nav2 reported the goal reached |
| `reached_goal`, `final_goal_dist`, `time_to_goal` | from positions: within `goal_tolerance` (0.5 m) at the end / first time within it |
| `duration` | episode length (sim s) |
| `path_length`, `path_efficiency` | metres driven; straight start→goal distance ÷ path length (only if reached) |
| `mean_speed`, `max_speed` | path_length ÷ duration; max finite-difference speed |
| `n_humans` | pedestrians seen |
| `min_clearance`, `min_center_dist` | closest approach to any pedestrian |
| `collisions`, `collision_time` | number of contacts (overlapping samples < 0.25 s apart = one contact), seconds in contact |
| `intrusion_time` | seconds with any pedestrian within 0.5 m clearance ("personal space") |
| `min_wall_clearance`, `wall_collision` | closest approach to `walls.yaml` minus robot radius; < 0 = touched a wall |
| `cmd_tracking_err` | mean \|cmd_vel_nav − cmd_vel\| linear speed: how much the velocity smoother changed the policy's commands |

At 10 Hz a pedestrian moves ~13 cm between samples, so a very short graze can be missed (or a 2 cm overlap be
counted). Fine for comparing planners over many episodes; for a single borderline contact, look at the replay.

## 4. Playback

**CLI** (uv env, no ROS):

```bash
uv run scripts/playback.py ~/arena_runs/<run>/episode_000                        # window (needs a display)
uv run scripts/playback.py ~/arena_runs/<run>/episode_000 --save ep0.mp4         # video; .gif also works
uv run scripts/playback.py ~/arena_runs/<run>/episode_001 --view follow --save ep1.mp4
uv run scripts/playback.py ~/arena_runs/<run>/episode_000 --frames 0 20 40 --save-frames /tmp/frames   # PNGs
uv run scripts/playback.py ~/arena_runs/<run>                                    # run dir: summary table
```

Every episode command first prints `ep.summary()`. `--speed 4` (playback speed vs sim time) and `--fps 20` are the
defaults. Rendering is CPU-bound: ~100 s for a 65 s episode at the defaults; use `--speed 8 --fps 10` for quick looks.

**Notebook:** `uv run jupyter lab`, open `notebooks/episode_playback.ipynb`. It loads the newest run in
`~/arena_runs`, shows the metrics table, the raw tables, one frame, speed and nearest-pedestrian plots, the
contacts, and the replay.

What you see: walls (black, from `walls.yaml`) over the occupancy map (grey = outside); the global plan (green
dashes); the robot (yellow disc, red heading arrow) and its trail (orange); pedestrians as numbered discs (HuNav id)
with velocity arrows, orange when within 0.5 m of the robot and red when overlapping it; the goal (red star) and
start (grey ×); a status line with episode time, speed, commanded speed, distance to goal and how many people are
within personal space.

Videos use the ffmpeg binary bundled with the `imageio-ffmpeg` package (the VM has no system ffmpeg). Without it,
`save_video` falls back to a GIF and `play` to a JS player.

## 5. Known quirks (2026-10-11)

* **Pedestrians jump at the start of each episode.** In the first ~0.3 s after "Task Reset!" HuNav reports agents
  at their scenario start and then moves them ~5 m in a few steps (a large first integration step after the reset).
  Their finite-difference speeds spike to 20–35 m/s there. It's a simulator artifact; it shows up in the data as is.
* HuNav's `vx`, `vy` glitch (see `humans.csv`); use `vx_fd`, `vy_fd`.
* Episode 2+ pedestrian behaviour depends on the reset bug in [09 §5](09-hallway-scenarios.md#5-performance-notes--running-faster-than-real-time):
  check `ep.humans` before trusting later episodes.

## 6. Verification run (2026-10-11)

`--headless --record`, `hallway_tame.json`, `local_planner:=dsrnn`, run `20261011-021540_dsrnn_hallway_tame`:

| | episode 0 | episode 1 |
|---|---|---|
| outcome | success | success |
| duration / time to goal | 65.6 s / 64.3 s | 84.8 s / 83.9 s |
| path length / efficiency | 62.5 m / 0.93 | 69.3 m / 0.84 |
| mean / max speed | 0.95 / 1.01 m/s | 0.82 / 1.01 m/s |
| min clearance to a pedestrian | 0.13 m | −0.06 m |
| contacts | 0 | 2 (runner_1 at 45.1 s, runner_0 at 49.1 s, ~0.3 s each, ≤ 6 cm overlap) |
| time within personal space | 4.0 s | 13.2 s |
| min wall clearance | 0.83 m | 0.83 m |

Robot samples were evenly spaced (0.100 ± 0.002 s); all 20 pedestrians were present and moving in both episodes.
