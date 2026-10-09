# 6. Local patches we made & gotchas to know about

## Local modifications to Arena (uncommitted)

These live in the upstream checkouts. A `git pull` / `vcs import` / reinstall could lose or conflict with them. They are
**exported to [`../patches/`](../patches/)** (see its README to regenerate or re-apply). The first-session fixes also have
`.before-*` backups next to the edited files. Later edits are marked with the comment `initial-arena-testing` instead:
`grep -rn initial-arena-testing ~/arena_ws/src/arena` lists them.

### arena-rosnav (`~/arena_ws/src/arena/arena-rosnav`)

| File | Change | Why |
|---|---|---|
| `task_generator/task_generator/manager/world_manager/world_manager_ros.py` | Map subscription gets its own `MutuallyExclusiveCallbackGroup` | **Startup deadlock**: the task generator waited for the map while holding the callback group needed to receive it. The robot never spawned. |
| `task_generator/task_generator/simulators/human/hunav/hunav.py` | `self._wall_segments = []` initialised | Crash in worlds, like the hospital, whose walls come only from Gazebo geometry |
| `arena_bringup/launch/arena.launch.py`, `task_generator/launch/task_generator.launch.py` | New launch arg `localization` (`amcl` default / `ground_truth`), forwarded to `task_generator_node` as a parameter | Switchable localization, see [08](08-ground-truth-localization.md) |
| `.../simulators/sim/gazebo_simulator/gazebo_simulator.py` | Reads `localization`. Fills `{odom_source}` in the bridge mappings. In ground-truth mode publishes one identity `map → <robot>/odom` instead of a new static publisher per reset. | [08](08-ground-truth-localization.md) |
| `.../manager/robot_manager/robot_manager.py` | Ground truth: no AMCL, no static identity `odom → base_link` | [08](08-ground-truth-localization.md) |
| same | **Episode-reset fix**: `reset()` clears the goal-reached flag, and `_goal_status_callback` ignores Nav2 goals from earlier episodes | Upstream bug: after the first success, Nav2's retained `SUCCEEDED` status made every new episode count as done, so the task **reset every ~2 s forever** |
| same | **Spawn-height fix**: `move_robot_to_pos` works on a copy of the pose | Upstream bug: `z_offset` was added to the scenario's start pose in place, so the robot spawned ~6 cm higher on every reset and dropped onto the floor |
| `arena_bringup/launch/arena.launch.py`, `task_generator/launch/task_generator.launch.py` | New launch arg `scenario` (default `default.json`), passed as `task.scenario.file` **after** `task_generator.yaml` so it wins | Pick a scenario per run, e.g. `scenario:=hallway_tame.json` ([09](09-hallway-scenarios.md)) |
| `.../gazebo_simulator/gazebo_simulator.py` (`_generate_wall_sdf`), `.../human/hunav/hunav.py` (`plugin_entity`) | Wall visual/collision renamed `arena_wall_visual`/`arena_wall_collision` and added to the HuNav plugin's `<ignore_models>` | HuNav plugin bug: rotated wall boxes became phantom walls that trapped pedestrians ([09 §4](09-hallway-scenarios.md#4-pitfalls-we-hit-and-the-fix-in-the-generator)) |
| `arena_bringup/launch/arena.launch.py`, `task_generator/launch/task_generator.launch.py` | New launch arg `rviz` (`true`/`false`); `false` also skips the RViz-only `pedestrian_marker_publisher` | Gazebo window without RViz ([09 §5](09-hallway-scenarios.md#5-performance-notes--running-faster-than-real-time)) |
| `.../human/hunav/hunav.py` | HuNav plugin `update_rate` comment. Value kept at 1000 (unchanged behaviour). | See the hunav_gz_plugin row below |
| `arena_bringup/configs/task_generator.yaml` | `timeout: 600` (sim seconds) | Episodes end instead of hanging when the robot gets stuck. **Must be an integer**: Arena declares this param with default `-1`, and ROS 2 rejects a `600.0` (double) override, which crashes the task generator at startup. |

### simulation-setup (`~/arena_ws/src/arena/simulation-setup`)

| File | Change | Why |
|---|---|---|
| `worlds/hospital/worlds/hospital.world` | `<world name="world">` → `"default"` | Arena's bridges expect world `default` in Gazebo topic names |
| same | sensors render engine `ogre` → `ogre2`, added `gz-sim-imu-system` | Match the working empty.sdf. Lidar and IMU need these. |
| same | `max_step_size 0.001 → 0.0333`, `real_time_update_rate 1000 → 0` | Sim time was crawling |
| `worlds/hospital/scenarios/default.json` | Added `"model": "actor1"` to 4 pedestrians missing it | They failed to spawn |
| `entities/robots/jackal/urdf/jackal.gazebo` | Added `OdometryPublisher` (true pose on `/model/<robot>/ground_truth/{odometry,tf}`) | Ground-truth localization, [08](08-ground-truth-localization.md) |
| `entities/robots/jackal/mappings.yaml` | Odometry and TF bridge entries use `{odom_source}` | Switches wheel odometry ↔ ground truth |
| `configs/nav2/nav2.yaml` | `bt_navigator.default_server_timeout: 200` (was the Nav2 default of 20 ms) | Under load the planner missed the 20 ms window and the goal was aborted for good ([09](09-hallway-scenarios.md#4-pitfalls-we-hit-and-the-fix-in-the-generator)) |
| `worlds/school_hallway/` (untracked) | Per-file symlinks into this repo's `worlds/school_hallway/` | Created by `scripts/generate_hallway.py --link` |

### hunav_gz_plugin (`~/arena_ws/src/gazebo/hunav_gz_plugin`, C++, needs `colcon build --packages-select hunav_gz_plugin`)

| File | Change | Why |
|---|---|---|
| `src/HuNavSystemPlugin.cpp` `PreUpdate` | Commented out a leftover debug loop that logged every entity every step | Pure overhead |
| same + `include/.../HuNavSystemPlugin.h` | `lastUpdate_` initialised to 0, and a negative `dt` never counts as "too soon" | **Bug in our first update_rate patch:** `lastUpdate_` was uninitialised, so on the CPU VM garbage made `dt` negative and the plugin skipped every step. Pedestrians stood still from episode 1 (found 2026-10-09). |
| same | `<update_rate>` is now honoured (steps are skipped below the rate) | Upstream parsed it but never used it. With the value at 1000 nothing changes; lower values speed the sim up but slowed pedestrians ([09 §5](09-hallway-scenarios.md#5-performance-notes--running-faster-than-real-time)) |

See the current state any time with:

```bash
git -C ~/arena_ws/src/arena/arena-rosnav diff
git -C ~/arena_ws/src/arena/simulation-setup diff
```

**Only the Jackal has the ground-truth plugin.** Using `localization:=ground_truth` with another robot means adding the
same `OdometryPublisher` block to its Gazebo description and `{odom_source}` to its `mappings.yaml`.

## Open issues found in the configs (not yet changed)

| # | Issue | Where | Impact | Suggested fix |
|---|---|---|---|---|
| 1 | **`controller_frequency: 1.0`**: the local planner runs at 1 Hz. Verified live via `ros2 param get`. | `simulation-setup/configs/nav2/nav2.yaml` | Any controller reacts once per second. Likely a big part of DWB's oscillation and stalling in the hospital, and unfair to reactive crowd-nav policies. | Try `10.0`–`20.0`, the Nav2 default is 20. Compare before and after as your first real config experiment. |
| 2 | **Footprint is 0.2 × 0.2 m** but Jackal is ~0.42 × 0.33 m (`robot_radius: 0.267`) | `entities/robots/jackal/model_params.yaml` (+ same in `configs/nav2/model_params.yaml`) | Costmap underestimates the robot. It squeezes through gaps it can't fit, and clearance metrics are wrong. | Set the real rectangle (see [04 D](04-configuring-sim-and-robot.md#d-the-robot)) |
| 3 | **Lidar metadata ≠ simulated lidar**: 720 beams/270°/30 m declared vs 640/360°/12 m simulated (16 vertical rows) | `model_params.yaml → laser` vs `urdf/jackal.gazebo` | Learned policies with fixed input sizes break or misbehave | Decide on one sensor config per policy and make both files agree |
| 4 | **Velocity limits disagree**: DWB 0.26 m/s & 1 rad/s vs RL action space ±2 m/s & ±4 rad/s vs MPPI 0.5 m/s & 1.9 rad/s | controller configs, `model_params.yaml → actions` | Unfair comparisons: a faster controller "wins" on time | Pick one effective envelope for all compared policies |
| 5 | ~~Episode `timeout` defaults to infinite~~ **Fixed:** `timeout: 600` in `task_generator.yaml` | `task_generator.yaml` (param `timeout`) | Without it a stuck robot never resets and batch evaluations hang. Seen for real in `school_hallway` with AMCL. | Done. Keep it an integer. |
| 9 | **Goal is only re-sent for 60 s** after a reset, and the default behaviour tree has no recovery for a failed plan | `robot_manager.py` `_publish_goal_callback`; `configs/nav2/interplanners/` | One planning failure late in an episode ends navigation for good (only the timeout, #5, gets you out) | Rely on #5 for now. A recovery-enabled `inter_planner` is worth testing later. |
| 10 | **AMCL loses track in long uniform corridors** (6° heading drift in `school_hallway`), and upstream also runs competing static TF publishers | AMCL / `gazebo_simulator.py` | Robot believed to be inside a wall → all plans fail | **Fixed for our benchmarks** with `localization:=ground_truth` ([08](08-ground-truth-localization.md)). With `amcl`, the competing static publishers are still there. |
| 6 | ~~Pedestrians ignore the robot (`force_factor_robot: 0.0`)~~ **Corrected:** the `hunavsim` block is unused. The real trap is that **`behavior.social_force_factor` defaults to 0**, so pedestrians pass through each other. | scenario JSON / `hunav_agents/default.yaml` | Unrealistic crowd | Always set it per agent; `scripts/generate_scenario.py` does ([09](09-hallway-scenarios.md)) |
| 7 | HuNav obstacle forces skip mesh geometry | `hunav_gz_plugin` | Pedestrians may walk through hospital walls | Validate visually. Possibly add wall segments. |
| 8 | Hospital start heading `0.7` rad faces away from the corridor | `hospital/scenarios/default.json` | DWB spun in place at start in the chat session (at 1 Hz control) | Re-test after fixing #1 before changing the scenario |

## Environment gotchas

- **`FASTRTPS_DEFAULT_PROFILES_FILE`** is set by `arena.bash` to `~/.ros/fastdds.xml`, which doesn't exist. Our launch script unsets it. If you launch manually, do the same.
- **`ROS_DOMAIN_ID=1`**: un-sourced terminals see nothing.
- **Always `vglrun -d egl0`** for GUI launches on the VM.
- **Zombie Gazebo**: after Ctrl+C, check `ps` for `gz sim` and kill leftovers before relaunching. Two Gazebos cause baffling behaviour.
- **Build wrapper skip logic**: `. colcon_build` skips packages whose `install/` dir is newer than `src/`, including packages that previously *failed*. Use `SKIP_OLD=0` or `--packages-select`.
- **`nav2_system_tests`** is `COLCON_IGNORE`d on purpose, because it requires Gazebo Classic, which conflicts with Harmonic.
- **`turtlebot4_ignition_gui_plugins`** needs `CPLUS_INCLUDE_PATH=/usr/include/ignition/cmake2` to build.
- **Worlds without a `.world` file** (`house17`, `generated`) silently load `empty.sdf`, so there are no walls in Gazebo even though the 2D map has them.
- **Upstream docs lag the code.** The readthedocs pages mix ROS 1 (Arena 3) and ROS 2 (Arena 4/5) instructions. For example, the evaluation README says `record_data:=true`, but the humble launch arg is `record_data_dir:=<name>`.
- **Multiple robots:** names come from the model. Check `ros2 param get /task_generator_node robot_names` to see what namespaces were created.
