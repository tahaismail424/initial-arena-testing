# 8. Ground-truth localization: what we changed and why

**TL;DR:** launch with `localization:=ground_truth` (our `scripts/arena-launch.sh` does this by default) and the
robot is handed its exact Gazebo pose, with no AMCL. `localization:=amcl` restores upstream behaviour.

This guide is written as a worked example of **adding a configurable feature to Arena**. The same pattern
(launch arg → ROS param → Python branch → config file) is how you'll add most things yourself.

---

## 1. The problem we hit

First run of `school_hallway`: the Jackal drove 33 m, then Nav2 failed every replan and the episode hung.

| Source (same instant) | x | y | heading |
|---|---|---|---|
| Gazebo truth (`gz model -m jackal -p`) | 34.46 | 0.96 | 3.2° |
| Wheel odometry (spawn + `odom`) | 34.45 | 2.00 | 5.4° |
| AMCL (what Nav2 used) | 33.34 | **−1.31** | ≈ −0.6° |

* **AMCL** matches lidar scans to the map. A long uniform hallway looks the same everywhere, so AMCL's heading
  drifted ~6°. Over 33 m that put the robot 3.3 m sideways, *inside a wall*, and the planner can't plan from inside a wall.
* **Wheel odometry** is accurate along the direction of travel (1 cm error over 33 m) but drifts on turns, because
  Jackal is **skid-steer**: it turns by scrubbing its wheels sideways, and the DiffDrive plugin's formula assumes clean
  rolling. The error grows with how much a policy turns, which makes it a confounder.
* Arena also stacked conflicting static TF publishers. That's covered below.

For benchmarking *navigation policies* we don't want localization quality to decide outcomes, so we give every robot
its **true** pose. Later, localization error can be reintroduced *on purpose* (see §6).

## 2. How a robot's pose reaches Nav2 (the TF chain)

Nav2 never reads "the robot's position" directly. It asks TF for `map → <robot>/base_link`, which is the product of two links:

```
map ──(A)──► jackal/odom ──(B)──► jackal/base_link
```

| | Link A: `map → odom` | Link B: `odom → base_link` |
|---|---|---|
| **upstream (amcl)** | AMCL (dynamic) **+** a static publisher Arena starts on *every* robot move (they fight) | Gazebo DiffDrive wheel odometry **+** a static identity publisher from `robot_manager` (they fight) |
| **ours (ground_truth)** | ONE static identity transform (map and Gazebo world share coordinates) | Gazebo **OdometryPublisher**: the model's true world pose |

With ground truth, `map → base_link` = identity × true pose = **true pose**. The `odom` topic Nav2 uses for velocity
also comes from the true-pose plugin.

## 3. What we changed: 4 Arena files + 2 of ours

All edits are marked with the comment `initial-arena-testing`, so `grep -rn initial-arena-testing ~/arena_ws/src/arena`
finds every one. Full diffs are in [`../patches/`](../patches/).

### Step 1: publish the true pose inside Gazebo
`simulation-setup/entities/robots/jackal/urdf/jackal.gazebo` gets a second plugin next to DiffDrive:

```xml
<plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
  <odom_frame>$(arg name)/odom</odom_frame>
  <robot_base_frame>$(arg name)/base_link</robot_base_frame>
  <odom_topic>/model/$(arg name)/ground_truth/odometry</odom_topic>
  <tf_topic>/model/$(arg name)/ground_truth/tf</tf_topic>
  <odom_publish_frequency>50</odom_publish_frequency>
  <dimensions>2</dimensions>
</plugin>
```

It publishes on its **own** Gazebo topics (`.../ground_truth/...`), so adding it changes nothing until something
bridges those topics into ROS. `$(arg name)` becomes the robot's name (`jackal`) when Arena spawns it.

> How we found the parameter names: the headers aren't installed, so we ran
> `strings /usr/lib/x86_64-linux-gnu/libgz-sim8-odometry-publisher-system.so.8 | grep -E "odom|tf_topic|noise"`.
> Handy trick for any Gazebo plugin.

### Step 2: make the bridge source switchable
`simulation-setup/entities/robots/jackal/mappings.yaml` lists which Gazebo topics get bridged to which ROS topics.
Two entries got a placeholder:

```yaml
"gz_topic": "/model/{robot_name}{odom_source}/odometry",   # → ROS  <robot>/odom
"gz_topic": "/model/{robot_name}{odom_source}/tf",         # → ROS  /tf
```

`{robot_name}` and `{world}` were already placeholders that Arena fills in with Python's `str.format`. We added
`{odom_source}`: `""` for wheel odometry (upstream behaviour, unchanged) or `"/ground_truth"` for the true pose.

### Step 3: fill the placeholder and fix link A (`gazebo_simulator.py`)
`arena-rosnav/task_generator/task_generator/simulators/sim/gazebo_simulator/gazebo_simulator.py`

* `__init__`: reads the ROS parameter once:
  `self._ground_truth = self.node.rosparam[str].get('localization', 'amcl') == 'ground_truth'`
* `_robot_bridge`: passes `'odom_source': '/ground_truth' if self._ground_truth else ''` to the mapping substitution.
* `move_entity`: in ground-truth mode, publishes a single identity `map → <robot>/odom` the first time a robot is
  placed and returns early. Upstream code (still used for amcl) starts a *new* static publisher at the spawn pose on
  every reset.

### Step 4: turn off AMCL and fix link B (`robot_manager.py`)
`arena-rosnav/task_generator/task_generator/manager/robot_manager/robot_manager.py`

* reads the same parameter in `__init__`
* `_launch_robot`: passes `amcl: 'false'` to the robot's Nav2 launch when ground truth is on
  (`simulation-setup/launch/nav2.launch.py` already had an `IfCondition` on that argument, so no change was needed there)
* `set_up_robot`: skips `_odom_base_transform()`, the static identity `odom → base_link` that would fight the real one

### Step 5: plumb the switch from the command line to the node
A launch argument has to be passed down through every launch file between the command line and the node:

```
ros2 launch arena_bringup arena.launch.py localization:=ground_truth
   │  arena.launch.py         : LaunchArgument('localization', default 'amcl', choices [amcl, ground_truth])
   │                            passed into the include with **localization.dict
   ▼
task_generator.launch.py      : LaunchArgument('localization', default 'amcl')
   │                            passed to the node as a parameter with **localization.str_param
   ▼
task_generator_node           : ROS parameter "localization" → read by gazebo_simulator.py and robot_manager.py
```

Arena's `LaunchArgument` helper (in `arena_bringup/substitutions.py`) gives you `.dict` (for launch includes) and
`.str_param` (for node parameters), so each hop is one line. You can check it live:
`ros2 param get /task_generator_node localization`.

### Step 6: our side
* `scripts/arena-launch.sh` now adds `localization:=ground_truth` by default. Anything you pass after the task mode
  overrides it, e.g. `arena-launch.sh school_hallway scenario localization:=amcl`.
* `scripts/check_run.py` prints the believed pose (TF) vs the true pose (Gazebo) and the Nav2 goal status. Use it to
  validate any localization change.

### Rebuild needed?
No. Every file we edited already existed and is symlinked into `install/` (see [02](02-workspace-and-build.md)). A
relaunch picks the changes up.

## 4. How we validated it

1. **Unit check of the bridge mapping**: loaded `mappings.yaml` with Arena's own `BridgeConfiguration` class and
   printed the generated bridge arguments in both modes. amcl mode produces exactly the original topics.
2. **Full run**: `school_hallway`, default scenario, DWB + NavFn, `localization:=ground_truth`, with
   `check_run.py` in a second terminal. Results are in §5.

## 5. Results

Validation run (2026-09-28): `school_hallway`, default scenario, DWB + NavFn, `localization:=ground_truth`.

| | AMCL (before) | Ground truth (after) |
|---|---|---|
| Distance covered | 33.5 m of 58 m, then stuck forever | **full 58 m, "Reached the goal!"** |
| Believed vs true pose | 3.3 m sideways, 6° heading | y error 0.00 m, heading error 0.0° (the ~0.1 m x "error" is the `gz` CLI lag while moving) |
| Path | wandered ±0.5 m | dead centre of the hallway (y = 1.50) the whole way |
| Episode time | n/a | **248 s, then 243 s** (58 m at ~0.24 m/s, DWB's 0.26 m/s cap) |
| Reset after success | n/a | exactly one reset per success, robot back at (1, 1.5) at z = 0.0635, two consecutive episodes both reached the goal |

(This validation used the original 3 m-wide hallway. The world is now 12 m wide; see [09](09-hallway-scenarios.md).)

### Two upstream bugs this exposed (fixed, same file)
Neither shows up until a robot actually *reaches* its goal, which never happened in the hospital runs:

1. **Reset loop.** After a success Arena reset every ~2 s forever. `robot_manager._goal_status_callback` called the
   robot "done" if the *last* entry of Nav2's goal-status list was `SUCCEEDED`, but Nav2 keeps finished goals in that
   list, and `reset()` never cleared the flag. Fix: `reset()` clears the flag and records the goal ids Nav2 already
   knows. The callback then only counts goals that are new since the reset. (We compare goal **ids**, not timestamps,
   because `task_generator_node` and Nav2 may run on different clocks.)
2. **Spawn-height creep.** `move_robot_to_pos` added the robot's `z_offset` to the scenario's start pose *in place*.
   Since that pose object is reused every episode, the spawn height grew 0.19 → 0.25 → … → 0.70 m and the robot was
   dropped onto the floor. Fix: `copy.deepcopy(pose)` first.

Lesson: **validate multi-episode behaviour, not just one episode.** Benchmarks run hundreds of resets.

## 6. Later: making localization realistic again, on purpose

* **Controlled noise:** OdometryPublisher supports `<gaussian_noise>` (and `<xyz_offset>`/`<rpy_offset>`), so you can
  add a *known* amount of pose error and treat it as an experimental variable.
* **AMCL with landmarks:** add features along the hallway (door frames, lockers, pillars) via the world generator so
  AMCL has something to lock onto, then run with `localization:=amcl`. Crowds occluding the lidar will make this much
  harder, which could be a research question in itself.

## 7. The general recipe (reuse this)

1. **Find where the behaviour lives.** grep the launch files and `task_generator` for the node or topic name.
2. **Add a launch argument** in `arena.launch.py` and forward it through each launch file to the node (`.dict` / `.str_param`).
3. **Read it in Python** with `self.node.rosparam[<type>].get('<name>', <default>)`.
4. **Branch**, keeping the upstream behaviour as the default.
5. **Mark every edit** with a greppable comment and **export a patch** (`git diff > patches/<repo>.patch`).
6. **Validate with a script**, not by eye: compare what the software believes with ground truth.
