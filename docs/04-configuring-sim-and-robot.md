# 4. Configuring the simulation and the robot

Short aliases used below:

```bash
AR=~/arena_ws/src/arena/arena-rosnav        # orchestrator
SS=~/arena_ws/src/arena/simulation-setup    # assets & robot/nav2 configs
```

Remember from [02](02-workspace-and-build.md): **editing an existing file needs only a relaunch. Adding a file needs a `colcon build --packages-select <pkg>`.**

---

## A. Task generator (episodes, scenarios, random obstacles)

**File:** `$AR/arena_bringup/configs/task_generator.yaml`. It is loaded into `/task_generator_node` for **every** world.

```yaml
/**:
  ros__parameters:
    task:
      scenario:
        file: default.json        # ← which worlds/<world>/scenarios/*.json to use (tm_*:=scenario)
      random:                     # ← used by tm_obstacles:=random
        dynamic: {n: [1, 3], models: [gazebo_actor]}
        static:  {n: [3, 5], models: [shelf]}
        interactive: {n: [0, 0], models: [shelf]}
        seed: -1                  # ≥0 for reproducible random episodes
      parametrized: {file: default.xml}
      staged: {curriculum: default.yaml, index: 0}
      environment: {file: office.yaml}
    hunavsim:
      parameters: {max_vel: 0.3, force_factor_desired: 1.0, force_factor_obstacle: 1.0,
                   force_factor_social: 5.0, force_factor_robot: 0.0}
```

Other `task_generator_node` parameters, defined in `$AR/task_generator/task_generator/constants/runtime.py`, that you can add under `ros__parameters:`:

| Param | Default | Meaning |
|---|---|---|
| `timeout` | `-1` (= **never**) | Episode timeout in sim-seconds. ⚠️ With the default, a stuck robot never triggers a reset. Set it (e.g. `120.0`) for any evaluation. |
| `episodes` | `-1` (= infinite) | Shut down after N episodes. Good for batch evaluation. |
| `goal_tolerance_radius` | `1.0` | How close counts as "reached" for Arena. Nav2's goal checker has its own 0.25 m tolerance. |
| `robot_safe_dist` | `0.25` | Clearance used when spawning |
| `rng` | `-1` | Global RNG seed |

> Scenario choice is global, not per world. Every world has its own `default.json`, so `file: default.json` "just works" when you switch worlds. For a non-default scenario, either set `file:` here or copy the scenario as a new file and point to it.

### Scenario JSON format

Located at `$SS/worlds/<world>/scenarios/<name>.json`. Coordinates are in the `map` frame, in metres. `[x, y, yaw]`.

```json
{
  "robots": [ {"start": [0, 15, 0.7], "goal": [0, -25, 0]} ],
  "obstacles": {
    "static":  [ {"name": "shelf1", "model": "shelf", "pos": [15.3, 19.6, -0.5]} ],
    "dynamic": [ {"name": "1", "model": "actor1", "type": "adult",
                  "pos": [-0.72, -7.53, 0], "waypoints": [[0, 15, 0]]} ],
    "interactive": []
  }
}
```

- `model` must match a directory in `$SS/entities/obstacles/{static,dynamic}/` (e.g. `actor1`, `actor2`, `gazebo_actor`, `shelf`, `patient_bed`, `nurse_station` …).
- Pedestrian extras read by the HuNav converter: `desired_velocity`, `cyclic_goals`, `behavior`. The legacy `waypoint_mode` field is **ignored**.
- Missing per-pedestrian settings fall back to `agent1` in `$AR/arena_bringup/configs/hunav_agents/default.yaml`: `max_vel 0.3`, `radius 0.4`, behaviour `REGULAR`, `cyclic_goals: true`.
- The previous static audit of all scenario files is in `~/Documents/ChatGPT/Master's Research/arena-scenario-audit/`.

**Workflow to create your own:** copy an existing JSON into the same folder with a new name. Edit it, run `colcon build --packages-select arena_simulation_setup`, set `task.scenario.file` to it, and relaunch.

---

## B. Pedestrians (HuNav)

Three layers decide how people move:
1. **Scenario JSON**: start pose, waypoints, model, optional behaviour.
2. **`hunav_agents/default.yaml`**: per-agent defaults and the behaviour type: `REGULAR=1, IMPASSIVE=2, SURPRISED=3, SCARED=4, CURIOUS=5, THREATENING=6`. Plus force factors.
3. ~~`task_generator.yaml → hunavsim.parameters`~~ **Correction (verified in code 2026-09-28): this block is not read by anything.** What actually controls the forces is each agent's `behavior` block (scenario JSON, else `agent1`). Watch out for these:
   - **`behavior.social_force_factor` defaults to 0**, and HuNav uses it as the social (people-avoid-people) force. Unless you set it, pedestrians walk *through each other*.
   - `goal_force_factor` is hard-coded to 20 in Arena's HuNav adapter; your value is ignored.
   - With behaviour `REGULAR` (1), pedestrians treat the robot as another person and avoid it through the social force. `IMPASSIVE` (2) treats it as an obstacle. Types 3–6 (`SURPRISED`, `SCARED`, `CURIOUS`, `THREATENING`) trigger reactions when the robot comes close.

   See [09](09-hallway-scenarios.md) for the full field list.

Code: `$AR/task_generator/task_generator/simulators/human/hunav/{__init__.py,hunav.py}` and the Gazebo plugin `~/arena_ws/src/gazebo/hunav_gz_plugin/src/HuNavSystemPlugin.cpp` (C++, needs a rebuild).

Known limitation: HuNav's obstacle forces skip **mesh** geometry, and the hospital's walls are meshes. So pedestrians may clip through walls. Check visually before trusting the crowd behaviour.

---

## C. World & physics

A world is `$SS/worlds/<name>/`:

```
map/map.yaml + map.pgm|png     2D occupancy map → Nav2 map_server (MUST align with the 3D world)
map/obstacles.yaml             optional world-level obstacles spawned by Arena
worlds/<name>.world            Gazebo SDF. If missing → empty.sdf fallback (house17, generated!)
scenarios/*.json
```

Arena's requirements for a `.world` file. We learned these while fixing the hospital:
- `<world name="default">`. The bridges build Gazebo topic names with the world name, so any other name means no lidar or odom.
- The sensors plugin with `<render_engine>ogre2</render_engine>`, and an `Imu` system plugin.

Physics block (in the `.world`):

```xml
<physics default="0" name="default_physics" type="ode">
  <max_step_size>0.0333</max_step_size>        <!-- seconds per physics step; bigger = faster, less accurate -->
  <real_time_factor>1</real_time_factor>       <!-- target speed vs wall clock -->
  <real_time_update_rate>0</real_time_update_rate>  <!-- 0 = run as fast as possible -->
</physics>
```

The hospital originally had a 1 ms step at 1000 Hz, which crawled. We changed it to 33 ms, the same as Arena's `empty.sdf`. For research runs, record the physics settings you used. 33 ms is coarse for fast contacts but OK for a 0.26 m/s Jackal.

Gazebo launch flags live in `$AR/arena_bringup/launch/simulator/sim/gazebo/gazebo.launch.py`: `-r` (start running), `--render-engine ogre`, and `-s` (server only) when headless.

**Adding a new world:** create the dir structure above and run `colcon build --packages-select arena_simulation_setup`. Then launch with `world:=<name>`. You can make the 2D map from the 3D world by driving around with SLAM (`slam_toolbox` is installed), or with arena_tools' map generator.

---

## D. The robot

Everything about a robot lives in `$SS/entities/robots/<robot>/`. For **Jackal**:

| File | Controls | Used by |
|---|---|---|
| `urdf/jackal.urdf.xacro` | Geometry: `wheelbase 0.262`, `track 0.37559`, `wheel_radius 0.098`, links, meshes | Gazebo spawn, robot_state_publisher |
| `urdf/jackal.gazebo` | **Gazebo plugins**: DiffDrive, IMU (50 Hz), **gpu_lidar** (10 Hz, 640 samples × 16 rows, 360°, 0.08–12 m, σ=0.01 noise) | Gazebo |
| `mappings.yaml` | Which Gazebo topics get bridged to which ROS topics (`odom`, `imu/data`, `cmd_vel`, `lidar`, `lidar/points`, …) | Per-robot `ros_gz_bridge` |
| `model_params.yaml` | Nav2 inputs: `observation_sources` (costmap sensors), `footprint`, frames. **Also** RL metadata: `actions`, `laser`, `robot_radius`, `is_holonomic` | `nav2.launch.py` merge + RL tools |
| `control.yaml` | ros2_control DiffDriveController params (not the active path in Gazebo; the gz DiffDrive plugin is) | — |
| `jackal.model.yaml` | Flatland 2D model (legacy simulator) | — |

### Common robot changes

**Speed and acceleration limits.** These belong to the *controller*, not the robot: `$SS/configs/nav2/controllers/<planner>/controller_config.yaml`. DWB currently has `max_vel_x: 0.26`, `max_vel_theta: 1.0`, `acc_lim_x: 2.5`. Also check `velocity_smoother` limits, which use Nav2 defaults because they aren't in `nav2.yaml`. Dump them with `ros2 param dump /task_generator_node/jackal/velocity_smoother`. The lower of the two limits wins.

**Footprint.** Set in `model_params.yaml`. Currently a **0.2 × 0.2 m square**, but the real Jackal is about 0.42 × 0.33 m (see `jackal.model.yaml`) and `robot_radius` says 0.267. The costmap therefore thinks the robot is tiny. Fix before any collision/clearance comparison:

```yaml
footprint: "[ [0.21, 0.165], [0.21, -0.165], [-0.21, -0.165], [-0.21, 0.165] ]"
```

**Lidar.** Change it in `urdf/jackal.gazebo` (`samples`, `min/max_angle`, `range/max`, `update_rate`, `noise`). ⚠️ `model_params.yaml → laser` currently claims **720 beams / 270° / 30 m**, but the simulated sensor is **640 beams / 360° / 12 m**. Any learned policy with a fixed input size cares about this. Make them agree, and make them match whatever the policy was trained on. Also `gpu_lidar` with 16 vertical rows gets flattened into a `LaserScan`. Consider `<vertical><samples>1</samples>` for a true 2D scan.

**Sensors for costmaps.** `model_params.yaml → observation_sources_dict` sets which topic feeds the costmaps (currently `${namespace}/lidar`).

**Adding a robot.** Copy an existing robot dir, e.g. `turtlebot`, to a new name. Edit the URDF, `mappings.yaml` and `model_params.yaml`, build `arena_simulation_setup`, then launch with `robot:=<name>`.

---

## E. The Nav2 YAML merge chain

This is the most confusing part of Arena, and the part you'll touch most. `$SS/launch/nav2.launch.py` builds Nav2's parameter file like this:

```
            configs/nav2/model_params.yaml                       (global defaults)
   merged ← entities/robots/<robot>/model_params.yaml            (robot overrides)
   merged ← configs/nav2/controllers/<local_planner>/controller_config.yaml
   merged ← configs/nav2/planners/<global_planner>/planner_config.yaml
   merged ← configs/nav2/interplanners/<inter_planner>/interplanner_config.yaml
   merged ← {frame, namespace, task_generator_node, bt xml paths}
                                   │
                                   ▼   substituted into ${...} placeholders of
                          configs/nav2/nav2.yaml   (the actual Nav2 template)
```

So in `nav2.yaml` you'll see things like:

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 1.0                    # ← hard-coded in the template (see 06!)
    controller_plugins: ${controller_plugins}    # ← from controller_config.yaml
    '${**controller_plugins_dict}': ''           # ← splices the whole dict in (FollowPath: {...})
local_costmap: ... footprint: ${footprint}       # ← from robot model_params.yaml
```

Rules of thumb:
- **Robot-specific** (footprint, sensors, frames) → `entities/robots/<robot>/model_params.yaml`
- **Policy-specific** (plugin name, speeds, critics) → `controllers/<name>/controller_config.yaml`
- **Global Nav2 behaviour** (controller rate, costmap size/inflation, goal checker, progress checker, AMCL) → `configs/nav2/nav2.yaml`
- When in doubt, `ros2 param dump` the live node to see the merged result.

### Key knobs in `nav2.yaml` worth knowing

| Param | Current | Why you care |
|---|---|---|
| `controller_server.controller_frequency` | **1.0 Hz** | Policy is queried once per second. Far too slow for crowds. Nav2 default is 20 Hz. |
| `progress_checker.movement_time_allowance` | 10 s | Goal aborted if the robot moves < 0.5 m in 10 s |
| `goal_checker.xy_goal_tolerance` | 0.25 m | Nav2's "arrived" radius |
| `local_costmap` width/height | 15 × 15 m, 0.1 m res | |
| `inflation_radius` | local 0.55, global 0.25 | How much clearance the planners want around obstacles |
| `global_costmap` width/height/origin | 20 × 20 m | Effectively ignored: the static layer resizes the costmap to the map. Verified live: the hospital global costmap is 800×800 cells at 0.1 m = 80 × 80 m, origin (-40,-40). |
