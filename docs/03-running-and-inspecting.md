# 3. Running & inspecting a simulation

## Launching

Use the repo launcher (`scripts/arena-launch.sh`), which wraps the exact command that works on this VM:

```bash
bash ~/Documents/Lyu_Lab/initial-arena-testing/scripts/arena-launch.sh hospital            # fixed hospital scenario
bash ~/Documents/Lyu_Lab/initial-arena-testing/scripts/arena-launch.sh map_empty explore   # random-goal demo
# extra args are passed straight through to ros2 launch:
bash ~/Documents/Lyu_Lab/initial-arena-testing/scripts/arena-launch.sh hospital scenario local_planner:=mppi log_level:=info
```

Under the hood it runs this (from a desktop terminal):

```bash
cd ~/arena_ws && source arena.bash
unset FASTRTPS_DEFAULT_PROFILES_FILE          # points to a file that doesn't exist here
/opt/VirtualGL/bin/vglrun -d egl0 \
  ros2 launch arena_bringup arena.launch.py \
    sim:=gazebo human:=hunav world:=hospital robot:=jackal \
    tm_robots:=scenario tm_obstacles:=scenario \
    local_planner:=dwb global_planner:=navfn
```

`vglrun -d egl0` makes Gazebo and RViz render on the NVIDIA L4 GPU, not on the CPU. Without it, rendering is painfully slow and GPU lidar may not work.

**Stopping:** Ctrl+C once, then wait. If Gazebo survives (it sometimes does), `pkill -f "gz sim"`. A leftover Gazebo from a previous run was the cause of duplicate-simulator weirdness in the chat. Check with:

```bash
ps -eo pid,etime,args | grep -E 'gz sim|ros2 launch' | grep -v grep
```

## Launch arguments (`arena.launch.py`)

| Arg | Default | Values / notes |
|---|---|---|
| `sim` | `dummy` | **`gazebo`** (only one installed). `isaac` needs the Isaac installer. |
| `human` | `hunav` when `sim:=gazebo` | `hunav`, `dummy` (no pedestrians) |
| `world` | `map_empty` | a dir name under `simulation-setup/worlds/`: `map_empty`, `hospital`, `factory`, `ignc`, `house17`, `generated` |
| `robot` | `jackal` | model name, `jackal,turtlebot`, `jackal[3]` (3 copies), or `demo.yaml` (a file in `arena_bringup/configs/robot_setup/`) |
| `local_planner` | `dwb` | a dir name under `simulation-setup/configs/nav2/controllers/` (see [05](05-policies-and-pytorch.md)) |
| `global_planner` | `navfn` | `navfn`, `smac_2d`, `smac_hybrid`, `smac_state_lattice`, `theta_star` |
| `inter_planner` | `navigate_w_replanning_time` | a dir under `configs/nav2/interplanners/` (behaviour-tree XML) |
| `tm_robots` | `explore` | `scenario`, `random`, `explore`, `guided` |
| `tm_obstacles` | `random` | `scenario`, `random`, `parametrized`, `environment` |
| `tm_modules` | `rviz_ui` | comma list: `staged`, `dynamic_map`, `benchmark`, … (`rviz_ui` and `clear_forbidden_zones` are always added) |
| `headless` | `0` | `0` show all, `1` RViz only (no Gazebo GUI), `2` nothing. Use `2` for batch runs. |
| `log_level` | `warn` | `info`/`debug` when debugging |
| `record_data_dir` | `''` | set a name to turn on `arena_evaluation`'s data recorder for each robot |
| `env_n`, `env_d` | `1`, `50` | parallel environments in one sim, spaced `env_d` m apart (training use) |
| `localization` | `amcl` (**`ground_truth` in our `arena-launch.sh`**) | *Our addition.* `ground_truth` = robot gets its exact Gazebo pose, no AMCL. Jackal only. See [08](08-ground-truth-localization.md). |
| `rviz` | `true` | *Our addition.* `rviz:=false` = Gazebo window without RViz (lighter). |
| `scenario` | `default.json` | *Our addition.* Scenario JSON in `worlds/<world>/scenarios/`, used with `tm_robots/tm_obstacles:=scenario`. See [09](09-hallway-scenarios.md). |

Things you **can't** set from the command line, because they're only in `task_generator.yaml`: random obstacle counts, episode count, timeout (now 600 sim-seconds). The scenario file *can* now be set, with `scenario:=`. See [04](04-configuring-sim-and-robot.md).

**Benchmark runs: use `headless:=2`.** With 20 pedestrians, the Gazebo GUI + RViz overloaded the 8-core VM and made
Nav2 time out ([09 §5](09-hallway-scenarios.md#5-performance-notes)).

## Checking a run from a second terminal

`scripts/check_run.py` prints where the robot **believes** it is (TF), where it **really** is (Gazebo), the error
between them, and Nav2's goal status:

```bash
cd ~/arena_ws && source arena.bash && unset FASTRTPS_DEFAULT_PROFILES_FILE
python ~/Documents/Lyu_Lab/initial-arena-testing/scripts/check_run.py --period 10            # stops at first goal
python ~/Documents/Lyu_Lab/initial-arena-testing/scripts/check_run.py --period 30 --keep-going  # watch many episodes
```

With ground-truth localization expect ~0.1 m of x "error" while moving. That's just the delay of the `gz` CLI call; y
and heading error should be 0.

## Namespaces: where to look

| Thing | Name |
|---|---|
| Task generator | `/task_generator_node` |
| Robot `jackal` | `/task_generator_node/jackal/*` (lidar, odom, cmd_vel, plan, costmaps, all Nav2 nodes) |
| Map | `/task_generator_node/map` |
| Pedestrians | `/task_generator_node/human_states` (`hunav_msgs/Agents`), `/task_generator_node/people` (`people_msgs/People`) |
| Episode events | `/task_generator_node/task_reset` (Int16, the episode counter), `/task_generator_node/finished` |

## `ros2` CLI cheat-sheet

Source first (`cd ~/arena_ws && source arena.bash`), or you'll be on the wrong `ROS_DOMAIN_ID`.

```bash
# What exists?
ros2 node list
ros2 topic list -t                         # -t shows message types
ros2 service list | grep task_generator
ros2 param list /task_generator_node

# Watch data
ros2 topic echo /task_generator_node/jackal/cmd_vel
ros2 topic echo /task_generator_node/jackal/odom --field pose.pose.position
ros2 topic hz  /task_generator_node/jackal/lidar          # should be ~10 Hz of SIM time
ros2 topic info -v /task_generator_node/jackal/cmd_vel    # who publishes / subscribes

# Interrogate a message type
ros2 interface show sensor_msgs/msg/LaserScan
ros2 interface show hunav_msgs/msg/Agent

# Parameters (live!)
ros2 param get /task_generator_node/jackal/controller_server controller_frequency
ros2 param get /task_generator_node/jackal/controller_server FollowPath.max_vel_x
ros2 param set /task_generator_node/jackal/controller_server FollowPath.max_vel_x 0.5   # takes effect immediately for DWB
ros2 param dump /task_generator_node/jackal/controller_server > /tmp/controller.yaml     # see the MERGED config actually in use

# Episodes
ros2 service call /task_generator_node/reset_task std_srvs/srv/Empty     # new episode now

# Nav2 lifecycle health
ros2 lifecycle get /task_generator_node/jackal/controller_server          # want: active

# TF
ros2 run tf2_tools view_frames                                             # writes frames.pdf
ros2 run tf2_ros tf2_echo map jackal/base_link

# Graph view (GUI)
rqt_graph
```

`ros2 param dump` is the most useful debugging trick in this whole stack. Arena builds the Nav2 config by merging five YAML files, and a dump shows you what actually came out.

## Gazebo-side tools

```bash
gz topic -l                                   # Gazebo's own topics (not ROS)
gz topic -e -t /clock                         # sim clock
gz model --list
```

Real-time factor: the Gazebo GUI shows RTF at the bottom right. RTF < 1 means simulated time runs slower than wall time. The hospital was at a tiny RTF with its original 1 ms physics step.

## RViz

The Arena RViz config is generated at launch by `rviz_utils/rviz_config`. Things to look at:
- **Task Generator panel**: Reset Task, obstacle counts (random mode), scenario and world selectors.
- **Costmaps** (local/global), **plan** (green global path), **local_plan**, **lidar**, and **pedestrian markers** with velocity arrows.
- **2D Goal Pose** tool: publishes `/goal_pose`, which a relay forwards to each robot's `goal_pose`.

Save your own RViz layout (File → Save Config As) into this repo if you customise it.

## Logs

- Console of the launch terminal (raise `log_level:=info`).
- `~/.ros/log/<timestamp>-.../launch.log`: the full log of each launch.
- `~/.gz/sim/8/server_console.log`: Gazebo server messages.
