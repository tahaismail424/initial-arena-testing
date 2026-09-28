# 2. Workspace layout & the edit → build → run loop

## Where things live

```
~/arena_ws/                         ← colcon workspace (ARENA_WS_DIR)
├── arena.bash → src/arena/arena-rosnav/tools/source.bash   (source this!)
├── colcon_build → .../tools/colcon_build                    (Arena's build wrapper)
├── build/  install/  log/          ← colcon outputs (484 packages)
└── src/
    ├── ros2/        ROS 2 Humble core, built FROM SOURCE (not /opt/ros)
    ├── deps/        nav2 (voshch fork of navigation2), BehaviorTree.CPP, hunav_sim,
    │                slam_toolbox, robots/{jackal,turtlebot4,irobot_create}, msgs…
    ├── gazebo/      ros_gz (bridge), sdformat_urdf, hunav_gz_plugin, turtlebot4_simulator
    ├── tools/       gz-usd
    ├── planners/    ← does NOT exist yet; created when you install planners (see 05)
    └── arena/
        ├── arena-rosnav/          ★ the orchestrator (git: arena-rosnav/arena-rosnav @ humble)
        │   ├── arena_bringup/         launch files + configs/ (task_generator.yaml, hunav_agents/, robot_setup/, training/)
        │   ├── task_generator/        ★ the brain: node.py, tasks/, manager/, simulators/
        │   ├── utils/                 arena_rclpy_mixins, rviz_utils (RViz panel), rl_utils (legacy), msgs
        │   ├── training/              legacy ROS1 SB3 training script (not functional on humble)
        │   ├── installers/            1_gazebo.sh, 2_isaac.sh, 3_planners.sh, x_training.sh
        │   ├── .repos/                arena / gazebo / isaac / planners .repos (vcstool manifests)
        │   ├── .installed             which optional installers ran (currently: gazebo.sh)
        │   ├── pyproject.toml         poetry env → .venv (Python for all Arena nodes)
        │   └── .venv/
        ├── simulation-setup/      ★ assets + robot/Nav2 configs (git: voshch/arena-simulation-setup @ humble-fix)
        │   ├── worlds/<world>/        map/, worlds/<world>.world, scenarios/*.json
        │   ├── entities/robots/<robot>/   URDF, model_params.yaml, mappings.yaml, control.yaml
        │   ├── entities/obstacles/{static,dynamic}/   spawnable obstacle + pedestrian models
        │   ├── configs/nav2/          nav2.yaml template + controllers/ planners/ interplanners/
        │   ├── configs/environment/   (tm_obstacles:=environment presets)
        │   ├── gazebo_models/         ~380 Gazebo model dirs (hospital furniture, etc.)
        │   └── launch/                robot.launch.py, nav2.launch.py
        ├── evaluation/            arena_evaluation: data recorder + metrics + plots
        └── tools/                 arena_tools (map generation utilities etc.)
```

The **★** directories are where you'll spend 95% of your time.

## Sourcing the environment

Every new terminal needs:

```bash
cd ~/arena_ws && source arena.bash
```

What `arena.bash` does:
- Activates the **poetry venv** (`arena-rosnav/.venv`) and puts its site-packages on `PYTHONPATH`.
- Sources `/opt/ros/humble/setup.bash` if present, then `install/local_setup.bash`. The workspace overlay wins.
- Sets `ROS_DOMAIN_ID=1`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, and `GZ_VERSION=harmonic`.
- Sets `FASTRTPS_DEFAULT_PROFILES_FILE=~/.ros/fastdds.xml`. **That file does not exist on this VM**, so our launchers `unset` it.
- Defines a helper: `r2st <service>` calls a service with its default request.

> ⚠️ `ROS_DOMAIN_ID=1`: a terminal that did **not** source `arena.bash` is on domain 0 and won't see any Arena topics. When `ros2 topic list` looks empty, this is usually why.
>
> It must be run from `~/arena_ws`, because it uses `$(pwd)` as `ARENA_WS_DIR`.

## `--symlink-install`: which edits need a rebuild?

Arena builds with `colcon build --symlink-install`. I verified that the installed files for Arena's own packages are symlinks back into `src/` (`install/... → build/... → src/...`). In practice:

| You edited… | Rebuild needed? | Then… |
|---|---|---|
| An **existing** YAML/JSON/launch/world/URDF file in `arena-rosnav` or `simulation-setup` | **No** | Restart the launch (Ctrl+C, relaunch) |
| An **existing** Python file in `task_generator`, `arena_bringup`, `rviz_utils`, … | **No** | Restart the launch |
| **Added a new file or directory**, e.g. a new controller config dir, scenario, or robot | **Yes**, for that package | `colcon build --symlink-install --packages-select arena_simulation_setup` (or whichever package), then restart |
| Changed a `setup.py`, `package.xml`, `CMakeLists.txt`, `.msg`/`.srv` | **Yes** | Build that package |
| C++ source (Nav2 plugins, `hunav_gz_plugin`, `rviz_utils` panel) | **Yes** | Build that package |

> Why new files need a rebuild: Arena resolves every asset through `get_package_share_directory(...)`, meaning `install/<pkg>/share/<pkg>/`. colcon creates one symlink *per file* there at build time. A new file, even a new scenario JSON in an existing `scenarios/` folder, stays invisible until you build that package again.

## Building

Preferred for day-to-day: build only what you touched.

```bash
cd ~/arena_ws && source arena.bash
colcon build --symlink-install --packages-select arena_simulation_setup task_generator \
  --cmake-args -DBUILD_TESTING=OFF
source install/local_setup.bash     # pick up new packages/files
```

Arena's wrapper (`. colcon_build`) builds **everything**, but skips any package whose `install/` dir is newer than its `src/`. That skip logic is what bit us during install: packages that had *failed* were treated as done. Use `SKIP_OLD=0 . colcon_build` to force a full pass. Things to remember:
- It passes `--continue-on-error`, so read the summary at the end.
- `nav2_system_tests` is disabled with a `COLCON_IGNORE` file, because it needs Gazebo Classic, which conflicts with Harmonic.
- Building `turtlebot4_ignition_gui_plugins` needed `export CPLUS_INCLUDE_PATH=/usr/include/ignition/cmake2:$CPLUS_INCLUDE_PATH`.

When a build fails: `cat ~/arena_ws/log/latest_build/<package>/stderr.log`.

## Updating Arena (careful)

`ros2 run arena_bringup pull` does `git pull --autostash` on arena-rosnav, then `vcs import` of `arena.repos` plus one `.repos` file per entry in `.installed`, then `rosdep` and poetry.

⚠️ **We have uncommitted local patches** in `arena-rosnav` and `simulation-setup` (see [06](06-local-patches-and-gotchas.md)). Before pulling, save them:

```bash
cd ~/arena_ws/src/arena/arena-rosnav && git diff > ~/Documents/Lyu_Lab/initial-arena-testing/patches/arena-rosnav.patch
cd ~/arena_ws/src/arena/simulation-setup && git diff > ~/Documents/Lyu_Lab/initial-arena-testing/patches/simulation-setup.patch
```

Better long-term: fork these two repos and commit your changes on a branch.

## Two Pythons: don't mix them up

| Python | Path | Used by |
|---|---|---|
| Arena poetry venv (3.10) | `~/arena_ws/src/arena/arena-rosnav/.venv/bin/python` | All Arena nodes, `ros2` CLI after sourcing |
| System python3 | `/usr/bin/python3` | Nothing Arena-related. Don't `pip install` here for Arena. |
| Per-planner `uv` venvs | created under `build/<planner>/` when learned planners are built | That planner's Python process only (e.g. DRL-VO pins Python 3.8 + torch 1.7.1) |

Add Python deps for Arena nodes with `cd ~/arena_ws/src/arena/arena-rosnav && poetry add <pkg>`, or with `poetry run pip install` for quick experiments. PyTorch is **not** installed in the Arena venv. It only appears if you install the `training` group (`installers/x_training.sh`) or a learned planner.
