#!/usr/bin/env bash
# Launch Arena (Gazebo + HuNav + Jackal + Nav2) on a Lyu Lab VM.
# Run from a terminal in the VM's graphical desktop (or with DISPLAY=:1). Ctrl+C stops everything.
#
# Usage: arena-launch.sh [world] [task_mode] [extra ros2 launch args...]
#   arena-launch.sh school_hallway scenario scenario:=hallway_tame.json rviz:=false   # crowd, Gazebo window only
#   arena-launch.sh school_hallway scenario scenario:=hallway_tame.json headless:=2   # crowd, no windows
#   arena-launch.sh school_hallway scenario scenario:=hallway_tame.json local_planner:=crowdnav
#   arena-launch.sh map_empty explore                                                 # random-goal demo
#   arena-launch.sh school_hallway scenario localization:=amcl                       # estimated pose
#
# task_mode "scenario" uses tm_robots:=scenario tm_obstacles:=scenario;
# anything else is passed as tm_robots with tm_obstacles:=random.
# Defaults: DWB + NavFn, localization:=ground_truth (exact Gazebo pose, no AMCL). Later args override.
#
# Rendering (Gazebo needs OpenGL even headless: the Jackal's lidar is a gpu_lidar):
#   * NVIDIA GPU present  -> run under VirtualGL on the GPU (`vglrun -d egl0`)        [arena-dev, L4]
#   * no GPU              -> Mesa software rendering (llvmpipe) on the CPU            [arena-dev-cpu]
#   Force one with ARENA_RENDER=gpu or ARENA_RENDER=cpu.

WORLD="${1:-hospital}"
MODE="${2:-scenario}"
shift 2 2>/dev/null || shift $#

if [ "$MODE" = scenario ]; then
  TM=(tm_robots:=scenario tm_obstacles:=scenario)
else
  TM=(tm_robots:="$MODE" tm_obstacles:=random)
fi

if pgrep -f "gz sim" >/dev/null; then
  echo "A Gazebo instance is already running. Stop it first (pkill -f 'gz sim')." >&2
  exit 1
fi

RENDER="${ARENA_RENDER:-auto}"
if [ "$RENDER" = auto ]; then
  if nvidia-smi >/dev/null 2>&1 && [ -x /opt/VirtualGL/bin/vglrun ]; then RENDER=gpu; else RENDER=cpu; fi
fi

export PATH="$HOME/.local/bin:$PATH"
cd "$HOME/arena_ws" || exit 1
source arena.bash
# arena.bash points this at ~/.ros/fastdds.xml, which does not exist on these VMs.
unset FASTRTPS_DEFAULT_PROFILES_FILE

ARGS=(sim:=gazebo human:=hunav world:="$WORLD" robot:=jackal
      "${TM[@]}" local_planner:=dwb global_planner:=navfn localization:=ground_truth "$@")

if [ "$RENDER" = gpu ]; then
  echo "[arena-launch] rendering on the NVIDIA GPU (VirtualGL)"
  exec /opt/VirtualGL/bin/vglrun -d egl0 ros2 launch arena_bringup arena.launch.py "${ARGS[@]}"
else
  echo "[arena-launch] no GPU: rendering on the CPU with Mesa llvmpipe (slower; prefer rviz:=false or headless:=2)"
  export LIBGL_ALWAYS_SOFTWARE=1   # Mesa software OpenGL for Gazebo's GUI and its gpu_lidar sensor
  export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/50_mesa.json  # EGL -> Mesa, not NVIDIA
  exec ros2 launch arena_bringup arena.launch.py "${ARGS[@]}"
fi
