#!/usr/bin/env bash
# Launch Arena (Gazebo + HuNav + Jackal + Nav2) on the Lyu Lab VM.
# Run from a terminal in the VM's graphical desktop. Ctrl+C stops everything.
#
# Usage: arena-launch.sh [world] [task_mode] [extra ros2 launch args...]
#   arena-launch.sh                                  # hospital, scenario mode
#   arena-launch.sh map_empty explore                # random-goal demo in the empty world
#   arena-launch.sh hospital scenario local_planner:=mppi log_level:=info
#   arena-launch.sh school_hallway scenario localization:=amcl   # estimate pose instead of ground truth
#
# task_mode "scenario" uses tm_robots:=scenario tm_obstacles:=scenario;
# anything else is passed as tm_robots with tm_obstacles:=random.
# Localization defaults to ground_truth (robot is given its exact Gazebo pose, no AMCL) so that
# benchmark results reflect the navigation policy, not localization errors. Later args override.

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

export PATH="$HOME/.local/bin:$PATH"
cd "$HOME/arena_ws" || exit 1
source arena.bash
# arena.bash points this at ~/.ros/fastdds.xml, which does not exist on this VM.
unset FASTRTPS_DEFAULT_PROFILES_FILE

exec /opt/VirtualGL/bin/vglrun -d egl0 ros2 launch arena_bringup arena.launch.py \
  sim:=gazebo human:=hunav world:="$WORLD" robot:=jackal \
  "${TM[@]}" local_planner:=dwb global_planner:=navfn localization:=ground_truth "$@"
