#!/usr/bin/env bash
# Stop a running Arena simulation cleanly, whoever started it.
#   bash scripts/arena-stop.sh
#
# 1. Ctrl+C equivalent (SIGINT) to `ros2 launch`, so it can shut its nodes down in order.
# 2. Wait up to 20 s for it to exit.
# 3. Force-kill anything Arena-related that is still alive (Gazebo often lingers).
# Patterns use the [x]yz trick so pgrep/pkill never match this script's own command line.

if pgrep -f "[r]os2 launch arena_bringup" >/dev/null; then
  echo "stopping ros2 launch (SIGINT)..."
  pkill -INT -f "[r]os2 launch arena_bringup"
  for _ in $(seq 1 20); do
    pgrep -f "[r]os2 launch arena_bringup" >/dev/null || break
    sleep 1
  done
fi

LEFTOVERS='[g]z sim|[t]ask_generator_node|[h]unav_agent_manager|[c]ontroller_server|[p]lanner_server|[b]t_navigator|[p]arameter_bridge|[r]viz2|[s]tatic_transform_publisher|[n]av2py_run'
if pgrep -f "$LEFTOVERS" >/dev/null; then
  echo "force-killing leftovers:"
  pgrep -af "$LEFTOVERS" | cut -c1-100
  pkill -9 -f "$LEFTOVERS"
  sleep 1
fi

if pgrep -f "[g]z sim|[r]os2 launch arena_bringup" >/dev/null; then
  echo "WARNING: something is still running:"; pgrep -af "[g]z sim|[r]os2 launch arena_bringup"
else
  echo "all stopped."
fi
