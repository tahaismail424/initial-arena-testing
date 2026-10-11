#!/bin/bash
# Count pedestrian actor entities (by Name component) in the live Gazebo ECM.
source ~/arena_ws/arena.bash >/dev/null 2>&1
out=$(timeout 40 gz service -s /world/default/state --reqtype gz.msgs.Empty --reptype gz.msgs.SerializedStepMap --timeout 30000 --req '' 2>/dev/null)
names=$(echo "$out" | grep -E '^\s*component: "(walker|runner|distracted|friend|from_corridor)[a-z_]*_[0-9]+"$' | sed 's/.*"\(.*\)"/\1/')
echo "actor_entities=$(echo "$names" | grep -c .) unique_names=$(echo "$names" | sort -u | grep -c .) models=$(echo "$out" | grep -cE '^\s*component: "(ground_plane|walls_0|jackal|hunav_plugin)"$')"
