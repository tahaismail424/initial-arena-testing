# 13. Pedestrian reset between episodes (fixed 2026-10-11)

Until this fix, pedestrians vanished after the first episode (see [09 §5](09-hallway-scenarios.md#5-performance-notes--running-faster-than-real-time)). Multi-episode runs now work:
every episode re-spawns all 20 scenario pedestrians at their scenario start poses and they walk again.

## Root cause

What happens at a reset (`TaskFactory._reset_task` → `EnvironmentManager.respawn`):

1. `BaseHumanSimulator.unuse_obstacles()` → `HunavHumanSimulator._remove_obstacles_impl()`
   (`arena-rosnav/task_generator/task_generator/simulators/human/hunav/hunav.py:526`). That calls the plugin's
   `delete_actors` service. `HuNavSystemPluginIGN::deleteActorsCallback` (`hunav_gz_plugin/src/HuNavSystemPlugin.cpp:1091`)
   **removes every pedestrian actor from Gazebo**, clears its `pedestrians_` and sets `agentsInitialized_ = false`.
   Then Arena clears its agent containers and calls HuNav's `reset_agents`.
2. `BaseHumanSimulator.spawn_dynamic_obstacles()` (`.../simulators/human/__init__.py:79-104`) looks each pedestrian up
   in `_known_obstacles` by name. Every name is still known from episode 1, so it only calls `move_entity()`
   (Gazebo: `Unable to update the pose for entity id:[0], name[walker_...]`, because the entity is gone). It marks it
   `INUSE`, so the `if known.layer == UNUSED: spawn_entity(...)` branch **never runs**. The actors are never re-spawned.
3. HuNav itself is re-registered fine (Arena's `compute_agents` call in `_spawn_dynamic_obstacles_impl`). But the
   plugin keeps calling `get_agents` (`initializeAgents` returns early on the first agent missing from the ECM, so
   `agentsInitialized_` stays false: the `GET AGENTS CALLBACK` ~30×/s we saw). Its own `compute_agents` calls carry
   an empty agent list. `hunav_agent_manager` publishes `human_states` straight from that request, hence
   **`humans 0`** in episode 2+.

So `delete_actors` was not missing. "Delete actors service currently not available" only shows up at the very first
reset (startup), before the first pedestrian has spawned the plugin. Nothing to delete then, so it's harmless. From
episode 2 on the delete **worked**, and the base class's reuse logic didn't know that.

Secondary race (plugin): the callback deferred the removal to the *next* `PreUpdate`. Arena re-spawns the same names
right after the call returns. If Gazebo's `UserCommands` processed a create before that removal, the create would be
rejected (name still taken) and that pedestrian would be missing for the whole episode. Not observed, but the timing
allows it on a faster or less loaded machine.

## Fix

| Repo / file | Change |
|---|---|
| arena-rosnav `task_generator/.../simulators/human/hunav/hunav.py` `_remove_obstacles_impl` (commit `9a1c17ea`, branch `lyu-lab`) | When `delete_actors` succeeds, `self._known_obstacles.forget(name)` for every pedestrian. The following `spawn_dynamic_obstacles()` then spawns fresh actors (SDF pose = scenario start). It only forgets when the delete succeeded, so we never spawn duplicates. |
| hunav_gz_plugin `src/HuNavSystemPlugin.cpp`, `include/.../HuNavSystemPlugin.h` (commit `90e91f6`, branch `lyu-lab`, C++ → `colcon build --symlink-install --packages-select hunav_gz_plugin`) | `deleteActorsCallback` calls `RequestRemoveEntity` immediately through a stored ECM pointer (`ecm_`, set in `Configure`/`PreUpdate`). The callback only ever runs while those spin the ROS node on the sim thread, so this is safe. The removal lands at the end of the current iteration, before Arena's re-spawn can be processed. It also removes actors named in the last `get_agents` reply that weren't initialised yet (`agentNames_`), so no leftover blocks a re-spawn. The old deferred path stays as a fallback if `ecm_` is null. |

Exported in [`../patches/`](../patches/) (`arena-rosnav.patch`, `hunav_gz_plugin.patch`).

## Verification (school_hallway, hallway_tame.json, dsrnn, headless:=2, ground_truth)

Run 2 (both patches). Monitor script (`scripts/reset_monitor.py`) (per 10 sim s: agents in `/task_generator_node/human_states`,
how many moved >0.2 m in the last 10 s; robot pose 1 s before each reset). Actor entities counted from the live ECM
(`gz service -s /world/default/state`, by Name component, `scripts/count_actors.sh`). Note: `gz model --list` does **not**
list actors.

| Episode | agents in human_states | moving (>0.2 m / 10 s) | start poses | robot at end | actor entities |
|---|---|---|---|---|---|
| 1 | 20 | 20/20 every sample | first msg 0.00 m from scenario starts | episode ended after 73 s sim (timeout 600 s) → goal | 20 |
| 2 | 20 | 20/20 | 0.00 m | 0.10 m from goal (59, 6) | 20 (20 unique names) |
| 3 | 20 | 20/20 | 0.00 m | 0.32 m from goal | 20 |
| 4 | 20 | 20/20 | 0.00 m | 0.26 m from goal | 20 |
| 5 | 20 | 20/20 (stopped after) | 0.00 m | n/a | 20 |

* A probe right after a reset: `runner_0` reported at exactly its scenario start (38.47, 1.75), then walking at
  ~0.09 m per 33 ms step (≈2.7 m/s, its target is 2.65).
* No `Unable to update the pose`, no `already exists`, no errors at any reset (episode 2+). Gazebo models stay at 4
  (ground_plane, walls_0, jackal, hunav_plugin) and actors at 20.
* Run 1 (Arena patch only, before the plugin patch): 5 episodes, same picture (20 agents, 20/20 moving, 20 actors).

## Remaining caveats

* **First-tick jump after (re)spawn (pre-existing, also in episode 1):** HuNav stamps its requests with *wall* time
  (plugin `rosnode_->get_clock()->now()`, Arena's node too), and `hunav_agent_manager` integrates over the stamp
  difference. During a reset Gazebo is blocked in the plugin's `get_agents` call (Arena answers it only after the
  reset callback finishes, same callback group), so the first 1–2 ticks after a reset integrate over ~0.4 s of wall time.
  Pedestrians jump ~1 m along their path right at the start. Harmless for benchmarks, but worth knowing. Fixing it means
  stamping with sim time in the plugin.
* Each reset now spawns 20 actors through `/world/default/create` (~1 s). Re-positioning existing actors would be
  faster. It would need the plugin to re-init in place without deleting, plus skipping Arena's `move_entity` on actors
  (it would also overwrite their height).
* The first reset still logs "Delete actors service currently not available" (plugin not spawned yet). Harmless.
* `GazeboSimulator.pause_simulation()` is a no-op upstream (`return True  # TODO`), so the sim keeps running during
  resets. The fix doesn't depend on pausing.
