# 9. Hallway crowd scenarios: spec → JSON → HuNav

**TL;DR**

```bash
cd ~/Documents/Lyu_Lab/initial-arena-testing
uv run scripts/generate_hallway.py -n school_hallway --link           # world (60 x 12 m, 3 side corridors)
uv run scripts/generate_scenario.py scenarios/hallway_tame.yaml --link  # crowd -> worlds/school_hallway/scenarios/hallway_tame.json
# (new files only) cd ~/arena_ws && source arena.bash && colcon build --symlink-install --packages-select arena_simulation_setup
bash scripts/arena-launch.sh school_hallway scenario scenario:=hallway_tame.json              # watch it
bash scripts/arena-launch.sh school_hallway scenario scenario:=hallway_tame.json headless:=2  # benchmark it
```

Check any run from a second terminal with `scripts/check_crowd.py` (pedestrians) and `scripts/check_run.py` (robot).

---

## 1. The pipeline

```
scenarios/hallway_tame.yaml  ──►  scripts/generate_scenario.py  ──►  worlds/school_hallway/scenarios/hallway_tame.json
   (agent CLASSES: counts,           (+ world_params.yaml for the       (one entry per pedestrian: start, route,
    speeds, route patterns,           layout; seed -> reproducible)      speed, behaviour)   + preview_hallway_tame.png
    behaviours, seed)                                                              │
                                                                                   ▼  launch arg scenario:=hallway_tame.json
                              Arena task_generator (tm_obstacles:=scenario) ──► HuNav agent manager + Gazebo actors
```

* **Spec (YAML, you edit):** *what kind* of crowd. Classes of people, how many, how fast, how they move.
* **Scenario JSON (generated, don't hand-edit):** the concrete crowd: every pedestrian's start, loop and parameters.
  Same spec + same seed gives the same JSON, which is what makes benchmarks repeatable.
* **Launch arg `scenario:=<file>`** (our addition, same recipe as `localization`, see [08 §7](08-ground-truth-localization.md#7-the-general-recipe-reuse-this)).
  It overrides `task.scenario.file` from `task_generator.yaml`. It's applied *after* the YAML in the node's parameter
  list, so it wins.

## 2. The current spec: `scenarios/hallway_tame.yaml`

| Class | # | Speed (m/s) | Route pattern | Behaviour |
|---|---|---|---|---|
| `walker_with_robot` | 4 | 1.1–1.4 | `lane` heading +x (lower half: people keep right) | REGULAR |
| `walker_toward_robot` | 4 | 1.1–1.4 | `lane` heading −x (upper half), i.e. oncoming | REGULAR |
| `distracted` | 2 | 0.6–0.8 | `zigzag` across the hallway every 8 m | IMPASSIVE, social force 1.0 (reacts late) |
| `runner` | 2 | 2.5–3.0 | `lane`, one each direction | REGULAR, radius 0.4 |
| `friend_group` | 2 groups (3 + 2) | 0.9–1.1 | `lane`, members on parallel lanes 0.8 m apart, shared `group_id` | REGULAR |
| `from_corridor` | 3 | 1.0–1.3 | `door_loop`: out of a side corridor, along the hallway, into another corridor, back | REGULAR |

**Route patterns** (all are *closed loops*, because HuNav's `cyclic_goals` walks from the last waypoint straight back to
the first):

* `lane`: far end on your side → cross over → back on the other side → cross over. Turnarounds are 5–8 m from each
  end wall, randomised per agent.
* `zigzag`: weave between the two halves every `zigzag_step` m, end to end and back.
* `door_loop`: door A → your side of the hallway → door B → the other side → door A. Each corridor walker gets its
  own doors.

To make a new variant, copy the YAML, change counts, speeds, classes or the seed, and run the generator with the new
file. Output is named after the spec's `name:`.

## 3. What each JSON field does (read from the code)

Arena keeps the whole JSON entry as `extra`. Its HuNav adapter
(`task_generator/.../simulators/human/hunav/__init__.py` + `hunav.py`) turns it into a `hunav_msgs/Agent`:

| JSON field | HuNav meaning | Default if missing (`hunav_agents/default.yaml`, agent1) |
|---|---|---|
| `pos` | start position `[x, y, yaw]` | — |
| `waypoints` | goals, in order | — |
| `desired_velocity` | preferred walking speed (m/s) | 0.3 (a slow shuffle) |
| `radius` | body radius for the social forces (m) | 0.4 |
| `goal_radius` | "reached this waypoint" distance (m) | 0.3 |
| `cyclic_goals` | loop the waypoints forever | true |
| `group_id` | ≥ 0 = walks as a group with the same id (group forces) | −1 (alone) |
| `type` | `hunav_msgs/Agent.PERSON` = **1** (it's a `uint8`; old Arena scenarios wrongly use `"adult"`) | 1 |
| `behavior.type` | 1 REGULAR (robot = another person), 2 IMPASSIVE (robot = obstacle), 3 SURPRISED, 4 SCARED, 5 CURIOUS, 6 THREATENING (3–6 react when the robot comes close) | 1 |
| `behavior.social_force_factor` | how strongly they avoid *other people and the robot* | **0** ⚠️ |
| `behavior.obstacle_force_factor` | how strongly they avoid walls | 65 |
| `behavior.goal_force_factor` | pull toward the next waypoint | **ignored: Arena hard-codes 20** |
| `behavior.vel`, `dist`, `duration`, `once`, `configuration`, `other_force_factor` | parameters of the reactive behaviours (3–6) | agent1's |
| `class` | *ours*, ignored by Arena; used by `check_crowd.py` and the preview | — |

## 4. Pitfalls we hit (and the fix in the generator)

| Symptom | Cause | Fix |
|---|---|---|
| Pedestrians would walk *through* each other | `social_force_factor` defaults to 0 | the spec sets 5.0 for everyone (1.0 for distracted) |
| 10 pedestrians jammed against the far wall | every lane turned around at exactly x = L−2. Goal force (20) ≫ social force (5), so people aiming at the same point pile up | turnarounds randomised to 5–8 m from each end, plus ±0.4 m jitter on every waypoint |
| Friends overlapping (0.2 m apart) | group members shared one route, so they converged on the same waypoints | each member gets a parallel lane 0.8 m from the next |
| Corridor walkers frozen 1.3 m short of their door | waypoint 1 m from a dead end: the walls on three sides push back harder than the goal pulls | door waypoint halfway into the corridor |
| Robot start blocked by a pedestrian | turnarounds at x = 2 were right on the robot's start (1, 6) | turnarounds ≥ 5 m from the ends, and no pedestrian starts within 4 m of the robot |
| Pedestrians frozen on an **invisible line at y ≈ 6** near both hallway ends, and inside the side corridors | **HuNav plugin bug:** it turns every Gazebo box into an axis-aligned box of its *unrotated* size. Arena's walls are thin boxes rotated to their segment, so every north–south wall (end caps, corridor sides) became a phantom east–west wall. HuNav's `closest_obs` for the stuck pedestrians pointed at (x, 6.1). | Wall geometry renamed to `arena_wall_visual` / `arena_wall_collision` (`gazebo_simulator.py`) and added to the plugin's `<ignore_models>` (`hunav.py`). HuNav still avoids walls through the correct `get_walls` segments from `walls.yaml`. ⚠️ The same bug would affect any *rotated* static obstacle (a rotated shelf, say). |
| Robot's goal **aborted** mid-run, never retried | `bt_navigator` waits only 20 ms for the planner to accept a replan. The machine was overloaded (GUI + RViz + 20 animated actors: load 14 on 8 cores), so it timed out, and the default BT has no recovery | `default_server_timeout: 200` in `configs/nav2/nav2.yaml` (same for every policy), and benchmark runs use `headless:=2` |

### Known limitation: pedestrians don't *socially* see the robot
Arena's fork of the HuNav Gazebo plugin has robot tracking **commented out** (`HuNavSystemPlugin.cpp`, `robot_name`
around lines 110 and 265). HuNav's internal robot therefore stays at (0, 0) (check
`/task_generator_node/robot_states`). Consequences:

* Pedestrians still **physically avoid** the robot: its Gazebo collision boxes are in their obstacle list (obstacle
  force), much like `IMPASSIVE` people.
* They do **not** treat it as a person (social force), and behaviours 3–6 (`SURPRISED`, `SCARED`, `CURIOUS`,
  `THREATENING`) **never trigger**, because HuNav thinks the robot is far away.

This is fine for a first benchmark: every policy faces the same crowd. Restoring it means re-enabling that code and
rebuilding `hunav_gz_plugin` (C++). It's a good future task if reactive crowds matter for your research question.

## 5. Performance notes & running faster than real time

**Display options** (from least to most load):

| Want | Args |
|---|---|
| nothing (benchmarks) | `headless:=2`, or `arena-launch.sh --headless ...` |
| Gazebo window only, no RViz | `rviz:=false` (*our addition*: also stops the RViz-only pedestrian marker publisher, ~0.6 core) |
| RViz only | `headless:=1` |
| everything | (default) |

**Measured cost of the Gazebo window** (2026-10-11, CPU-only VM, 8 cores, `hallway_tame.json`, `local_planner:=dsrnn`,
40 s window inside episode 1, `top` + `/clock` vs wall clock):

| Mode | Whole machine busy | `gz sim` server | `gz sim` GUI | Real-time factor |
|---|---|---|---|---|
| `rviz:=false` (Gazebo window) | 5.7 cores | 2.2 cores | **1.0 core** (+ ~0.15 for the VNC X server) | 1.00 |
| `headless:=2` / `--headless` | 5.3 cores | 2.3 cores | – | 0.99 |

What this means:
* **Headless saves about one core** (the GUI process renders the 3-D scene with Mesa llvmpipe on the CPU).
  The real-time factor does not change, because Gazebo **paces itself to 1×** and 8 cores are enough for both. The
  saving matters when you raise `real_time_factor` (below) or run two sims side by side.
* **Rendering can't be switched off completely.** Even with `-s` (server only, what `headless` does), Gazebo renders
  the Jackal's `gpu_lidar` with OpenGL, and the costmaps need that scan. That's part of the server's ~2.2 cores, together
  with physics and the HuNav plugin.
* **Cheap trims that are left** (not done, documented for later): the ROS↔Gazebo bridge also converts every scan into a
  `lidar/points` `PointCloud2` that nothing in our Nav2 setup reads (`simulation-setup/entities/robots/jackal/mappings.yaml`);
  `parameter_bridge` uses only ~0.06 core, so it's not worth patching. The rest is Nav2 (bt_navigator, planner,
  controller, smoother, behaviors: ~0.2 core each) and the task generator (~0.4 core).
* ⚠️ **Orphaned planner processes were the biggest waste we found.** nav2py starts the policy as
  `nav2py_run` → `<venv>/bin/python -m nav2py_<name>`, and the python child survived `ros2 launch` shutdown. Five old
  DS-RNN servers (up to 8 h old) were each spinning at ~80% CPU, i.e. **~4 cores**, which pulled the RTF down to 0.67.
  `arena-stop.sh` now kills them too. If a run looks slow, check `pgrep -af "python -m nav2py_"` first.

**Everything already runs on simulated time.** Every node uses `/clock`, and episode timeouts are in sim seconds.
Speed is decided by Gazebo alone:

* Gazebo Harmonic **paces itself to 1× by default** (`real_time_factor`). The world's `real_time_update_rate: 0` is a
  Gazebo *Classic* setting and does nothing here. Change it at runtime (any time during a run):
  ```bash
  gz service -s /world/default/set_physics --reqtype gz.msgs.Physics --reptype gz.msgs.Boolean \
    --timeout 5000 --req 'max_step_size: 0.0333, real_time_factor: 3.0'
  ```
* Measured on this VM (headless, 2026-09-28):

  | Setup | Max real-time factor | Notes |
  |---|---|---|
  | hallway, **no pedestrians** | 10× reachable | but **Nav2 breaks at 10×** ("Failed to make progress", all episodes time out). **3× works**: normal episodes. |
  | hallway, **20 pedestrians** | **~1.4×** | Gazebo's single main thread is pinned at 100% by the HuNav plugin's per-step work (every pedestrian × every entity, plus a blocking ROS service call each step) |
  | same, pedestrians at 15 Hz (our plugin patch) | ~3.3× | ⚠️ pedestrians then moved at ~40% of their desired speed, so **not used**. Worth investigating later (probably the plugin's per-update distance limit). |

* **Caveat for learned policies at >1×:** Nav2 runs its loops on sim time but *computes* in real time. A slow policy
  (e.g. a big network on CPU) gets fewer decisions per simulated second when the sim runs faster. Always check the logs
  for `missed its desired rate`. If it appears, the speed-up is changing the result.
* Practical plan: run crowd benchmarks headless at ~1–1.4×, and **run several simulations in parallel overnight**.
  Each needs its own `ROS_DOMAIN_ID` and `GZ_PARTITION`; not set up yet. One sim uses ~1 core for Gazebo plus ~2–3 for
  Nav2 and Arena, so 2 in parallel is realistic on 8 cores.

### ✅ Fixed (2026-10-11): pedestrians now survive an episode reset
Before the fix, pedestrians vanished after the first episode (`human_states` empty, `GET AGENTS CALLBACK` loop). Cause: HuNav's plugin deleted the actors at each reset, but Arena still knew their names and only tried to *move* them instead of re-spawning them. Fixed in arena-rosnav and hunav_gz_plugin; verified over 5 consecutive episodes (20/20 pedestrians, at their start poses, walking). Full write-up: [13](13-pedestrian-reset-fix.md).

## 6. Validation results (2026-09-28)

`school_hallway` (60 × 12 m), `hallway_tame.json` (seed 0, 20 pedestrians), Jackal + NavFn + DWB (unchanged),
`localization:=ground_truth`, `headless:=2`, all fixes above applied:

| Check | Result |
|---|---|
| Pedestrians moving (every 60 s sample over the whole episode) | **20/20, stuck: none** |
| Closest pedestrian pair | 0.71–1.29 m (nobody overlapping) |
| Speeds (sim time) | every class near its target: walkers ~1.0–1.2 of 1.25–1.3 m/s, runners 1.3–1.8 of 2.65 (their loops include turns), distracted ~0.55 of 0.62 |
| Robot–pedestrian encounters | oncoming and overtaking walkers passed 0.88–1.25 m from the robot |
| Episode outcome | **goal reached in ~296 s** (empty hallway: ~245 s), then a clean reset into episode 2 |
| Nav2 planner time-outs | 0 (the previous GUI run had one that aborted the goal) |
| Real-time factor (headless) | ~0.9 |

Before the phantom-wall fix, the same scenario also reached the goal (~298 s), but 2–4 pedestrians were permanently
stuck on the invisible walls, which made the crowd effectively thinner.

**Open quirk:** HuNav still reports a small obstacle region ~1.5–2 m *behind* the robot that moves with it. It comes
from the robot's own Gazebo geometry, probably the same axis-aligned-box issue on a rotated robot part. Effect: people
keep a little extra distance behind the robot. It's tied to the robot, not the controller, so it's identical for
every policy. Worth a look if people-behind-robot interactions become important.
