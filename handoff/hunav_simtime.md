# Handoff: make HuNav use simulation time (started 2026-10-12)

Problem (docs/14 §12, campus_verification/summary_table.md): HuNav integrates pedestrian motion over WALL-clock
time, so pedestrian speed in sim time = desired / RTF (plaza/gauntlet RTF ~0.7 -> 1.2-1.5x too fast), and resets
throw pedestrians 4-22 m (the plugin is blocked during a reset, so the first update covers a long wall-time gap).

Plan:
1. Find where time stamps come from: hunav_gz_plugin (src/HuNavSystemPlugin.cpp, rosnode_->get_clock()->now()),
   hunav_agent_manager (bt_node.cpp / agent_manager.cpp: how dt is computed), Arena's hunav.py (task generator).
2. Fix: stamp the plugin's requests with Gazebo SIM time (UpdateInfo simTime) or give its node use_sim_time; make
   sure the agent manager computes dt from those stamps (or uses sim time); clamp dt after resets.
3. Verify: hallway_tame or plaza_dense at reduced RTF -> pedestrian speed/desired ~1.0 (campus_verification/verify.py
   speed column), reset jump small (campus_verification/reset_jump.py).
4. Commit on lyu-lab branches (hunav_gz_plugin / hunav_sim / arena-rosnav), docs/06 + docs/14 §12, patches.

Status: see the bottom of this file.

## Status log
- (start) investigating
- plugin stamps sim time (hunav_gz_plugin src/HuNavSystemPlugin.cpp, uncommitted on lyu-lab) + bt_node.cpp dt clamp [0, 0.2 s] (hunav_sim new branch lyu-lab, uncommitted); both built. NEXT: verify live (plaza_dense_s0 + hallway_tame), commit, docs.
- committed: hunav_gz_plugin lyu-lab + hunav_sim lyu-lab (see git log). Verification run: campus_verification/data/simtime_dwb (hot_switch.py, plaza_dense_s0 + door_exit_s0, 2 eps each, no warmup/prime). NEXT: verify.py speed/desired + reset_jump.py on it; then docs/06, docs/14 §12, patches (hunav_sim base a69cf96).
