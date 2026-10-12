# Campus world handoff (campus agent)

Last update: 2026-10-11 22:30 UTC

## Done
- `scripts/generate_campus.py`: TODOs implemented (free space union + checks, walls via gen_walls, map.png,
  default.json = gauntlet task, warmup.json = 4 m drive without people). Anchors: corridor_mid (18,2.5) added,
  corridor_east moved to (26,2.5).
- `scripts/generate_campus_scenario.py`: all templates implemented (corridor_counterflow revised, door_exit,
  blind_corner, intersection, plaza_dense, gauntlet), Builder loop helpers (phase_on_loop/add_on_loop),
  static checker `check_scenario` (points in free space, line of sight of every leg, robot start/goal clear,
  unrotated + unique panels; generation fails on any problem), per-scenario maps via infra's
  `scripts/scenario_maps.py` (flag `--no-map`). Same seed -> identical JSON verified (30 files).
- `scenarios/campus/stress.yaml`: parameter blocks for all templates + gauntlet (smaller nested crowds).
- Generated: worlds/campus (seeds 0-4 for every template), linked into Arena, arena_simulation_setup rebuilt.
- Checkpoint B: DWB drove empty gauntlet default.json: success 318.8 s
  (campus_verification/data/manual/20261011-211247_dwb_default).
- `campus_verification/verify.py` (per-episode checks a/b/c), `hot_switch.py` (several scenarios per launch,
  same lock; --timeout, warmup), `reset_jump.py`.

## STATUS: DONE (2026-10-12 04:40 UTC)
All tasks complete: world, 5 templates + gauntlet, per-scenario maps, verification runs (both planners),
docs/14 rewritten (sections except 9/11), figures docs/img/campus_*.png, README index. Final commit: see git log.
Results table: campus_verification/summary_table.md (from summarize.py). No sim running, no lock held.

## Run data (gitignored, campus_verification/data/)
| dir | what | result |
|---|---|---|
| matrix_dwb / matrix_dsrnn | corridor, door, intersection (s0-2), both planners | DWB 9/9; DS-RNN corridor 0/3, door 0/3, intersection 2/3 (wall pins = policy) |
| final_dwb / final_dsrnn | blind_corner s0-2, plaza s0-2, gauntlet s0-1 on the final generator (8525b0d) | DWB 8/8 (gauntlet 312.0 / 321.4 s); DS-RNN blind 2/3, plaza 3/3, gauntlet 0/2 (wall pins) |
| matrix_dsrnn_c, matrix_dwb_g, of10_dwb, tune_forces*, pilot* | superseded versions / tuning | see docs/14 §8, §10 |

## Infra-domain issues (also docs/14 §12)
HuNav wall-clock integration (speed x 1/RTF; reset throws people 4-22 m, through walls); cyclic_goals false
segfaults the HuNav plugin; int HuNav fields dropped silently; Gazebo Ogre abort on a big scenario switch;
arena_batch.py lacks per-job timeout/priming (hot_switch.py has them); DS-RNN robot already moving at episode start.

