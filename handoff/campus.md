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

## Run data (gitignored, campus_verification/data/)
| dir | what | status |
|---|---|---|
| matrix_dsrnn (+.csv) | DS-RNN 5 templates x s0-2 | corridor 0/3, door 0/3 (pinned on panel ends: policy), intersection 2/3 (pinned at riser wall end), plaza 3/3; blind = old version (superseded) |
| matrix_dwb (+.csv) | DWB 5 templates x s0-2, --prime, timeout 300 | 15/15 success |
| matrix_dsrnn_c (+.csv) | DS-RNN blind s0-2 (timed, old lane), gauntlet s0,s1 | blind 3/3; gauntlets pinned at corridor mouth (5.92, 4.7) at 7-12 s: policy |
| matrix_dwb_g (+.csv) | DWB gauntlet s0, s1 | s0 success 316.9 s; s1 1st attempt deadlock at inside corner (people pushed into robot+wall 280 s; fixed by lane change 8525b0d), relaunch success 319.6 s |
| final_dsrnn / final_dwb | rerun of blind_corner s0-2, plaza s0-2, gauntlet s0,s1 on the final generator (8525b0d) | RUNNING (from ~03:40 UTC), chained in one nohup |
| of10_dwb, tune_forces*, pilot* | force tuning / pilots | done (docs/14 §8 table) |

## In progress
- docs/14 rewrite: draft of §1-§8 in the session scratchpad doc_head.md (copy to docs/14 then write §10
  verification + §12 limitations; keep §9 and §11 verbatim). Then README index line, final report.

## Sims / locks
- hot_switch.py holds /tmp/arena-sim.lock while running; if the session died: bash scripts/arena-stop.sh;
  rm -rf /tmp/arena-sim.lock.
