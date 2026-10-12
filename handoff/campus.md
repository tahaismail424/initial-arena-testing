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
| matrix_dsrnn (+ .csv) | DS-RNN, 5 templates x seeds 0-2 (blind_corner = OLD untimed version) | done: corridor 0/3, door 0/3 (pinned on panels), blind 3/3, intersection 2/3, plaza 3/3 |
| matrix_dwb (+ .csv) | DWB, 5 templates x seeds 0-2, --prime, timeout 300 | done: 15/15 success |
| of10_dwb | obstacle_force 10 test (people closer, 0.3 m into panels) -> kept 30 | done |
| matrix_dsrnn_c | DS-RNN blind_corner s0-2 (timed) + gauntlet s0,s1, timeout 300 | RUNNING (00:02 UTC) |
| (next) matrix_dwb_g | DWB gauntlet s0,s1, --prime --timeout 900 | todo |
Analyse: `uv run campus_verification/verify.py <dir> --out <dir>.csv`; params: campus_verification/params_check.txt.

## In progress (2026-10-12 00:05 UTC)
- Runs above. Then docs/14 rewrite (all sections except §9/§11) + figures docs/img/campus_*; final report.
- Infra-domain issues collected in the session scratchpad campus_issues_for_infra.md (also to be listed in
  docs/14 limitations): HuNav wall-clock integration (speed x 1/RTF, reset throws people up to 8-22 m, through
  walls), non-cyclic agents segfault the HuNav plugin, int fields dropped, no timeout in arena_batch.

## Sims / locks
- hot_switch.py holds /tmp/arena-sim.lock while running; if the session died: bash scripts/arena-stop.sh;
  rm -rf /tmp/arena-sim.lock.
