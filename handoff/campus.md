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

## In progress
- Pilot DS-RNN run (campus_verification/data/pilot_dsrnn, gitignored): corridor_counterflow_s0 and door_exit_s0
  -> DS-RNN pinned on panels (policy) and sat until timeout; blind_corner_s0 success 31.8 s; intersection_s0 ran
  > 7 min (not yet analysed).
- Findings to act on: (1) first reset after launch: pedestrians jump 5-8.5 m (later resets ~0.6 m) -> warmup
  scenario; (2) pedestrian-pedestrian overlap frequent in dense flows (door: 87% of samples have a pair < 0.4 m)
  -> investigating (social_force_factor / density).

- Gotcha found: integer force factors (e.g. 12) in a scenario make Arena DROP the agent ("must be of type
  'float'"); Builder now coerces, check_scenario rejects non-floats.
- Force tuning (pedestrian overlap): variants worlds/campus/scenarios/tune_*_sf12/sf20.json (temporary, not
  committed) running in campus_verification/data/tune_forces2 (hot_switch.py, DWB, timeout 70 s).
  Baseline sf5/of10: overlap (<0.4 m) share of pedestrian-time 23-24% door/corridor, 3% intersection, 9% plaza.
- Infra test files (ilt_*, spike_*) removed from repo + Arena links (coordinator OK'd).
- Scheduler: arena_batch.py (infra, multi-scenario) has no episode-timeout / warmup option; hot_switch.py kept for
  --timeout + warmup.json.

## Next
- Fix ped overlap, re-pilot; then the run matrix: 5 templates x 3 seeds x {dwb, dsrnn} (1-2 eps) + gauntlet x 2
  seeds x {dwb, dsrnn}. Use hot_switch.py (or arena_batch.py once infra adds scenarios globs), timeout 240 s for
  templates, 900 s for DWB gauntlet.
- docs/14 rewrite (not §9/§11), figures docs/img/campus_*.
- Spike files worlds/school_hallway/scenarios/spike_*.json: infra agent still has ilt_* there; leave until confirmed.

## Sims / locks
- None held by the campus agent right now.
