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

## In progress (2026-10-11 23:00 UTC)
- Commits so far: be1951f (WIP templates), 7c412da (float fields), 6c3c0b1 (door exit-once wave, lanes, forces
  20/30), + hot_switch crash-relaunch (this commit).
- RUNNING: DS-RNN template matrix, seeds 0-2 x 5 templates, 1 ep each, timeout 150 s:
  `uv run campus_verification/hot_switch.py --planner dsrnn --episodes 1 --timeout 150 --max-minutes 90
  --out campus_verification/data/matrix_dsrnn <15 scenario files>` (log campus_verification/data/matrix_dsrnn.out).
  To resume: rerun with the scenarios whose episodes are missing (verify.py lists what exists).
- Findings: (a) HuNav integrates on WALL time -> pedestrian sim-time speed = desired / RTF (plaza 27 people,
  RTF 0.71 -> 1.37x); first reset after launch -> 5-8.5 m jump (warmup.json avoids it); int force factors drop
  agents. Notes for infra: scratchpad campus_issues_for_infra.md (copy into the final report).
  Ped overlap: head-on on the SAME line (single-file door, same corridor lane) is what makes HuNav people walk
  through each other -> door_exit is now one-way (exit wave, people leave along the crossing corridor and stop),
  two-way door loops only for doors >= 1.7 m (gauntlet), corridor lanes always 0.6 m apart, forces 20/30.
  DS-RNN pins itself on walls/panel corners near people (policy) -> sometimes Gazebo segfaults (hot_switch
  relaunches).
- Infra test files (ilt_*, spike_*) removed (also dangling links in ~/arena_ws build/ and install/).

## Next
- DWB template matrix (timeout 300), gauntlet s0,s1 x {dwb (timeout 900), dsrnn (timeout 300)}.
- Analysis per criterion a/b/c (verify.py + manifest), figures docs/img/campus_*, docs/14 rewrite (not §9/§11).

## Sims / locks
- hot_switch.py holds /tmp/arena-sim.lock while running; if the session died: bash scripts/arena-stop.sh;
  rm -rf /tmp/arena-sim.lock.
