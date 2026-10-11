# Resuming interrupted work (campus world + testing infrastructure)

Started 2026-10-11. Two Claude agents were working in parallel when the session may have stopped (token budget):

| Agent | Task | Progress file |
|---|---|---|
| infra | clear global costmap at reset; per-scenario Nav2 maps (scenario JSON `"map"` field + `scripts/scenario_maps.py` + arena-rosnav patch); hot-switching scheduler (`scripts/arena_batch.py`, `scenarios:` + `episodes_per_scenario:`); recorder labels each episode with its actual scenario | [`infra.md`](infra.md) |
| campus | `scripts/generate_campus.py` (world), all templates + gauntlet in `scripts/generate_campus_scenario.py` / `scenarios/campus/stress.yaml`; DWB + DS-RNN verification runs; docs/14 | [`campus.md`](campus.md) |

The full plan and requirements are in docs/14 and in the two progress files.

## To resume (tell Claude: "resume the campus/infra work, read handoff/README.md")
1. **Clean up the simulator first.** A sim or lock may be left over:
   ```bash
   bash scripts/arena-stop.sh
   cat /tmp/arena-sim.lock/owner 2>/dev/null && rm -rf /tmp/arena-sim.lock   # only if no sim is running
   ```
2. Check the state: `git status` / `git log` here and on the `lyu-lab` branches of
   `~/arena_ws/src/arena/arena-rosnav` and `.../simulation-setup`. Look for uncommitted or WIP work.
3. Read `infra.md` and `campus.md`; continue from "next". Interrupted batches resume with
   `uv run scripts/arena_batch.py <jobs.yaml> --resume`.
4. The verification criteria Taha asked for (campus agent):
   - (a) human behaviour typical: no clipping into walls, panels or each other, no stuck agents;
   - (b) scenarios respond to parameters, with the expected randomness across seeds;
   - (c) the Jackal can succeed in every scenario unless a failure is a genuine policy issue.

   Both DWB and DS-RNN, ≥3 seeds per template, ≥2 gauntlet seeds.
5. **After that (not started):** experiment notebooks with stats and replays across the campus scenarios (like
   `first_experiment/`) to judge whether the scenarios cover the policy-testing scope. Then decide: change the
   scenarios, or move on to other policies.
