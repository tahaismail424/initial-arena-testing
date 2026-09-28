# initial-arena-testing

Personal study notes and experiments for **Arena 5.0 (arena-rosnav, `humble` branch)** on the Lyu Lab GCP VM.
The goal: get comfortable enough with the stack to change simulations, robots, and navigation policies,
then benchmark crowd-navigation policies in a configurable **school-hallway** world.

Everything here was written from the actual install at `~/arena_ws` (inspected from 2026-09-25 on), not from generic docs.
Arena's upstream docs describe older versions in places. When the docs and the code disagree, trust the code.

## Read in this order

| # | Guide | What you get out of it |
|---|---|---|
| 1 | [Mental model](docs/01-mental-model.md) | What Gazebo, ROS 2, Nav2, HuNav, RViz and Arena each do, and how one velocity command flows through them |
| 2 | [Workspace & build](docs/02-workspace-and-build.md) | Where every repo lives, which files are "live", and when you need to rebuild |
| 3 | [Running & inspecting](docs/03-running-and-inspecting.md) | Launch arguments, a `ros2` CLI cheat-sheet, and how to poke at a running sim |
| 4 | [Configuring sim & robot](docs/04-configuring-sim-and-robot.md) | Worlds, scenarios, pedestrians, physics, robot sensors/limits, and the Nav2 YAML merge chain |
| 5 | [Policies & PyTorch](docs/05-policies-and-pytorch.md) | Swapping classical planners, installing the learned ones (DRL-VO, CrowdNav, SICNav), and wiring in your own PyTorch model |
| 6 | [Local patches & gotchas](docs/06-local-patches-and-gotchas.md) | Everything we changed in Arena and why, plus things that will bite you |
| 7 | [Study plan](docs/07-study-plan.md) | Hands-on exercises that build up to the policy comparison |
| 8 | [Ground-truth localization](docs/08-ground-truth-localization.md) | Worked example of adding a configurable feature to Arena (the `localization` switch) |
| 9 | [Hallway crowd scenarios](docs/09-hallway-scenarios.md) | Spec → JSON → HuNav: agent classes, what every field does, and the pitfalls we hit |

## Quick start

```bash
cd ~/Documents/Lyu_Lab/initial-arena-testing
uv run scripts/generate_hallway.py -n school_hallway --link                  # world
uv run scripts/generate_scenario.py scenarios/hallway_tame.yaml --link       # crowd
bash scripts/arena-launch.sh school_hallway scenario scenario:=hallway_tame.json               # watch
bash scripts/arena-launch.sh school_hallway scenario scenario:=hallway_tame.json headless:=2   # benchmark
```

Run the launch from a terminal in the VM's graphical desktop. `Ctrl+C` stops everything. After `--link` creates *new*
files, rebuild once: `cd ~/arena_ws && source arena.bash && colcon build --symlink-install --packages-select arena_simulation_setup`.

## Repo layout

| Path | What |
|---|---|
| `docs/` | The guides above |
| `scripts/arena-launch.sh` | Launch wrapper (VirtualGL, env fixes, `localization:=ground_truth` default) |
| `scripts/generate_hallway.py` | Parametric hallway world generator (map.png / map.yaml / walls.yaml / scenario / preview) |
| `scripts/generate_scenario.py` | Crowd scenario generator: spec YAML → scenario JSON + preview |
| `scripts/check_run.py` | Believed-vs-true pose + Nav2 goal status for a running episode |
| `scripts/check_crowd.py` | Pedestrian health for a running episode: per-class speeds, gaps, stuck agents, RTF |
| `scenarios/` | Crowd **specs** (e.g. `hallway_tame.yaml`) |
| `worlds/` | Generated worlds + scenarios (the source of truth; Arena sees them via per-file symlinks) |
| `patches/` | `git diff` exports of our changes to Arena, with how to re-apply them |

## Current fixed setup

| Setting | Value |
|---|---|
| Robot | Jackal (skid-steer diff-drive: forward speed `v` + yaw rate `ω`, turns in place) |
| World | `school_hallway`: 60 × 12 m hallway, 3 m cross corridors at x = 15/30/45 extending 4 m each side |
| Scenario | `hallway_tame.json`: 20 pedestrians (two-way walkers, distracted, runners, 2 friend groups, corridor walkers). Robot (1, 6) → (59, 6). |
| Localization | **ground truth** (exact Gazebo pose, no AMCL). This removes a confounder; see [08](docs/08-ground-truth-localization.md). |
| Episodes | end on goal or after 600 sim-seconds (`timeout`), then reset automatically |
| Baseline policy | NavFn (global) + DWB (local) |
| Simulator | Gazebo Harmonic, HuNav pedestrians |

## Related files outside this repo

- `~/Documents/ChatGPT/Master's Research/arena-scenario-audit/`: static audit of the installed scenario files
- `~/arena-install/install*.log`: logs from the install attempts
- `chat_history.md` (this repo): the install/debug conversation these notes build on
