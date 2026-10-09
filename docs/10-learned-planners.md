# 10. Learned planners (nav2py): install, PaS-CrowdNav bring-up, and adding your own

**Status (2026-09-28):** the nav2py stack is installed and **proven end to end**. Nav2 loads a learned planner's C++
plugin, the plugin starts the policy's own Python/PyTorch process, and velocity commands flow back at 4 Hz on the GPU.
PaS-CrowdNav drives the Jackal in `school_hallway`, but **it is not goal-directed yet** (see §4).

```bash
bash scripts/arena-launch.sh school_hallway scenario scenario:=hallway_tame.json local_planner:=crowdnav            # PaS-CrowdNav
bash scripts/arena-launch.sh school_hallway scenario scenario:=hallway_tame.json local_planner:=crowdnav rviz:=false # Gazebo only
```

---

## 1. Where everything lives

| What | Where | Git |
|---|---|---|
| Upstream planner wrappers (as listed in Arena's `planners.repos`) | `~/arena_ws/src/planners/<Name>/` | each is its own upstream repo; managed by `vcs` |
| nav2py bridge + CMake helpers | `~/arena_ws/src/planners/deps/{nav2py,ament_cmake_venv,ament_cmake_venv_uv}` | upstream (voshch) |
| Each planner's Python venv (own Python + torch) | `~/arena_ws/build/<pkg>/venv`, used via `install/<pkg>/venv` | build output |
| Which planner Nav2 loads for `local_planner:=X` | `~/arena_ws/src/arena/simulation-setup/configs/nav2/controllers/X/controller_config.yaml` | simulation-setup (`lyu-lab` branch) |
| **Our fixes to PaS-CrowdNav** | `~/arena_ws/src/planners/PaS_CrowdNav`, branch `lyu-lab` | ⚠️ upstream is `zenghjian/Nav2_PaS_CrowdNav`: needs a fork on your GitHub before it can be pushed |
| **Planners we write ourselves** (proposed) | a new repo of yours (e.g. `tahaismail424/crowdnav-planners`) cloned to `~/arena_ws/src/planners/lyu/` | yours. List it in a `.repos` file in this repo so the workspace can be rebuilt with one `vcs import` |

Why not inside `initial-arena-testing`? colcon only builds packages under `~/arena_ws/src`. Symlinking a package folder
there works for colcon (unlike the world-file case), but a separate repo keeps planner code (C++, pinned torch, weights)
out of your notes/experiments repo, and it can be shared or forked independently.

## 2. How a learned planner is wired (recap of [05](05-policies-and-pytorch.md#how-nav2py-works-read-from-the-source))

```
local_planner:=crowdnav
   └─► controllers/crowdnav/controller_config.yaml  plugin: nav2py_pas_crowdnav_controller::PasCrowdNavController
          └─► Nav2 controller_server loads that C++ class (pluginlib)
                 └─► configure(): popen("install/<pkg>/lib/<pkg>/nav2py_run --port 0")   # starts the Python policy
                        ◄── Python prints its TCP port + magic bytes on stdout; C++ connects
                 └─► every control tick: send("path", ...), send("costmap_pose", ...)  ──►  Python: build obs, torch, act
                                         ◄── two doubles: linear.x, angular.z
```

`local_planner:=X` "just works" only if (a) the package is built and (b) the wrapper feeds the network what it was
trained on. (b) is where most of the work is.

## 3. What was installed, and how

```bash
cd ~/arena_ws && source arena.bash
echo planners.sh >> src/arena/arena-rosnav/.installed                      # what Arena's 3_planners.sh records
vcs import --recursive --input src/arena/arena-rosnav/.repos/planners.repos src
colcon build --symlink-install --packages-select ament_cmake_venv ament_cmake_venv_uv nav2py nav2py_pas_crowdnav_controller
```

⚠️ **Use `--packages-select`, not `--packages-up-to`.** This workspace builds ROS 2 itself from source, so `up-to`
rebuilt 225 packages (rclcpp, Nav2, …) and took 40 min. `select` builds just what you name (PaS: ~20 s once its venv
exists).

| Planner (`local_planner:=`) | Built | Runs | Notes |
|---|---|---|---|
| PaS-CrowdNav (`crowdnav`) | ✅ | ✅ commands flow at 4 Hz, GPU | not goal-directed yet (§4) |
| DRL-VO (`drlvo`) | ✅ | Python side starts + handshakes ✅ (not run in sim yet) | pedestrian map input hard-coded to zeros in Arena's wrapper |
| CrowdNav++ (`crowdnav_attngraph`) | ✅ | ❌ Python exits at import: `No module named 'rclpy'` | prototype: pedestrian input comes from CrowdNav's internal toy simulator |
| SICNav (`sicnav`) | ✅ (after 3 fixes, §6) | Python side starts + handshakes ✅ | MPC (not neural); needs `ipopt` |

## 4. PaS-CrowdNav bring-up log (what broke, and the fix)

Every item was found by reading PaS's *training* code (`PaS_CrowdNav/crowd_sim/...`, `crowd_nav/policy/pas_rnn.py`)
and comparing it with Arena's wrapper (`nav2py_pas_crowdnav_controller/{__main__,pas_controller}.py`).

| # | Symptom | Cause | Fix |
|---|---|---|---|
| 1 | Nav2 ran PaS at 1 Hz | `controller_frequency: 1.0` hard-coded in `nav2.yaml`; PaS trained with 0.25 s steps | `nav2.yaml` now uses `${controller_frequency}`: default 1.0 in `configs/nav2/model_params.yaml`, **4.0 in `controllers/crowdnav/controller_config.yaml`**. Verified live: `Double value is: 4.0`. Note a `${x:-default}` fallback would arrive as a *string*, hence the default lives in model_params. |
| 2 | `Invalid handle. Cannot load symbol cublasLtCreate`, then Python exits, then `controller_server` dies (SIGPIPE) | the VM has system CUDA 12.9 on the global loader path (`/etc/ld.so.conf.d`); torch cu121 / cuDNN loaded the wrong `libcublasLt` | `__main__` re-execs itself once with the venv's `nvidia/*/lib` first on `LD_LIBRARY_PATH` (the exec keeps PID + stdout, so nav2py's handshake still works). A ctypes preload alone was **not** enough: cuDNN dlopens cuBLAS by path search. |
| 3 | Speed stuck at exactly 0.10 m/s, barely turning | wrapper treated outputs as absolute (v, ω) clipped to ±0.1 | training semantics: `(Δv ±0.1, Δθ ±0.25 rad)`, v accumulated up to v_pref = 0.5 m/s, ω = Δθ / 0.25 s |
| 4 | Robot drove backwards | goal fed as (gx, gy) in the **robot** frame; training uses **(px − gx, py − gy) in the world frame** + world θ | rotate by yaw and negate. Robot state (v, w) = last action, as in training |
| 5 | People appeared 1.5× smaller/closer | Nav2 local costmap is 15 × 15 m; PaS trained on 10 × 10 m (robot-centred, world-aligned, 0.1 m) | crop the centre 10 × 10 m before resizing to 96 × 96 |
| 6 | Every cycle "missed its desired rate of 4 Hz" | 22,500-number YAML costmap parsed with the pure-Python loader + a per-cell Python loop | C YAML loader + numpy. *(Rate after the fix not yet re-measured.)* |

**Still open:** in an offline rollout (empty costmap, goal 5 m straight ahead, unicycle kinematics, 30 steps), the policy
turns ~150° and drives away, **with either goal-sign convention**. So something deeper doesn't match: the shipped
`policy.pt`/`vae.pth` may come from a different config than `crowd_nav/configs/config.py` (holonomic? another
`robot_vector` layout?), or the wrapper's recurrent hidden-state/mask handling differs from `evaluation.py`. Next step,
if we pursue PaS: reproduce PaS's own `test.py` evaluation in its CrowdNav env with these weights. If it isn't
goal-directed *there* either, the weights/config are the problem, not Arena.

## 5. Workflow for adding a new planner (the checklist we just exercised)

1. **Read the paper's training env**, not just the README. Write down:
   - observation: every tensor, its frame (world vs robot), sign conventions (`p − g` vs `g − p`), units, grid size/resolution/orientation;
   - action: absolute or incremental, clip ranges, holonomic vs unicycle;
   - `time_step` (sets `controller_frequency`), v_pref and robot radius (set your speed caps).
2. **Copy a nav2py package** (PaS is a good template: C++ plugin sends path + costmap + pose; Python class loads the model).
   Rename the package, class, `<pkg>.xml` plugin description and `pyproject.toml` deps (pin torch).
3. **Add `controllers/<name>/controller_config.yaml`** with the plugin name and `controller_frequency`.
4. `colcon build --symlink-install --packages-select <pkg>`, then launch with `local_planner:=<name>`.
5. **Validate in layers:**
   (a) standalone inference on a synthetic input;
   (b) offline rollout toward a goal in empty space;
   (c) in-sim: plugin loads, commands flow, no rate misses;
   (d) behaviour with `check_run.py` / `check_crowd.py`.
   (b) is the cheapest way to catch convention bugs; it found #4 and the open issue above.
6. If the policy needs **exact pedestrian states** (SARL, RGL, DS-RNN, CrowdNav++, HEIGHT): read
   `/task_generator_node/human_states` (hunav_msgs/Agents). rclpy on Humble needs **Python 3.10**. A policy venv on
   3.8 can't subscribe itself, so the C++ plugin must subscribe and forward the data via `nav2py_send`.

## 6. Other planners' build status

Smoke test = start each planner's real launcher (`install/<pkg>/lib/<pkg>/nav2py_run --port 0`) outside the sim,
check it prints nav2py's handshake (port + magic bytes 187 201) and survives 40 s of initialisation without tracebacks.

* **PaS-CrowdNav, DRL-VO, SICNav:** pass.
* **CrowdNav++:** its venv (Python 3.10, torch 1.12.1+cu116) can't `import rclpy`. The wrapper imports ROS Python
  modules, which aren't installed into planner venvs. It needs either the ROS paths passed into the venv (possible, since it's
  Python 3.10 like Humble) or removal of the rclpy dependency. Not fixed; it's a prototype anyway (see [05](05-policies-and-pytorch.md)).
* **SICNav needed three fixes to build:**
  1. old `gym` metadata rejected by modern setuptools;
  2. `${venv_python}` renamed to `${venv_python_bin}` in ament_cmake_venv (edited `Sicnav/CMakeLists.txt`);
  3. `uv` won't overwrite an existing venv.

  Working recipe:
  ```bash
  printf 'setuptools==65.5.0\nwheel==0.38.0\n' > /tmp/sicnav_constraints.txt
  rm -rf ~/arena_ws/build/nav2py_sicnav_controller/venv
  PIP_CONSTRAINT=/tmp/sicnav_constraints.txt colcon build --symlink-install --packages-select nav2py_sicnav_controller
  ```
  (`wheel` must be 0.38.0: SICNav pins it exactly.)
* A **failed** package leaves a half-made `install/<pkg>` that makes every `source arena.bash` print
  `not found: .../local_setup.bash`. Delete `install/<pkg>` and `build/<pkg>` if you give up on a package.
## 7. DS-RNN in its native simulator (sanity check before porting)

Before wiring DS-RNN into Arena, we confirmed that the pretrained **unicycle** checkpoint reproduces its published numbers
in the authors' own CrowdNav simulator. If it fails there, it would fail in Arena too.

- Code: `~/Documents/Lyu_Lab/CrowdNav_DSRNN` (upstream: Shuijing725/CrowdNav_DSRNN)
- Dependency: `~/Documents/Lyu_Lab/baselines` (openai/baselines; only `vec_env`, `logger`, `bench` are used)
- Environment: uv, defined by a local `pyproject.toml` in CrowdNav_DSRNN (Python 3.8, CPU torch 1.7.1, gym 0.15.7,
  numpy 1.23, baselines editable from `../baselines`, Python-RVO2 built from git). No TensorFlow.

```bash
cd ~/Documents/Lyu_Lab/CrowdNav_DSRNN
uv sync                      # creates .venv (first time builds RVO2 with cmake + Cython)
MPLBACKEND=Agg uv run python test.py --model_dir data/example_model_unicycle --test_model 55554.pt   # 500 episodes, ~3 min on CPU
uv run python test.py --model_dir data/example_model_unicycle --test_model 55554.pt --visualize --test_case 0   # watch one (needs a display)
```

Results go to `data/example_model_unicycle/test/test_55554.pt.log` (the authors' original is kept as `.log.upstream`).

| | success | collision | timeout | nav time | path length | CHC |
|---|---|---|---|---|---|---|
| Authors' log (2021) | 0.88 | 0.12 | 0.00 | 11.79 s | 11.06 m | 22.97 |
| Ours (CPU, 2026-10-09) | 0.89 | 0.11 | 0.00 | 11.87 s | 10.94 m | 21.28 |

### What had to change

| Where | Change | Why |
|---|---|---|
| CrowdNav_DSRNN `pyproject.toml` (new) | uv project, `package = false`, CPU torch index, `pyrvo2` from git with `cython<3` as an extra build dep | Upstream only has a README list (Python 3.6, torch ≤1.7.1). Python 3.8 is the newest CPython with torch 1.7.1 wheels. The RVO2 distribution is named `pyrvo2` (the module is still `import rvo2`), and its `setup.py` imports Cython at build time. |
| baselines `setup.py` | The "TensorFlow needed" `assert` became a warning | It refuses to install without TF, but DS-RNN never touches TF code. uv's build-only deps didn't help, because the assert also fires while uv queries build requirements. |
| CrowdNav_DSRNN `test.py` | `config.training.cuda = config.training.cuda and torch.cuda.is_available()` | The saved config says `cuda = True`, so it crashes on this CPU-only VM |
| `pytorchBaselines/evaluation.py` | `float(...)` around the cumulative reward | Rewards are torch tensors, and numpy ≥1.20's `np.average` crashes on a list of them (at the very end, after all 500 episodes) |
| same | Path length uses robot_node indices 0, 1 (px, py), not 1, 2 (py, radius) | Upstream bug: gave nonsense path lengths |
| same | Skip the path/CHC update on the terminal step | The vec env auto-resets, so the last `obs` is the next episode's start. That added a 6–12 m jump to every path. |

All code edits carry a `Lyu Lab local change` comment (`grep -rn "Lyu Lab local change"`). Both repos are upstream
clones, so these edits should go on `lyu-lab` branches of your forks.
