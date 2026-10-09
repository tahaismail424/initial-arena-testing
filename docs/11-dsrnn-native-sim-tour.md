# 11. DS-RNN's native simulator: a guided tour

A map of `~/Documents/Lyu_Lab/CrowdNav_DSRNN`: what runs during an experiment, what each file does, and which knobs
matter. Install notes and benchmark numbers are in [10 §7](10-learned-planners.md#7-ds-rnn-in-its-native-simulator-sanity-check-before-porting).

## 1. Watching it

```bash
cd ~/Documents/Lyu_Lab/CrowdNav_DSRNN
uv run python test.py --model_dir data/example_model_unicycle --test_model 55554.pt --visualize                 # random test episodes, one after another
uv run python test.py --model_dir data/example_model_unicycle --test_model 55554.pt --visualize --test_case 7    # replay test case 7 forever
```

- Don't set `MPLBACKEND=Agg` here. That backend draws to memory only, so you get the
  "Matplotlib is currently using agg" warning and no window.
- `--test_case N` resets to the **same seed every episode**. That's why every episode in your run was identical
  (reward 28.39, 111 steps). Leave it off to cycle through the 500 test cases.
- Close the window or press Ctrl+C to stop. A visual run writes `data/<model>/test/test_visual.log`.

**What you're looking at** (a 12 × 12 m view, redrawn every 0.1 s step):

| Drawing | Meaning |
|---|---|
| Yellow filled circle | The robot. Its red arrow shows the heading. |
| Red star | The robot's goal |
| Hollow numbered circles | Humans. Each arrow shows its velocity direction. Green = inside the robot's field of view, red = outside. With FOV = 2π they're all green. |
| Dashed lines | The robot's FOV edges, drawn only when FOV < 2π |

### Why it didn't show anything (fixed)

1. uv's Python ships tkinter in a form matplotlib 3.7 can't load. matplotlib silently fell back to `agg`, which
   has no window. **Fix:** added `pyqt5` to the env so matplotlib uses Qt (`QtAgg`).
2. baselines depends on `opencv-python`, which bundles its *own* Qt plugins and hijacks Qt's plugin path
   ("Could not load the Qt platform plugin xcb"). **Fix:** baselines' `setup.py` now asks for `opencv-python-headless`.

## 2. What actually runs: the call chain

```
test.py
 ├─ Config  ← data/<model>/configs/config.py   (a snapshot saved at training time; falls back to crowd_nav/configs/config.py)
 ├─ make_vec_envs()              pytorchBaselines/a2c_ppo_acktr/envs.py
 │    └─ gym.make('CrowdSimDict-v0')   crowd_sim/envs/crowd_sim_dict.py  (subclass of crowd_sim/envs/crowd_sim.py)
 │         wrapped in baselines Monitor → DummyVecEnv → VecPyTorch (numpy ↔ torch)
 ├─ Policy(base='srnn')          pytorchBaselines/a2c_ppo_acktr/model.py       actor-critic + Gaussian action head
 │    └─ SRNN network            pytorchBaselines/a2c_ppo_acktr/srnn_model.py  ← THE neural network (DS-RNN)
 └─ evaluate()                   pytorchBaselines/evaluation.py                episode loop + metrics
```

`train.py` uses the same env and network. It reads **`crowd_nav/configs/config.py`**, trains with PPO
(`pytorchBaselines/a2c_ppo_acktr/algo/`), and copies that config into `<output_dir>/configs/`. That copy is why every
model folder carries its own config. **Note:** the top-level `crowd_nav/configs/config.py` is set up for the
*holonomic* model, so the unicycle settings live only in `data/example_model_unicycle/configs/config.py`.

### What's in `crowd_nav/policy/`, and is it used?

These are **not** the neural network. They're the "brains" attached to the simulated *agents*:

| File | Used in the experiments? | What it is |
|---|---|---|
| `srnn.py` | **Yes**, attached to the robot | A thin wrapper that only **clips the network's raw action** (`clip_action`). Unicycle: Δv and Δθ are each clipped to **±0.1** per step. Holonomic: (vx, vy) is clipped to ‖v‖ ≤ v_pref. |
| `orca.py` | **Yes**, the humans' policy (`humans.policy = "orca"`) | ORCA collision avoidance via Python-RVO2. Each human sees the other humans, and the robot only if `robot.visible`. |
| `social_force.py` | No (alternative human policy) | Social-force humans: `humans.policy = "social_force"`, with parameters in `config.sf`. |
| `policy.py`, `policy_factory.py` | Plumbing | Base class, and the name → class map used by the config strings. |

## 3. One environment step (`CrowdSimDict.step`)

1. **Clip** the network output (srnn.py above): `a = (Δv, Δθ)`, each in ±0.1.
2. **Unicycle integration:** `v = clip(v + Δv, -v_pref, v_pref)`. The robot keeps a *running* speed, so the network
   steers acceleration, not speed. Turning is `Δθ` radians this step, i.e. ω = Δθ / 0.1 s, at most 1 rad/s.
3. **Humans** each compute an ORCA action.
4. **Reward / termination** are computed (§5).
5. **Move everyone.** The robot follows a differential-drive arc (`agent.py: compute_position`). Humans move holonomically.
6. **Build the observation** (§4).
7. **Re-goal humans** that reached their goal (`end_goal_changing`, chance 1.0). Humans keep walking all episode.

## 4. The observation (what the network sees)

| Key | Shape | Contents | Frame |
|---|---|---|---|
| `robot_node` | 1 × 7 | px, py, radius, gx, gy, v_pref, θ | **Absolute world coordinates** |
| `temporal_edges` | 1 × 2 | robot vx, vy | World frame |
| `spatial_edges` | 5 × 2 | (human − robot) position, one row per human | World-frame offsets |

- Humans outside the FOV are frozen at their last observed state, or set to (15, 15) at reset.
- No human velocities, no walls, no map. The policy knows humans only by where they are over time; the RNNs infer motion.
- The network:
  - a temporal-edge GRU on the robot's velocity;
  - a spatial-edge GRU on each human offset (weights shared across humans);
  - attention over the humans;
  - a robot-node GRU, fed by the attention output and an embedding of `robot_node`;
  - actor and critic heads.
- At test time `deterministic=True` takes the mean of the Gaussian, with no sampling.

## 5. Episodes and reward

**Episode layout** (FoV mode, `group_human = False`, unicycle):
- **Robot start:** on a circle of radius 6 at a random angle, with a random heading.
- **Robot goal:** uniform in the square [−6, 6]², at least 6 m from the start.
- **5 humans:** start on the radius-6 circle (± noise), heading for the antipodal point. Each starts at least 3 m from
  the robot's start and goal.
- `randomize_attributes`: each human gets v_pref ~ U(0.5, 1.5) m/s and radius ~ U(0.3, 0.5) m.
- `robot.visible = False`: **humans don't react to the robot.** The robot does all the avoiding.
- Seeds: test case k uses seed 1000 + k, so test runs are reproducible.

**Reward** (`crowd_sim.py: calc_reward`), per step:

| Event | Reward | Ends episode? |
|---|---|---|
| Reach goal (within robot radius) | +10 | yes |
| Collision | −20 | yes |
| Time ≥ 49 s | 0 (Timeout) | yes |
| Within 0.25 m of a human ("Danger") | (d − 0.25) · 10 · 0.1 | no |
| Otherwise | 2 × (progress toward goal this step, m) | no |
| Always (unicycle) | −2·Δθ², and −2·\|v\| if driving backwards | — |

## 6. Configurables

Edit the config **inside the model folder** (`data/example_model_unicycle/configs/config.py`) to change what
`test.py` does. Edit `crowd_nav/configs/config.py` to change what `train.py` trains.

| Setting | Default (unicycle model) | Safe to change at test time? |
|---|---|---|
| `env.test_size` | 500 | Yes (number of test episodes) |
| `env.time_limit` | 50 s | Yes |
| `sim.human_num` | 5 | Changes the input size. Not tested: the per-human weights are shared, so it may run, but it's out of the training distribution. |
| `sim.circle_radius` | 6 m | Changes the arena size. Out of distribution, because positions are absolute. |
| `sim.group_human` | False | Switches to the "group" scenario (static circles of humans). Not what this model was trained on. |
| `humans.policy` | orca | `social_force` gives differently-behaving humans. A good robustness test. |
| `humans.v_pref`, `humans.radius`, `env.randomize_attributes` | 1, 0.3, True | Yes: crowd speed and size |
| `robot.visible` | False | Yes: True makes humans yield to the robot (easier) |
| `robot.FOV` | 2 (×π) | Yes: smaller values hide humans (harder) |
| `noise.add_noise`, `noise.magnitude` | False, 0.1 | Yes: sensor-noise robustness |
| `robot.v_pref`, `env.time_step`, `action_space.kinematics` | 1 m/s, 0.1 s, unicycle | **No.** These are baked into what the network learned. |
| `reward.*`, `ppo.*`, `training.*`, `SRNN.*` | — | Training only (`SRNN.*` must match the checkpoint, or loading fails) |

## 7. Quirks found while reading the code

- **Mid-episode goal changes never happen.** `random_goal_changing` (25% chance every 5 s) is gated on
  `global_time % 5 == 0`, but time is a float sum of 0.1 steps that never lands exactly on a multiple of 5. Only the
  "new goal after reaching the old one" rule is active.
- **Unicycle "straight line" bug:** if |Δθ| < 1e-4, `compute_position` uses turning radius R = 0 and the robot
  **doesn't move** that step. This is rare with a continuous policy output. Arena's real kinematics don't have the problem.
- `--test_case` replays one seed forever (see §1).

## 8. What matters for porting to Arena

- The robot's **absolute** px, py, gx, gy go into the network, and it only ever saw values within about ±6 m. Feed it a
  local frame: robot and nearby goal shifted near the origin, e.g. a waypoint about 6 m ahead on Nav2's path.
- **Exactly 5 humans, positions only, 10 Hz control, v ≤ 1 m/s, |ω| ≤ 1 rad/s, acceleration ≤ 1 m/s².** Carry the
  running speed v between ticks.
- There are **no walls** in training. A hallway is out of distribution. Expect the policy to need Nav2's global path
  (and maybe a safety layer) for walls.
- Trained humans ignore the robot. That matches our HuNav setup, where the robot isn't socially perceived yet.
