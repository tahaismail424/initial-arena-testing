"""Builds first_experiment.ipynb (run once; then just use the notebook). Execute it with:
    uv run jupyter nbconvert --to notebook --execute --inplace first_experiment.ipynb
"""
import nbformat as nbf

nb = nbf.v4.new_notebook()
cells = []


def md(text):
    cells.append(nbf.v4.new_markdown_cell(text.strip("\n")))


def code(text):
    cells.append(nbf.v4.new_code_cell(text.strip("\n")))


md("""
# First experiment: DS-RNN vs DWB in the tame hallway

**What was run** (`run_experiment.sh`): 20 recorded episodes of each planner, back to back, in one Arena launch per
planner:

| | |
|---|---|
| World / scenario | `school_hallway` (60 × 12 m, 3 side corridors), `hallway_tame.json`: 20 pedestrians (walkers both ways, distracted, 2 runners at ~2.7 m/s, groups, corridor crossers) |
| Robot task | Jackal from (1, 6) to the goal at (59, 6); success = within 0.25 m of the goal position (heading ignored) |
| Planners | **DS-RNN** (learned, `nav2py_dsrnn_controller`, ≤ 1.0 m/s) and **DWB** (Nav2's classical default, ≤ 0.26 m/s) |
| Global planner | NavFn for both. Localization: ground truth. Episode timeout: 600 s |
| Mode | headless, real-time (RTF ≈ 1), CPU-only VM |

**Read the results with these caveats in mind:**
1. **Same scenario 20 times.** Start, goal and every pedestrian's route are identical in each episode; the only
   episode-to-episode variation is timing noise in the simulator. So the confidence intervals describe *this one
   scenario*, not crowds in general. A real benchmark would randomise scenarios (different seeds / densities).
2. **Asymmetric perception (on purpose).** DS-RNN gets every pedestrian's exact position from HuNav; DWB only sees
   people as lidar obstacles in its costmap.
3. **Different speed limits.** DWB's own config caps it at 0.26 m/s, DS-RNN at 1.0 m/s. Time-to-goal mostly reflects that.
4. **Pedestrians ignore the robot.** HuNav's people don't see the robot, so they never yield. Every contact is the
   robot failing to get out of the way (or being run into while standing still).

Everything below is computed from the recorded positions (`social_metrics.py`), identically for both planners.
""")

code("""
%matplotlib inline
import warnings
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from IPython.display import Video, display

from arena_analysis import load_run
import social_metrics as sm
import plots

warnings.filterwarnings("ignore", category=FutureWarning)
pd.set_option("display.width", 200)
pd.set_option("display.max_columns", 40)
pd.set_option("display.precision", 2)

DATA = Path("data")
N = 20   # episodes per planner
""")

md("""
## 1. Load the recordings

Each planner's launch wrote one *run* folder (`data/<time>_<planner>_hallway_tame/`) with one sub-folder per episode.
We take the newest run per planner and keep the first `N` finished episodes (the last one was cut off when the sim
was stopped and is labelled `incomplete`).
""")

code("""
episodes = {}
for planner in ["dsrnn", "dwb"]:
    run_dir = sorted(DATA.glob(f"*_{planner}_*"))[-1]
    run = load_run(run_dir)
    done = [ep for ep in run.episodes if ep.meta.get("outcome") != "incomplete"][:N]
    episodes[planner] = done
    print(f"{planner}: {len(done)} episodes from {run_dir.name}")
""")

md("""
## 2. One row per episode

`sm.episode_metrics(ep)` returns ~40 numbers per episode. The most useful ones:

| column | meaning |
|---|---|
| `success` | Nav2 reported the goal reached (else `timeout` / `aborted`) |
| `time_to_goal` | seconds until the robot was within the goal tolerance |
| `path_efficiency` | straight-line distance / distance driven (1 = perfectly straight) |
| `collisions` | number of robot-pedestrian **contacts** (bodies overlapping: centre distance < robot radius 0.27 m + pedestrian radius 0.35 m) |
| `min_clearance` | closest the robot's body came to any pedestrian's body (m; negative = overlap) |
| `min_ttc` | smallest **time-to-collision**: if everyone kept their velocity, how many seconds until a contact. Below ~1 s is dangerous |
| `time_ttc_danger` | seconds with TTC < 1 s (imminent-collision situations, even if they were resolved) |
| `time_intimate` / `time_personal` | seconds with the closest pedestrian in the intimate (< 0.45 m) / personal (0.45–1.2 m) zone |
| `n_freezes`, `freeze_time` | robot standing still (< 0.05 m/s for ≥ 1 s) before reaching the goal |
| `mean_abs_jerk` | average jerk (m/s³), how abrupt the speed changes are |
| `heading_per_m` | total turning per metre driven (rad/m), "path irregularity" |
| `omega_flips_per_min` | how often the turn command switches direction per minute: weaving / oscillation |
""")

code("""
table = pd.DataFrame([{**sm.episode_metrics(ep), "planner": p} for p, eps in episodes.items() for ep in eps])
cols = ["planner", "episode", "outcome", "time_to_goal", "path_efficiency", "collisions", "min_clearance", "min_ttc",
        "time_ttc_danger", "time_intimate", "n_freezes", "freeze_time", "mean_abs_jerk", "omega_flips_per_min"]
table[cols]
""")

md("""
## 3. Headline comparison

Rates (fraction of episodes) come with a 95% **Wilson** interval; averages with a 95% **bootstrap** interval
(resampling episodes). Overlapping intervals mean the difference may just be noise.
""")

code("""
sm.compare(table)
""")

md("""
### Are the differences real?

`plots.significance` runs **Fisher's exact test** on the rates and the **Mann-Whitney U test** on the per-episode
values (it compares the distributions without assuming they're normal). p < 0.05 is the conventional threshold.
With this many metrics, one "significant" result in twenty can be luck. And remember caveat 1: "significant" here
means "reproducibly different *on this scenario*".
""")

code("""
plots.significance(table, "dsrnn", "dwb")
""")

md("""
## 4. Task performance: does it get there, and how directly?
""")

code("""
plots.distributions(table, ["time_to_goal", "path_length", "path_efficiency", "mean_speed"],
                    labels={"time_to_goal": "time to goal [s]", "path_length": "path length [m]",
                            "path_efficiency": "path efficiency", "mean_speed": "mean speed [m/s]"});
""")

code("""
plots.speed_profile(episodes);
""")

md("""
The speed profile shows *where* each planner slows down. Dips line up with the crowd: the side corridors at x ≈ 15,
30 and 45 m are where crossers enter, and where the two groups meet the robot.
""")

md("""
## 5. Safety: collisions, near-misses, time-to-collision

**Collision rate** (fraction of episodes with ≥ 1 contact) is the standard headline. The details matter more,
though: *when* and *where* contacts happen, and *who moved into whom*. A robot that drives into people is a very
different failure from one that stands still and gets walked into (remember: these pedestrians never yield).
""")

code("""
contacts = pd.concat([sm.contact_events(ep).assign(planner=p, episode=ep.meta["episode"])
                      for p, eps in episodes.items() for ep in eps], ignore_index=True)
print("contacts per planner:", contacts.groupby("planner").size().to_dict())
contacts[["planner", "episode", "name", "t_rel", "x", "y", "duration", "min_clearance",
          "robot_speed", "human_speed", "robot_toward", "human_toward", "robot_active"]]
""")

code("""
if len(contacts):
    plots.contacts_overview(contacts)
    print("Which pedestrian classes the robot touches (names come from the scenario spec):")
    display(contacts.assign(cls=contacts["name"].str.replace(r"_\\d+$", "", regex=True))
                    .groupby(["planner", "cls"]).size().unstack(fill_value=0))
""")

code("""
plots.distributions(table, ["min_clearance", "min_ttc", "time_ttc_danger", "collision_time"],
                    labels={"min_clearance": "min clearance [m]", "min_ttc": "min TTC [s]",
                            "time_ttc_danger": "time with TTC < 1 s [s]", "collision_time": "time in contact [s]"});
""")

md("""
### One episode up close

Closest-pedestrian clearance over an episode (black), with the proxemic zones shaded (red = contact, orange =
intimate, yellow = personal, green = social) and the time-to-collision (purple). Each dip is one encounter. We show
the episode with the smallest clearance for each planner.
""")

code("""
fig, axes = plt.subplots(len(episodes), 1, figsize=(12, 3 * len(episodes)))
for ax, (planner, eps) in zip(np.atleast_1d(axes), episodes.items()):
    worst = min(eps, key=lambda e: table.loc[(table.planner == planner) & (table.episode == e.meta["episode"]),
                                             "min_clearance"].iloc[0])
    plots.clearance_timeline(worst, ax=ax)
fig.tight_layout()
""")

md("""
## 6. Comfort: proxemics

Even without contact, passing within ~0.45 m of someone (their *intimate* zone) is uncomfortable, and 0.45–1.2 m
(*personal* zone) is where people expect only acquaintances. Social-navigation papers report time or fraction of
time in these zones. Faster robots spend less time near people simply because they spend less time in total, so
also compare the zone time to the episode duration.
""")

code("""
plots.proxemic_bars(table)
table.groupby("planner")[["time_contact", "time_intimate", "time_personal", "time_social", "duration"]].mean()
""")

md("""
## 7. Freezing

The **freezing robot problem**: in dense crowds, planners often decide every option is unsafe and stop. In
the real world that causes deadlocks, because people expect the robot to move. Here a freeze is ≥ 1 s below
0.05 m/s before the goal. A freeze far from any pedestrian means something else went wrong (e.g. a planning
failure or a wall).
""")

code("""
fr = pd.concat([sm.freezes(ep).assign(planner=p, episode=ep.meta["episode"])
                for p, eps in episodes.items() for ep in eps], ignore_index=True)
print("episodes with a freeze:", table.groupby("planner")["froze"].sum().to_dict())
fr
""")

md("""
## 8. Smoothness: how the motion looks

Jerky, weaving motion is unpleasant to be around and hard for people to predict. DS-RNN's ±1 rad/s "bang-bang"
steering (its turn output always saturates the ±0.1 rad/step clip) should show up as many direction flips per
minute.
""")

code("""
plots.distributions(table, ["mean_abs_accel", "mean_abs_jerk", "heading_per_m", "omega_flips_per_min"],
                    labels={"mean_abs_accel": "mean |accel| [m/s²]", "mean_abs_jerk": "mean |jerk| [m/s³]",
                            "heading_per_m": "turning per metre [rad/m]",
                            "omega_flips_per_min": "turn-direction flips / min"});
""")

md("""
## 9. All paths at once

Every episode's path per planner. Red × = contact, blue ○ = freeze. Because the scenario is identical in every
episode, a tight bundle of paths means the planner behaves consistently. The spread shows how much timing noise
changes its decisions.
""")

code("""
plots.trajectories(episodes);
""")

md("""
## 10. Watch some episodes

Static frames first: the moment of each planner's first contact (follow view, 16 m window around the robot).
Then videos of a **typical** episode (median time to goal) and the **worst** episode (most contacts, then smallest
clearance) per planner. Rendering takes ~0.3 s per frame, so videos are rendered once into `media/` and reused.

Legend in the frames: orange = robot (arrow = heading, line = path so far), numbered circles = pedestrians with
velocity arrows (red when touching the robot), green dashed = global plan, red star = goal.
""")

code("""
for planner, eps in episodes.items():
    c = contacts[contacts.planner == planner]
    if c.empty:
        print(f"{planner}: no contacts")
        continue
    first = c.sort_values(["episode", "t_rel"]).iloc[0]
    ep = next(e for e in eps if e.meta["episode"] == first.episode)
    ax = ep.plot(first.t_rel, view="follow")
    ax.set_title(f"{planner} episode {first.episode}: contact with {first['name']} at t = {first.t_rel:.1f} s")
    plt.show()
""")

code("""
MEDIA = Path("media"); MEDIA.mkdir(exist_ok=True)

def pick(planner):
    t = table[table.planner == planner]
    typical = t.iloc[(t.time_to_goal - t.time_to_goal.median()).abs().argsort().iloc[0]]
    worst = t.sort_values(["collisions", "min_clearance"], ascending=[False, True]).iloc[0]
    return {"typical": int(typical.episode), "worst": int(worst.episode)}

videos = []
for planner, eps in episodes.items():
    for label, i in pick(planner).items():
        ep = next(e for e in eps if e.meta["episode"] == i)
        out = MEDIA / f"{planner}_{label}_ep{i:03d}.mp4"
        if not out.exists():
            ep.save_video(out, speed=8 if planner == "dsrnn" else 16, fps=10)
        videos.append((planner, label, i, out))
for planner, label, i, out in videos:
    print(f"{planner} {label} (episode {i}):")
    display(Video(str(out), embed=False, width=900))
""")

md("""
To watch any episode in the notebook yourself:

```python
ep = episodes["dwb"][3]
ep.play(speed=8)                 # inline video
ep.play(speed=4, view="follow")  # zoomed in on the robot
ep.plot(42.0)                    # one frame, 42 s into the episode
```
""")

md("""
## 11. Takeaways

*(Filled in after looking at the results above. See the end of the notebook run.)*
""")

md("""
## 12. Ideas for the next experiments

- **Randomise the scenario** (different seeds for `generate_scenario.py`, densities 10/20/40) so the intervals
  describe crowds in general rather than one replayed scene.
- **Same speed limit for both planners** (e.g. both at 0.5 or 1.0 m/s) to separate "faster" from "better".
- **Perception ladder**: DS-RNN with limited range / FOV / occlusion / noise vs perfect perception.
- **Pedestrians that react to the robot** (HuNav robot tracking is currently disabled), then pedestrian
  disturbance metrics: how much people slow down or detour because of the robot.
- **More planners**: CrowdNav++, MPPI, SICNav, same notebook.
""")

nb["cells"] = cells
nb["metadata"]["kernelspec"] = {"name": "python3", "display_name": "Python 3", "language": "python"}
nbf.write(nb, "first_experiment.ipynb")
print("wrote first_experiment.ipynb")
