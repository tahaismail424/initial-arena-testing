# First experiment: DS-RNN vs DWB in the tame hallway

20 recorded episodes per planner, `school_hallway` + `hallway_tame.json`, headless, ground-truth localization.

| File | What |
|---|---|
| `jobs.yaml` | The experiment definition. Run it with `uv run scripts/arena_batch.py first_experiment/jobs.yaml` (from the repo root). |
| `data/` | Recorded runs (one folder per planner, one sub-folder per episode) + `batch_manifest.json` + launch logs. Not in git (~100 MB). |
| `social_metrics.py` | Social-navigation metrics per episode: contacts (who moved into whom), time-to-collision, proxemic zones, freezing, smoothness. Plus Wilson and bootstrap confidence intervals. |
| `plots.py` | Plots (trajectories, distributions, speed profile, contacts, proxemics, clearance timeline) and significance tests. |
| `first_experiment.ipynb` | The analysis. Open with `uv run jupyter lab` from this folder. |
| `make_notebook.py` | Regenerates the notebook skeleton (overwrites it). |
| `media/` | Videos rendered by the notebook (not in git). |
| `batch.log` | Scheduler output of the run. |

Re-run the analysis on new data: re-run the scheduler (new run folders; the notebook picks the newest per planner),
then `uv run jupyter nbconvert --to notebook --execute --inplace first_experiment.ipynb`.
