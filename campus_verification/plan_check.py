"""Criterion (c): do the global plans go through the doors/gaps? For every recorded plan that crosses y = 30
(riser -> plaza), is the crossing point inside the scenario's free space (i.e. in the door, not through a panel)?

    uv run campus_verification/plan_check.py campus_verification/data
"""
import json
import sys
from pathlib import Path

import numpy as np
import shapely
import yaml

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "scripts"))
from generate_campus_scenario import Campus, free_space  # noqa: E402
from arena_analysis import load_run  # noqa: E402

campus = Campus(yaml.safe_load((REPO / "worlds/campus/zones.yaml").read_text()))
for root in sys.argv[1:]:
    for rj in sorted(Path(root).rglob("run.json")):
        for ep in load_run(rj.parent).episodes:
            sc = ep.meta.get("scenario", "")
            if not sc.startswith(("door_exit", "gauntlet")) or ep.plan.empty:
                continue
            free = free_space(json.loads((REPO / "worlds/campus/scenarios" / sc).read_text()), campus)
            n = ok = 0
            for _, g in ep.plan.groupby("plan_idx"):
                x, y = g["x"].to_numpy(), g["y"].to_numpy()
                for i in np.where((y[:-1] < 30) & (y[1:] >= 30))[0]:
                    xc = x[i] + (30 - y[i]) / (y[i + 1] - y[i]) * (x[i + 1] - x[i])
                    n += 1
                    ok += bool(shapely.contains_xy(free.buffer(-0.05), xc, 30.0))
            print(f"{sc:20s} {ep.meta['local_planner']:6s} ep{ep.index:3d} map={ep.meta.get('map_loaded', '')[-28:]:28s} "
                  f"plan crossings of y=30: {n:4d}, inside the door: {ok}")
