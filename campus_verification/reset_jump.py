"""How far each pedestrian moved in the first samples after a reset vs where the scenario starts it.

    uv run campus_verification/reset_jump.py <episode_dir> [<scenario.json>]
"""
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd

REPO = Path(__file__).resolve().parents[1]


def jumps(ep_dir, scen=None):
    ep_dir = Path(ep_dir)
    meta = json.loads((ep_dir / "meta.json").read_text())
    if scen is None:
        p = REPO / "worlds" / "campus" / "scenarios" / meta["scenario"]
        scen = json.loads(p.read_text())
    A = {a["name"]: a for a in scen["obstacles"]["dynamic"]}
    h = pd.read_csv(ep_dir / "humans.csv")
    h["tr"] = h["t"] - meta["t_start"]
    rows = []
    for n, g in h.groupby("name"):
        if n not in A:
            continue
        g = g.sort_values("t")
        p0 = np.array(A[n]["pos"][:2])
        first = g.iloc[0]
        q = g.iloc[(g["tr"] - 1.0).abs().argmin()]
        rows.append({"name": n, "first_t": first["tr"], "first_disp": np.hypot(first.x - p0[0], first.y - p0[1]),
                     "disp_1s": np.hypot(q.x - p0[0], q.y - p0[1]), "speed": A[n]["desired_velocity"]})
    return pd.DataFrame(rows)


if __name__ == "__main__":
    d = jumps(sys.argv[1], json.loads(Path(sys.argv[2]).read_text()) if len(sys.argv) > 2 else None)
    print(d.round(2).to_string(index=False))
    print(f"median first-sample displacement {d.first_disp.median():.2f} m, max {d.first_disp.max():.2f} m; "
          f"implied integration time ~{(d.first_disp / d.speed).median():.1f} s")
