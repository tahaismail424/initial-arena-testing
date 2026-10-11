"""Criterion (b), static part: do the generated scenarios respond to their parameters and vary across seeds?

    uv run campus_verification/params_check.py [--seeds 30] [--fig docs/img/campus_params.png]

For every template, generates seeds 0..N-1 in memory (same code path as the generator) and checks:
  * sampled parameters span their spec ranges;
  * geometry measured from the PANELS matches the parameters: corridor clear width (cross-section of the free space
    at x = 14), door width (at y = 30), riser width (at y = 10);
  * crowd counts per class match the count parameters, desired speeds lie in the speed ranges;
  * different seeds give different pedestrian starts (mean nearest-start distance between consecutive seeds);
  * same seed -> identical JSON (generated twice).
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import yaml
from shapely.geometry import LineString

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "scripts"))
from generate_campus_scenario import Campus, free_space, generate  # noqa: E402


def section(free, x0, y0, x1, y1, ref):
    """Clear width along the segment (x0, y0)-(x1, y1): the length of the free piece that contains the point `ref`
    (block_rect panels are hollow outlines, so the inside of a block is 'free' but unreachable; it must not count)."""
    from shapely.geometry import Point
    cut = free.intersection(LineString([(x0, y0), (x1, y1)]))
    pieces = getattr(cut, "geoms", [cut])
    return next((g.length for g in pieces if g.distance(Point(ref)) < 1e-6), np.nan)


def geometry(scen, campus, params):
    free = free_space(scen, campus)
    p = params.get("door_exit", params)          # gauntlet: nested block
    w = max(1.2, min(p.get("door_width", 5.0), 4.0))
    cx = 32.5 + p.get("door_offset", 0.0)
    cx = min(max(cx, 30.5 + w / 2), 34.5 - w / 2)
    return {"corridor_width_meas": section(free, 14, -0.5, 14, 5.5, (14, 2.5)),
            "door_width_meas": section(free, 29.5, 30.0, 35.5, 30.0, (cx, 30.0)),
            "riser_width_meas": section(free, 29.5, 10.0, 35.5, 10.0, (30.05, 10.0))}


def flat(params, prefix=""):
    out = {}
    for k, v in params.items():
        if isinstance(v, dict):
            out.update(flat(v, f"{prefix}{k}."))
        elif isinstance(v, (int, float)) and not isinstance(v, bool):
            out[prefix + k] = v
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--spec", default=str(REPO / "scenarios/campus/stress.yaml"))
    ap.add_argument("--seeds", type=int, default=30)
    ap.add_argument("--fig", help="save a figure of measured geometry vs parameter")
    a = ap.parse_args()
    spec = yaml.safe_load(Path(a.spec).read_text())
    campus = Campus(yaml.safe_load((REPO / "worlds/campus/zones.yaml").read_text()))
    rows = []
    for tmpl in spec["templates"]:
        prev_starts = None
        for seed in range(a.seeds):
            scen, params = generate(spec, campus, tmpl, seed)
            again, _ = generate(spec, campus, tmpl, seed)
            ag = scen["obstacles"]["dynamic"]
            classes = pd.Series([x["class"] for x in ag]).value_counts().to_dict()
            starts = np.array([x["pos"][:2] for x in ag])
            if prev_starts is not None and len(starts) and len(prev_starts):
                d = np.min(np.hypot(*(starts[:, None, :] - prev_starts[None, :, :]).transpose(2, 0, 1)), axis=1)
                start_shift = float(d.mean())
            else:
                start_shift = np.nan
            prev_starts = starts
            rows.append({"template": tmpl, "seed": seed, **flat(params), **geometry(scen, campus, params),
                         **{f"n_{k}": v for k, v in classes.items()}, "n_total": len(ag),
                         "n_panels": len(scen["obstacles"]["static"]),
                         "v_min": min((x["desired_velocity"] for x in ag), default=np.nan),
                         "v_max": max((x["desired_velocity"] for x in ag), default=np.nan),
                         "start_shift": start_shift,
                         "deterministic": json.dumps(scen, sort_keys=True) == json.dumps(again, sort_keys=True)})
    df = pd.DataFrame(rows)
    pd.set_option("display.width", 250)
    pd.set_option("display.max_columns", 40)
    for tmpl, g in df.groupby("template", sort=False):
        g = g.dropna(axis=1, how="all").drop(columns=["template", "seed"])
        print(f"\n== {tmpl} ({len(g)} seeds): min / max of every column")
        print(g.agg(["min", "max"]).round(2).to_string())
    # geometry vs parameter checks
    cc = df[df.template == "corridor_counterflow"]
    de = df[df.template == "door_exit"]
    bc = df[df.template == "blind_corner"]
    print("\ncorridor width: |measured - width| max %.3f m" % (cc["corridor_width_meas"] - cc["width"]).abs().max())
    print("door width:     |measured - door_width| max %.3f m" % (de["door_width_meas"] - de["door_width"]).abs().max())
    exp_riser = np.where(5.0 - bc["riser_width"] < 0.5, 5.0, bc["riser_width"])
    print("riser width:    |measured - riser_width (5 if >= 4.5)| max %.3f m" % np.abs(bc["riser_width_meas"] - exp_riser).max())
    print("deterministic (same seed -> identical JSON):", bool(df["deterministic"].all()))
    if a.fig:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, axs = plt.subplots(1, 3, figsize=(12, 3.6))
        for ax, (g, p, m, lab) in zip(axs, [(cc, "width", "corridor_width_meas", "corridor width (m)"),
                                            (de, "door_width", "door_width_meas", "door width (m)"),
                                            (bc, "riser_width", "riser_width_meas", "riser width (m)")]):
            ax.plot(g[p], g[m], "o", ms=4)
            lo, hi = g[p].min(), g[p].max()
            ax.plot([lo, hi], [lo, hi], "k--", lw=0.8)
            ax.set_xlabel(f"sampled {p}")
            ax.set_ylabel(f"measured from panels")
            ax.set_title(lab)
        fig.suptitle(f"campus geometry vs parameters, {a.seeds} seeds per template", fontsize=10)
        fig.tight_layout()
        fig.savefig(a.fig, dpi=90)
        print("figure:", a.fig)
    return df


if __name__ == "__main__":
    main()
