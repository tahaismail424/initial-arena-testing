"""Plots and statistical tests for comparing planners (first experiment). Used by first_experiment.ipynb."""
from __future__ import annotations

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy import stats

import social_metrics as sm

COLORS = {"dsrnn": "tab:orange", "dwb": "tab:blue"}


def color(planner: str) -> str:
    return COLORS.get(planner, "tab:gray")


def draw_walls(ax, world, **kw):
    """Hallway outline from walls.yaml (world.walls: list of ((x0, y0), (x1, y1)) segments)."""
    for (x0, y0), (x1, y1) in world.walls:
        ax.plot([x0, x1], [y0, y1], color=kw.get("color", "black"), lw=kw.get("lw", 1.5))
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")


def trajectories(episodes_by_planner: dict, figsize=(14, 3.6)):
    """All robot paths per planner on the hallway map. Red x = contact with a pedestrian, blue o = freeze."""
    fig, axes = plt.subplots(len(episodes_by_planner), 1, figsize=(figsize[0], figsize[1] * len(episodes_by_planner)),
                             squeeze=False)
    for ax, (planner, eps) in zip(axes[:, 0], episodes_by_planner.items()):
        draw_walls(ax, eps[0].world)
        for ep in eps:
            ax.plot(ep.robot["x"], ep.robot["y"], color=color(planner), lw=0.8, alpha=0.5)
            c = sm.contact_events(ep)
            ax.plot(c["x"], c["y"], "x", color="red", ms=8, mew=2)
            f = sm.freezes(ep)
            ax.plot(f["x"], f["y"], "o", color="tab:blue", mfc="none", ms=8)
        ax.plot(*eps[0].start[:2], "o", color="gray")
        ax.plot(*eps[0].goal[:2], "*", color="red", ms=14)
        ax.set_title(f"{planner}: {len(eps)} episodes   (x = contact, o = freeze)")
    fig.tight_layout()
    return fig


def distributions(table: pd.DataFrame, columns: list[str], labels: dict | None = None, ncols: int = 4):
    """Box plot + every episode as a dot, one panel per metric, planners side by side."""
    labels = labels or {}
    planners = sorted(table["planner"].unique())
    nrows = int(np.ceil(len(columns) / ncols))
    fig, axes = plt.subplots(nrows, ncols, figsize=(3.6 * ncols, 3.2 * nrows), squeeze=False)
    rng = np.random.default_rng(0)
    for ax, col in zip(axes.flat, columns):
        data = [table.loc[table["planner"] == p, col].replace([np.inf, -np.inf], np.nan).dropna() for p in planners]
        ax.boxplot(data, widths=0.5, showfliers=False)
        for i, (p, d) in enumerate(zip(planners, data), start=1):
            ax.scatter(i + rng.uniform(-0.12, 0.12, len(d)), d, s=14, color=color(p), alpha=0.8, zorder=3)
        ax.set_xticks(range(1, len(planners) + 1), planners)
        ax.set_title(labels.get(col, col), fontsize=10)
        ax.grid(axis="y", alpha=0.3)
    for ax in list(axes.flat)[len(columns):]:
        ax.axis("off")
    fig.tight_layout()
    return fig


def speed_profile(episodes_by_planner: dict, bin_m: float = 1.0):
    """Robot speed along the hallway: median (line) and inter-quartile range (band) over episodes, per x bin."""
    fig, ax = plt.subplots(figsize=(12, 3.2))
    for planner, eps in episodes_by_planner.items():
        r = pd.concat([e.robot[["x", "speed"]] for e in eps])
        bins = np.arange(np.floor(r["x"].min()), np.ceil(r["x"].max()) + bin_m, bin_m)
        g = r.groupby(pd.cut(r["x"], bins), observed=True)["speed"]
        mid = np.array([iv.mid for iv in g.median().index])
        ax.plot(mid, g.median().to_numpy(), color=color(planner), label=planner)
        ax.fill_between(mid, g.quantile(0.25).to_numpy(), g.quantile(0.75).to_numpy(), color=color(planner),
                        alpha=0.25)
    ax.set_xlabel("x along the hallway [m]")
    ax.set_ylabel("speed [m/s]")
    ax.legend()
    ax.grid(alpha=0.3)
    fig.tight_layout()
    return fig


def contacts_overview(contacts: pd.DataFrame, hallway_length: float = 60.0):
    """Where (x along the hallway) and when (s into the episode) contacts happened, coloured by planner;
    filled = robot drove into the person, hollow = the person walked into a (near-)stationary robot."""
    fig, ax = plt.subplots(figsize=(12, 3.2))
    for planner, g in contacts.groupby("planner"):
        active = g["robot_active"].astype(bool)
        act, pas = g[active], g[~active]
        ax.scatter(act["x"], act["t_rel"], color=color(planner), s=40, label=f"{planner} (robot moving in)")
        ax.scatter(pas["x"], pas["t_rel"], facecolor="none", edgecolor=color(planner), s=40,
                   label=f"{planner} (robot hit)")
    ax.set_xlim(0, hallway_length)
    ax.set_xlabel("x along the hallway [m]")
    ax.set_ylabel("time into episode [s]")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)
    fig.tight_layout()
    return fig


def proxemic_bars(table: pd.DataFrame):
    """Average seconds per episode with the closest pedestrian in each proxemic zone."""
    zones = ["time_contact", "time_intimate", "time_personal", "time_social"]
    means = table.groupby("planner")[zones].mean()
    fig, ax = plt.subplots(figsize=(7, 3))
    left = np.zeros(len(means))
    shades = ["red", "orange", "gold", "lightgreen"]
    for z, c in zip(zones, shades):
        ax.barh(means.index, means[z], left=left, color=c, label=z.replace("time_", ""))
        left += means[z].to_numpy()
    ax.set_xlabel("seconds per episode (closest pedestrian in zone)")
    ax.legend(ncols=4, fontsize=8, loc="lower right")
    fig.tight_layout()
    return fig


def clearance_timeline(ep, ax=None):
    """Closest-pedestrian clearance and min TTC over one episode (the zone thresholds are shaded)."""
    if ax is None:
        _, ax = plt.subplots(figsize=(12, 3))
    d = ep.distances()
    closest = d.groupby("t")["clearance"].min()
    t = closest.index - ep.meta["t_start"]
    ax.plot(t, closest.to_numpy(), color="black", lw=1, label="closest clearance [m]")
    for (name, lo, hi), c in zip(sm.ZONES, ["red", "orange", "gold", "lightgreen"]):
        ax.axhspan(max(lo, -0.5), hi, color=c, alpha=0.2)
    ttc = sm.min_ttc(ep).clip(upper=10)
    ax2 = ax.twinx()
    ax2.plot(ttc.index, ttc.to_numpy(), color="tab:purple", lw=0.8, alpha=0.7)
    ax2.set_ylabel("min TTC [s] (capped at 10)", color="tab:purple")
    ax.set_ylim(-0.3, 4)
    ax.set_xlabel("time into episode [s]")
    ax.set_ylabel("clearance [m]")
    ax.set_title(f"{ep.meta['local_planner']} episode {ep.meta['episode']}: closest pedestrian")
    return ax


def significance(table: pd.DataFrame, a: str, b: str) -> pd.DataFrame:
    """Is the difference between planners a and b bigger than episode-to-episode noise?
    Rates: Fisher's exact test (2x2 table). Continuous metrics: Mann-Whitney U (no normality assumption).
    p < 0.05 is the usual (arbitrary) threshold; with ~18 metrics, expect ~1 false positive by chance."""
    A, B = table[table["planner"] == a], table[table["planner"] == b]
    rows = []
    for k in sm.RATES:
        if k in table:
            ka, kb = int(A[k].sum()), int(B[k].sum())
            _, p = stats.fisher_exact([[ka, len(A) - ka], [kb, len(B) - kb]])
            rows.append({"metric": f"{k} rate", a: f"{ka}/{len(A)}", b: f"{kb}/{len(B)}", "test": "Fisher", "p": p})
    for k in sm.MEANS:
        if k in table:
            xa = A[k].replace([np.inf, -np.inf], np.nan).dropna()
            xb = B[k].replace([np.inf, -np.inf], np.nan).dropna()
            if len(xa) > 1 and len(xb) > 1 and not (xa.nunique() == 1 and xb.nunique() == 1 and xa.iloc[0] == xb.iloc[0]):
                p = stats.mannwhitneyu(xa, xb).pvalue
            else:
                p = np.nan
            rows.append({"metric": k, a: f"{xa.median():.2f}" if len(xa) else "n/a",
                         b: f"{xb.median():.2f}" if len(xb) else "n/a", "test": "Mann-Whitney (medians)", "p": p})
    out = pd.DataFrame(rows)
    out["significant"] = out["p"] < 0.05
    return out
