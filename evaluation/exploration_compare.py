"""Exploration comparison of two runs in the same scene: ours vs the nearest-frontier baseline (Yamauchi 1997).

  python -m evaluation.exploration_compare <scene_truth.json> <ours_run> <baseline_run> out.png "title"

Coverage = real floor mapped as free (truth_eval) at the end; coverage curve = explored % over time (progress.csv);
path length, stuck events and collisions from each run's REPORT.md; map scores from truth_eval.json.
"""
import csv
import json
import re
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


def summary(run):
    rep = open(f"{run}/REPORT.md", encoding="utf-8").read()
    grab = lambda pat: (re.search(pat, rep) or [None, None])[1]  # noqa: E731
    t = json.load(open(f"{run}/truth_eval.json"))["map_vs_scene"]
    prog = list(csv.DictReader(open(f"{run}/progress.csv")))
    return {
        "floor": grab(r"Real floor mapped as free \| ([\d.]+) %"),
        "path": grab(r"Distance travelled \| ([\d.]+) m"),
        "stuck": grab(r"Stuck events / unreachable targets \| (\d+)"),
        "coll": grab(r"\| Collisions \| (\d+)"),
        "ate": grab(r"\| ATE RMSE \| ([\d.]+) cm"),
        "f1": round(t["f1_score"] * 100, 1), "recall": round(t["recall"] * 100, 1),
        "curve": ([float(p["t_s"]) for p in prog], [float(p["explored_pct"]) for p in prog]),
    }


def main():
    scene, ours, base, out, title = sys.argv[1:6]
    a, b = summary(ours), summary(base)
    for name, s in (("ours", a), ("nearest-frontier", b)):
        print(f"  {name:17s} floor {s['floor']} % | path {s['path']} m | stuck {s['stuck']} | collisions {s['coll']} | "
              f"ATE {s['ate']} cm | map recall {s['recall']} % | F1 {s['f1']} %")
    fig, ax = plt.subplots(figsize=(7, 4))
    ax.plot(*a["curve"], label="ours")
    ax.plot(*b["curve"], label="nearest-frontier (Yamauchi 1997)")
    ax.set_xlabel("time (s)"); ax.set_ylabel("map window explored (%)"); ax.set_title(title); ax.grid(lw=0.3)
    ax.legend()
    fig.savefig(out, dpi=130, bbox_inches="tight")


if __name__ == "__main__":
    main()
