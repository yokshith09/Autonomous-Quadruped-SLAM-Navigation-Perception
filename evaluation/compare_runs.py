#!/usr/bin/env python3
"""Compare a baseline run with one or more new runs, metric by metric.

  python -m evaluation.compare_runs <baseline_run> <new_run> [<new_run> ...] [--out file.md]

With several new runs the "New" column is the mean (with the spread, n = number of runs) so a change is judged on
repeated runs, not one. The verdict says Better, Worse or Same (within a tolerance) using the direction that is good for
each metric; metrics without a direction are shown for information. Missing values are shown as n/a, never as zero.
"""
import math
import sys

import numpy as np

from evaluation.baseline_pdf import derived, load_run

SAME_TOLERANCE = 0.03  # a relative change smaller than this is "Same"


def verdict(base, new, direction, tol=SAME_TOLERANCE):
    """Better / Worse / Same / '-' for a baseline value and a new value. direction: 'lower', 'higher' or None."""
    if direction is None or base is None or new is None:
        return "-"
    if isinstance(base, bool) or isinstance(new, bool):
        base, new = float(base), float(new)
    scale = max(abs(base), abs(new), 1e-9)
    if abs(new - base) <= tol * scale:
        return "Same"
    improved = new < base if direction == "lower" else new > base
    return "Better" if improved else "Worse"


def num(v):
    """A plain number or None (the run files use {'value': None, 'reason': ...} for unavailable values)."""
    if isinstance(v, bool):
        return float(v)
    if isinstance(v, (int, float)) and math.isfinite(v):
        return float(v)
    return None


def metrics(d):
    """[(group, name, value, unit, decimals, direction)] for one loaded run."""
    x = derived(d)
    m, t = d["m"], d.get("truth")
    o, loc, mp, c, ag = m["overall"], m["localization"], m["mapping"], m["compute"], x["agents"]
    ev = m["data_quality"]["events"]
    tv = lambda *k: _dig(t, *k)
    cm = 100.0
    rows = [
        ("Exploration", "Task completed (1 = yes)", o["task_success"], "", 0, "higher"),
        ("Exploration", "Rooms completed", o["rooms_completed"], "", 0, "higher"),
        ("Exploration", "Rooms found", o["rooms_discovered"], "", 0, None),
        ("Exploration", "Simulated time", o["sim_time_s"], "s", 0, None),
        ("Exploration", "Distance travelled", x["dist"], "m", 1, None),
        ("Exploration", "Average speed", x["speed"], "m/s", 3, "higher"),
        ("Exploration", "Real floor mapped", tv("floor_mapped_as_free_pct"), "%", 1, "higher"),
        ("Exploration", "Free space mapped outside the arena", tv("free_mapped_outside_arena_m2"), "m2", 2, "lower"),
        ("Exploration", "Frontiers remaining", mp["frontiers_remaining"], "", 0, "lower"),
        ("Exploration", "Stuck events", o["stuck_events"], "", 0, "lower"),
        ("Exploration", "Unreachable targets", o["unreachable_targets"], "", 0, "lower"),
        ("Exploration", "Room entries per room", x["room_entries_per_room"], "", 2, "lower"),
        ("Exploration", "Path walked after coverage plateau", x.get("waste_pct"), "%", 0, "lower"),
        ("Localization", "ATE RMSE", loc["ate_rmse_m"] * cm, "cm", 1, "lower"),
        ("Localization", "ATE as share of path", x["ate_pct_path"], "%", 2, "lower"),
        ("Localization", "Final drift", loc["final_drift_pct_of_distance"], "% of path", 2, "lower"),
        ("Localization", "Largest position error", loc["ate_max_m"] * cm, "cm", 1, "lower"),
        ("Localization", "RPE translation", loc["rpe_trans_pct"], "%", 1, "lower"),
        ("Localization", "RPE rotation", loc["rpe_rot_rmse_deg_per_m"], "deg/m", 2, "lower"),
        ("Localization", "Heading error (mean)", loc["heading_err_mean_deg"], "deg", 2, "lower"),
        ("Localization", "Loop closures", loc["loop_closures"], "", 0, None),
        ("Localization", "Scan-match acceptance", loc["icp_acceptance_pct"], "%", 1, "higher"),
        ("Mapping vs real scene", "Map accuracy", _s(tv("map_vs_scene", "accuracy_meters"), cm), "cm", 1, "lower"),
        ("Mapping vs real scene", "Map completion", _s(tv("map_vs_scene", "completion_meters"), cm), "cm", 1, "lower"),
        ("Mapping vs real scene", "Precision", _s(tv("map_vs_scene", "precision"), cm), "%", 1, "higher"),
        ("Mapping vs real scene", "Recall", _s(tv("map_vs_scene", "recall"), cm), "%", 1, "higher"),
        ("Mapping vs real scene", "F1", _s(tv("map_vs_scene", "f1_score"), cm), "%", 1, "higher"),
        ("Detection", "Detections on a real object", tv("detection", "detection_precision_pct"), "%", 1, "higher"),
        ("Processing time", "SLAM update (median)", ag["SLAMAgent"]["p50_ms"], "ms", 1, "lower"),
        ("Processing time", "Map update (median)", ag["MappingAgent"]["p50_ms"], "ms", 1, "lower"),
        ("Processing time", "Frontier selection (median)", ag["FrontierAgent"]["p50_ms"], "ms", 0, "lower"),
        ("Processing time", "Planning (p95)", ag["PlanningAgent"]["p95_ms"], "ms", 0, "lower"),
        ("Processing time", "Slowest single agent call", max(a["max_ms"] for a in m["agents"]), "ms", 0, "lower"),
        ("Processing time", "Control step (p95)", x["control_p95"], "ms", 0, "lower"),
        ("Processing time", "Real-time factor", o["real_time_factor"], "", 2, "higher"),
        ("Resources", "Process CPU (mean)", c["process_cpu_pct_mean"], "% of a core", 0, None),
        ("Resources", "Memory (peak)", c["rss_mb_peak"], "MB", 0, "lower"),
    ]
    return [(g, n, num(v), u, dec, dr) for g, n, v, u, dec, dr in rows]


def _dig(d, *keys):
    for k in keys:
        if d is None:
            return None
        d = d.get(k) if isinstance(d, dict) else None
    return d


def _s(v, scale):
    return None if num(v) is None else v * scale


def fmt(v, dec):
    return "n/a" if v is None else f"{v:.{dec}f}"


def compare(base_dir, new_dirs):
    """Returns (markdown text, rows) where rows hold the numbers behind the table."""
    base = metrics(load_run(base_dir))
    news = [metrics(load_run(n)) for n in new_dirs]
    n_new = len(news)
    out = [f"| Metric | Baseline | New ({n_new} run{'s' if n_new > 1 else ''}) | Change | Verdict |", "|---|---|---|---|---|"]
    rows, group, tally = [], None, {"Better": 0, "Worse": 0, "Same": 0}
    for i, (g, name, bv, unit, dec, direction) in enumerate(base):
        vals = [r[i][2] for r in news if r[i][2] is not None]
        nv = float(np.mean(vals)) if vals else None
        spread = f" ± {np.std(vals):.{dec}f}" if len(vals) > 1 else ""
        part = f" ({len(vals)}/{n_new} runs)" if vals and len(vals) < n_new else ""
        if g != group:
            out.append(f"| **{g}** | | | | |")
            group = g
        ch = "-" if bv is None or nv is None else f"{nv - bv:+.{dec}f}" + (f" ({100 * (nv - bv) / abs(bv):+.0f}%)" if abs(bv) > 1e-9 else "")
        v = verdict(bv, nv, direction)
        if v in tally:
            tally[v] += 1
        u = f" {unit}" if unit else ""
        out.append(f"| {name} | {fmt(bv, dec)}{u} | {fmt(nv, dec)}{spread}{u}{part} | {ch} | {v} |")
        rows.append((g, name, bv, nv, v))
    out.append("")
    out.append(f"**Summary:** {tally['Better']} better, {tally['Worse']} worse, {tally['Same']} within {int(SAME_TOLERANCE * 100)}% "
               f"(of {sum(1 for r in rows if r[4] != '-')} metrics with a direction). "
               f"Baseline: {base_dir}; new: {', '.join(new_dirs)}. " + ("Single new run: no variability can be stated." if n_new == 1 else
                                                                      f"Mean of {n_new} runs, spread = standard deviation."))
    return "\n".join(out), rows


def main(argv):
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass
    out_path = None
    if "--out" in argv:
        i = argv.index("--out")
        out_path = argv[i + 1]
        argv = argv[:i] + argv[i + 2:]
    if len(argv) < 2:
        print(__doc__)
        return 1
    text, _ = compare(argv[0], argv[1:])
    print(text)
    if out_path:
        open(out_path, "w", encoding="utf-8").write(text + "\n")
        print(f"\n  [SAVED] {out_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
