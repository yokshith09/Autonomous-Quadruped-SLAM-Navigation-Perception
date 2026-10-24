#!/usr/bin/env python3
"""Build runs/<run_id>/REPORT.md (+ map_overlay.png) from the files a run saved.

  python -m evaluation.report runs/<run_id>

Pure measurement report: every number is read from the run's own files (metrics.json, ablation.json,
trajectory.csv, manifest.json, artifacts/). Anything recomputed after the run is listed under
"Post-run corrections" so the raw metrics.json stays the as-run record. Section 7 (external benchmarks) is
intentionally empty until the measurements have been verified.
"""
import hashlib
import json
import os
import sys

import numpy as np

from evaluation.audit import audit
from evaluation.run_telemetry import extra_warnings, overhead_per_step_ms


def fmt(v, d=1, unit=""):
    if isinstance(v, dict) and "reason" in v:
        return f"n/a ({v['reason']})"
    if v is None:
        return "n/a"
    if isinstance(v, bool):
        return "yes" if v else "no"
    if isinstance(v, float):
        return f"{v:.{d}f}{unit}"
    return f"{v}{unit}"


def cm(v):
    return v * 100.0 if isinstance(v, (int, float)) and not isinstance(v, bool) else v


def table(headers, rows):
    out = ["| " + " | ".join(headers) + " |", "|" + "|".join("---" for _ in headers) + "|"]
    out += ["| " + " | ".join(str(c) for c in r) + " |" for r in rows]
    return "\n".join(out) + "\n"


def sha256(path, chunk=1 << 22):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        while block := f.read(chunk):
            h.update(block)
    return h.hexdigest()


def load(path):
    return json.load(open(path, encoding="utf-8")) if os.path.exists(path) else None


def map_overlay(run_dir):
    import cv2
    from grid import BayesianOccupancyGrid
    art = os.path.join(run_dir, "artifacts")
    if not all(os.path.exists(os.path.join(art, n)) for n in ("slam_grid.npy", "true_grid.npy")):
        return None
    S = 3
    g = BayesianOccupancyGrid(-5.5, 5.5, -5.5, 5.5, resolution=0.05)
    r = BayesianOccupancyGrid(-5.5, 5.5, -5.5, 5.5, resolution=0.05)
    g.grid_log_odds = np.load(os.path.join(art, "slam_grid.npy"))
    r.grid_log_odds = np.load(os.path.join(art, "true_grid.npy"))

    def render(gr):
        h, w = gr.grid_log_odds.shape
        im = np.full((h, w, 3), 128, np.uint8)
        im[gr.grid_log_odds <= gr.thresh_free] = 255
        im[gr.grid_log_odds >= gr.thresh_occ] = 0
        return cv2.resize(im, (w * S, h * S), interpolation=cv2.INTER_NEAREST)

    left, right = render(g), render(r)
    traj = np.loadtxt(os.path.join(run_dir, "trajectory.csv"), delimiter=",", skiprows=1)
    for cols, colour in (((1, 2), (255, 140, 0)), ((4, 5), (0, 140, 255))):  # BGR: ground truth blue, estimate orange
        pts = [g.world_to_grid(x, y) for x, y in traj[::10, cols]]
        for a, b in zip(pts, pts[1:]):
            cv2.line(left, (a[0] * S, a[1] * S), (b[0] * S, b[1] * S), colour, 1)
    for x, y, _ in g.detect_frontiers():
        cv2.circle(left, (x * S, y * S), 5, (0, 0, 255), 2)
    for im, text in ((left, "SLAM map | blue: ground-truth path, orange: estimated path, red: remaining frontiers"),
                     (right, "Reference grid built with ground-truth pose")):
        cv2.putText(im, text, (6, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 200), 1)
    cv2.imwrite(os.path.join(run_dir, "map_overlay.png"), np.hstack([left, right]))
    return "map_overlay.png"


def build(run_dir):
    m, man = load(os.path.join(run_dir, "metrics.json")), load(os.path.join(run_dir, "manifest.json"))
    abl = load(os.path.join(run_dir, "ablation.json"))
    kg = load(os.path.join(run_dir, "artifacts", "knowledge_graph.json"))
    o, loc, mp, c, a, q = m["overall"], m["localization"], m["mapping"], m["compute"], m["agents"], m["data_quality"]

    # ---- post-run corrections (raw metrics.json is left untouched)
    corr = []
    if kg is not None and o.get("doorways_detected") != len(kg.get("doorways", {})):
        corr.append(f"doorways_detected: the as-run value ({fmt(o.get('doorways_detected'))}) came from a bus-event counter that never "
                    f"fires (doorways are logged to episodic memory); replaced by {len(kg['doorways'])} from artifacts/knowledge_graph.json")
        o["doorways_detected"] = len(kg["doorways"])
    steps = sum(1 for _ in open(os.path.join(run_dir, "trajectory.csv"))) - 1
    calls = sum(r["calls"] for r in a)
    if "telemetry_overhead_basis" not in c:
        ov = overhead_per_step_ms(calls, steps)
        corr.append(f"telemetry overhead: the as-run value ({fmt(c.get('telemetry_overhead_ms_per_step'), 4)} ms/step) counted only the SLAM "
                    f"bookkeeping; recomputed as a calibrated estimate ({ov:.4f} ms/step = timing-wrapper micro-benchmark x {calls} calls / {steps} steps)")
        c["telemetry_overhead_ms_per_step"] = ov
    new_w = [w for w in extra_warnings(o, loc) if w not in q["warnings"]]
    if new_w:
        corr.append(f"{len(new_w)} data-quality warning(s) added after the run (the live summary now uses the same function)")
        q["warnings"] += new_w

    agent_total = sum(r["total_s"] for r in a) or 1.0
    md = [f"# Baseline report: run {m['run_id']}\n",
          "Every number comes from this run's own files. Measured = computed from telemetry or ground truth; "
          "estimate / diagnostic indicator are labelled as such; n/a = could not be measured (reason shown).\n"]

    # ---- run identification
    rec = os.path.join(run_dir, "recording.npz")
    src = [("raw SLAM input recording", "recording.npz",
            f"{os.path.getsize(rec) / 2**20:.0f} MB, sha256 {sha256(rec)[:16]}..." if os.path.exists(rec) else "not kept")]
    src += [(d, f, "") for d, f in (("per-step ground-truth and estimated pose", "trajectory.csv"), ("agent table", "agents.csv"),
                                    ("all metrics", "metrics.json"), ("run manifest", "manifest.json"),
                                    ("ablation (offline replay)", "ablation.csv"), ("reference/SLAM grids, knowledge graph", "artifacts"))
            if os.path.exists(os.path.join(run_dir, f))]
    md.append("## Run identification\n")
    md.append(table(["Item", "Value"], [
        ("Started", man.get("started")), ("Command", f"`{man.get('command')}`"),
        ("Code", f"git {man['git'].get('commit')}" + (" (+ uncommitted changes at run time)" if man["git"].get("uncommitted_changes") else "")),
        ("CPU / GPU", f"{man.get('cpu')} ({c.get('logical_cores')} threads) / {fmt(c.get('gpu'))}"),
        ("Platform", f"{man.get('platform')}, Python {man.get('python')}"),
        ("Configuration", f"`{json.dumps(man.get('config'))}`"),
        ("Ground truth", man.get("ground_truth")), ("Runs in this report", "1 (no variability can be shown)")]))
    md.append(table(["Source data", "File", "Detail"], src))

    # ---- 1
    md.append("## 1. Overall robot performance: what the robot achieved\n")
    speed = o["distance_travelled_gt_m"] / o["sim_time_s"] if o.get("sim_time_s") else None
    md.append(table(["Metric", "Value", "Meaning"], [
        ("Outcome", o["end_reason"], "why the run ended"),
        ("Task success", fmt(o["task_success"]), "mission reported all reachable frontiers explored"),
        ("Simulated time", fmt(o["sim_time_s"], 0, " s"), "robot time"),
        ("Wall-clock time", f"{fmt(o['wall_time_s'], 0, ' s')} (real-time factor {fmt(o['real_time_factor'], 2)})", "how long the computer needed; < 1 means slower than real time"),
        ("Distance travelled", fmt(o["distance_travelled_gt_m"], 1, " m"), "path length from ground truth"),
        ("Average speed", fmt(speed, 3, " m/s"), "distance / simulated time"),
        ("Rooms completed / discovered", f"{fmt(o['rooms_completed'])} / {fmt(o['rooms_discovered'])}", "rooms the mapper segmented"),
        ("Doorways detected", fmt(o["doorways_detected"]), "knowledge-graph entries; they accumulate as room segmentation changes, so this over-counts real doorways"),
        ("Explored free area", fmt(o["explored_free_area_m2"], 1, " m2"), "free cells x 0.05 m x 0.05 m"),
        ("Grid observed", fmt(o["explored_pct_of_bounding_box"], 1, " %"), "of the 11 x 11 m map box, not of the arena"),
        ("Stuck events / unreachable targets", f"{fmt(o['stuck_events'])} / {fmt(o['unreachable_targets'])}", "watchdog and planner events"),
        ("Human interventions", fmt(o["human_interventions"]), ""), ("Collisions", fmt(o["collisions"]), "")]))

    # ---- 2
    md.append("## 2. Agent and component performance\n")
    md.append("### 2a. Operational performance (how the agents ran, not whether they helped)\n")
    md.append(table(["Agent", "Rate (Hz)", "Calls", "Errors", "p50 (ms)", "p95 (ms)", "max (ms)", "Total (s)", "Share", "Task metric"],
                    [(r["agent"], fmt(r["rate_hz"], 0), r["calls"], r["errors"], fmt(r["p50_ms"], 2), fmt(r["p95_ms"], 2),
                      fmt(r["max_ms"], 0), fmt(r["total_s"], 0), f"{100 * r['total_s'] / agent_total:.0f} %", r["task_metric"]) for r in a]))
    ax = m["agents_extra"]
    md.append(f"Whole control-loop step: p50 {fmt(ax['control_loop_p50_ms'], 0)} ms, p95 {fmt(ax['control_loop_p95_ms'], 0)} ms, "
              f"max {fmt(ax['control_loop_max_ms'], 0)} ms. Agent time is {agent_total:.0f} s of {o['wall_time_s']:.0f} s wall time; "
              f"the rest is simulator stepping, image transfer, HUD and per-step map queries outside `agent.step()` (split not measured).\n")
    md.append("### 2b. Contribution to the system result (offline replay of this run's recording, one component removed)\n")
    if abl:
        rows = abl["rows"]
        md.append(table(["Configuration", "ATE RMSE (cm)", "Final error (cm)", "RPE (%)", "Heading err (deg)", "Loop closures", "Delta ATE vs full (cm)"],
                        [(r["variant"], fmt(r["ate_rmse_cm"], 1), fmt(r["final_err_cm"], 1), fmt(r["rpe_trans_pct"], 1),
                          fmt(r["heading_err_deg"], 2), r["loop_closures"], fmt(r.get("delta_ate_cm_vs_full"), 1)) for r in rows]))
        f0 = rows[0]
        md.append(f"Replay checks: two identical replays differ by at most {f0['determinism_max_diff_m']:.3g} m (determinism); "
                  f"the full replay differs from the live trajectory by at most {fmt(f0.get('replay_vs_live_max_cm'), 2)} cm over "
                  f"{f0.get('replay_steps')} steps. One recording only: differences are not statistically tested, and a component that "
                  f"does not change the ATE here may still matter in other conditions.\n")
    else:
        md.append("n/a (ablation not run: `python -m evaluation.ablate runs/<run_id>`)\n")

    # ---- 3
    md.append("## 3. Localization performance (independent of mapping)\n")
    md.append("Frame: 2D world frame of the simulator scene (x, y in m, yaw in rad). SLAM starts at the true start pose, so no "
              "trajectory alignment is applied. Evaluated at every control step (0.05 s) against simulator ground truth.\n")
    md.append(table(["Metric", "Value", "Meaning"], [
        ("ATE RMSE", fmt(cm(loc["ate_rmse_m"]), 1, " cm"), "typical position error over the whole run"),
        ("ATE median / max", f"{fmt(cm(loc['ate_median_m']), 1)} / {fmt(cm(loc['ate_max_m']), 1)} cm", ""),
        ("Final position error", f"{fmt(cm(loc['final_position_error_m']), 1, ' cm')} = {fmt(loc['final_drift_pct_of_distance'], 2, ' %')} of path", "end-of-run drift"),
        ("RPE translation (1 m segments)", f"{fmt(cm(loc['rpe_trans_rmse_m_per_1m']), 1, ' cm')} RMSE, mean {fmt(loc['rpe_trans_pct'], 1, ' %')} of segment", "local accuracy, independent of accumulated drift"),
        ("RPE rotation", fmt(loc["rpe_rot_rmse_deg_per_m"], 2, " deg/m"), ""),
        ("Heading error mean / max", f"{fmt(loc['heading_err_mean_deg'], 2)} / {fmt(loc['heading_err_max_deg'], 1)} deg", ""),
        ("Error first above 0.5 m / 1 m", f"{fmt(loc['time_error_first_exceeds_0p5m_s'], 0, ' s')} / {fmt(loc['time_error_first_exceeds_1m_s'], 0, ' s')}", "n/a = never reached"),
        ("Scan-match acceptance (diagnostic)", f"{fmt(loc['icp_acceptance_pct'], 1, ' %')} of {loc['tracking_updates']} updates", "see warning in section 6"),
        ("Tracking-loss episodes (diagnostic)", f"{loc['tracking_loss_episodes']} ({fmt(loc['tracking_loss_total_s'], 0, ' s')} total)", ">= 10 consecutive rejected updates"),
        ("Followed by a loop closure within 30 s", fmt(loc["relocalized_by_loop_closure_within_30s"]), "relocalization proxy"),
        ("Loop closures accepted", fmt(loc["loop_closures"]), ""),
        ("Odometry distance vs true", f"{fmt(loc['odometry_distance_m'], 1, ' m')} ({fmt(loc['odometry_scale_error_pct'], 1, ' %')})", "diagnostic only: ground-truth path length also counts body motion the gait did not command, and the odometry-only replay in 2b has a low local error (RPE), so this is not a clean odometry scale error")]))

    # ---- 4
    md.append("## 4. Mapping performance (independent of localization)\n")
    md.append("Reference = occupancy grid built from the same depth stream with the ground-truth pose (not an independent floor plan). "
              "Map size, coverage and accuracy are separate rows.\n")
    md.append(table(["Metric", "Value", "Meaning"], [
        ("Coverage: explored free area", fmt(mp["explored_free_area_m2"], 1, " m2"), "size of what was seen"),
        ("Wall accuracy", fmt(cm(mp["wall_accuracy_m"]), 1, " cm"), "SLAM walls to reference walls (lower is better)"),
        ("Wall completion", fmt(cm(mp["wall_completion_m"]), 1, " cm"), "reference walls to SLAM walls (lower is better)"),
        ("Completion ratio (5 cm)", fmt(mp["completion_ratio_5cm_pct"], 1, " %"), "reference wall cells matched within 5 cm"),
        ("Chamfer / Hausdorff-95", f"{fmt(cm(mp['chamfer_m']), 1)} / {fmt(cm(mp['hausdorff95_m']), 1)} cm", ""),
        ("Precision / recall / F1", f"{fmt(mp['precision_pct'], 1)} / {fmt(mp['recall_pct'], 1)} / {fmt(mp['f1_pct'], 1)} %", f"tolerance {fmt(cm(mp['tolerance_m']), 0)} cm"),
        ("Free-space IoU", fmt(mp["free_space_iou_pct"], 1, " %"), ""),
        ("Occupied cells SLAM / reference", f"{mp['occupied_cells']} / {mp['reference_occupied_cells']}", "more SLAM cells = thicker or smeared walls"),
        ("Map update p50 / p95", f"{fmt(mp['map_update_p50_ms'], 1)} / {fmt(mp['map_update_p95_ms'], 1)} ms", ""),
        ("Map memory", fmt(mp["map_memory_mb"], 2, " MB"), "occupancy grid"),
        ("Frontiers remaining", fmt(mp["frontiers_remaining"]), "unexplored boundaries left (see map_overlay.png)")]))
    if map_overlay(run_dir):
        md.append("![map overlay](map_overlay.png)\n")

    # ---- 5
    md.append("## 5. Computational performance\n")
    md.append(table(["Metric", "Value"], [
        ("Process CPU mean / p95", f"{fmt(c['process_cpu_pct_mean'], 0)} / {fmt(c['process_cpu_pct_p95'], 0)} % (of one core; CoppeliaSim not included)"),
        ("Memory (RSS) mean / peak", f"{fmt(c['rss_mb_mean'], 0)} / {fmt(c['rss_mb_peak'], 0)} MB (includes the in-memory recording buffer)"),
        ("GPU", f"{fmt(c['gpu'])}: {fmt(c['gpu_utilization'])}"),
        ("SLAM tracking p50 / p95", f"{fmt(c['slam_tracking_p50_ms'], 1)} / {fmt(c['slam_tracking_p95_ms'], 1)} ms"),
        ("Largest agent by total time", c["bottleneck_agent"]),
        ("Telemetry overhead (estimate)", f"{fmt(c['telemetry_overhead_ms_per_step'], 4)} ms/step, about {100 * c['telemetry_overhead_ms_per_step'] / ax['control_loop_p50_ms']:.3f} % of the median loop step")]))

    # ---- 6
    md.append("## 6. Data quality and limitations\n")
    md.append("**Warnings**\n\n" + "\n".join(f"- {w}" for w in q["warnings"]) + "\n")
    md.append("**Limitations**\n\n" + "\n".join(f"- {w}" for w in q["limitations"]) + "\n")
    md.append("**Integrity checks** (`audit.py`)\n\n" + table(["Check", "Result", "Detail"], [(n, "pass" if ok else "FAIL", d) for n, ok, d in audit(run_dir)]))
    if corr:
        md.append("**Post-run corrections** (the raw `metrics.json` is unchanged)\n\n" + "\n".join(f"- {x}" for x in corr) + "\n")
    md.append("**Metric definitions**\n\n" + "\n".join(f"- {k}: {v}" for k, v in man["metric_definitions"].items()) + "\n")

    md.append("## 7. External benchmark comparison\n\nNot yet done: added only after these measurements are verified.\n")
    open(os.path.join(run_dir, "REPORT.md"), "w", encoding="utf-8").write("\n".join(md))
    print(f"  [SAVED] {os.path.join(run_dir, 'REPORT.md')}")


if __name__ == "__main__":
    build(sys.argv[1])
