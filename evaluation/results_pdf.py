#!/usr/bin/env python3
"""Shareable results PDF: measured values with a one-line definition of what each measures, then the comparison with
published systems. No causes, no classes, no fixes (those are in the diagnostic report).

  python -m evaluation.results_pdf runs/<run_id> [out.pdf]
"""
import os
import sys

from evaluation import comparison as C
from evaluation.baseline_pdf import ROOT, derived, esc, load_run, render, tbl


def n(v, d=1):
    return "n/a" if v is None else f"{v:.{d}f}"


def build_html(d, x):
    m, man = d["m"], d["man"]
    o, loc, mp, c = m["overall"], m["localization"], m["mapping"], m["compute"]
    ag = x["agents"]
    ev = m["data_quality"]["events"]
    cm = lambda v: v * 100.0
    W = [130, 110, 250]
    h = ["<h1>Quadruped SLAM and exploration</h1>"
         f"<p class='note'>Measured results and comparison with published systems | run {esc(m['run_id'])} | simulation</p>"]

    h.append("<h2>Test conditions</h2>")
    h.append(tbl(["Item", "Value"], [
        ("Platform", "CoppeliaSim simulation, multi-room arena (8 rooms, about 10 x 10 m)"),
        ("Robot and sensing", "quadruped; forward RGB-D camera converted to a 2D scan; gait odometry; simulated gyro"),
        ("Algorithm", "2D scan-matching SLAM with loop closure and a 2D occupancy-grid map; CPU only"),
        ("Ground truth", "simulator pose of the robot body (used for scoring only)"),
        ("Computer", f"{man.get('cpu')}"),
        ("Run", f"1 run, {o['sim_time_s']:.0f} s of simulated time, {x['dist']:.1f} m travelled; ended at the time limit")], [110, 380]))

    h.append("<h2>1. Task and exploration</h2>")
    h.append(tbl(["Metric", "Value", "What it measures"], [
        ("Task completed", "no", "Whether the mission reported every reachable frontier explored"),
        ("Simulated time", f"{o['sim_time_s']:.0f} s", "Robot time of the run"),
        ("Distance travelled", f"{x['dist']:.1f} m", "Path length of the robot from ground truth"),
        ("Average speed", f"{x['speed']:.3f} m/s", "Distance travelled divided by simulated time"),
        ("Rooms completed / found", f"{o['rooms_completed']} / {o['rooms_discovered']}", "Rooms marked fully explored / rooms the mapper segmented"),
        ("Explored free area", f"{o['explored_free_area_m2']:.1f} m2", "Floor area observed as free space"),
        ("Map observed", f"{o['explored_pct_of_bounding_box']:.1f} %", "Share of the 11 x 11 m map grid that is no longer unknown"),
        ("Frontiers remaining", f"{mp['frontiers_remaining']}", "Boundaries between known and unknown space left at the end"),
        ("Stuck events", f"{o['stuck_events']}", "Times the anti-stuck watchdog fired"),
        ("Unreachable targets", f"{o['unreachable_targets']}", "Targets the path planner could not reach")], W))

    h.append("<h2>2. Localization (position and heading, against ground truth)</h2>")
    h.append(tbl(["Metric", "Value", "What it measures"], [
        ("ATE RMSE", f"{cm(loc['ate_rmse_m']):.1f} cm", "Root-mean-square position error over every control step (0.05 s), no alignment"),
        ("ATE median / maximum", f"{cm(loc['ate_median_m']):.1f} / {cm(loc['ate_max_m']):.1f} cm", "Median and worst position error"),
        ("ATE as share of path", f"{x['ate_pct_path']:.2f} %", "ATE RMSE divided by distance travelled"),
        ("Final error", f"{cm(loc['final_position_error_m']):.1f} cm ({loc['final_drift_pct_of_distance']:.2f} % of path)", "Position error at the end of the run"),
        ("RPE translation", f"{cm(loc['rpe_trans_rmse_m_per_1m']):.1f} cm per 1 m ({loc['rpe_trans_pct']:.1f} %)", "Error of the motion over each 1 m of true path (local accuracy)"),
        ("RPE rotation", f"{loc['rpe_rot_rmse_deg_per_m']:.2f} deg per m", "Heading error of the motion over each 1 m of true path"),
        ("Heading error mean / maximum", f"{loc['heading_err_mean_deg']:.2f} / {loc['heading_err_max_deg']:.1f} deg", "Orientation error against ground truth"),
        ("Loop closures", f"{loc['loop_closures']}", "Revisits recognized and used to correct drift"),
        ("Scan-match acceptance", f"{loc['icp_acceptance_pct']:.1f} %", "Share of SLAM updates in which the scan match passed its checks (otherwise odometry is used)"),
        ("Tracking-loss episodes", f"{loc['tracking_loss_episodes']} ({loc['tracking_loss_total_s']:.0f} s)", "Runs of 10 or more consecutive updates without an accepted scan match")], W))

    h.append("<h2>3. Mapping (against a reference grid built with the true pose)</h2>")
    h.append(tbl(["Metric", "Value", "What it measures"], [
        ("Wall accuracy", f"{cm(mp['wall_accuracy_m']):.1f} cm", "Mean distance from each mapped wall cell to the nearest reference wall cell"),
        ("Wall completion", f"{cm(mp['wall_completion_m']):.1f} cm", "Mean distance from each reference wall cell to the nearest mapped wall cell"),
        ("Completion ratio (5 cm)", f"{mp['completion_ratio_5cm_pct']:.1f} %", "Share of reference wall cells matched within 5 cm"),
        ("Chamfer / Hausdorff-95", f"{cm(mp['chamfer_m']):.1f} / {cm(mp['hausdorff95_m']):.1f} cm", "Mean and 95th-percentile two-way wall distance"),
        ("Precision / recall / F1", f"{mp['precision_pct']:.1f} / {mp['recall_pct']:.1f} / {mp['f1_pct']:.1f} %",
         f"Wall-cell agreement with the reference within {cm(mp['tolerance_m']):.0f} cm"),
        ("Free-space IoU", f"{mp['free_space_iou_pct']:.1f} %", "Overlap of mapped free space with reference free space"),
        ("Map update time (median / p95)", f"{mp['map_update_p50_ms']:.1f} / {mp['map_update_p95_ms']:.1f} ms", "Time to integrate one depth frame into the map"),
        ("Map memory", f"{mp['map_memory_mb']:.2f} MB", "Size of the occupancy grid (220 x 220 cells of 5 cm)")], W))

    h.append("<h2>4. Processing time and resources</h2>")
    h.append(tbl(["Agent", "Rate (Hz)", "Calls", "Errors", "Median (ms)", "p95 (ms)", "Max (ms)"],
                 [(r["agent"], f"{r['rate_hz']:.0f}", r["calls"], r["errors"], n(r["p50_ms"], 2), n(r["p95_ms"], 2), n(r["max_ms"], 0)) for r in m["agents"]],
                 [130, 50, 55, 45, 65, 60, 60]))
    h.append("<p class='note'>Time of one call to each agent's step function (median, 95th percentile, maximum).</p>")
    h.append(tbl(["Metric", "Value", "What it measures"], [
        ("Control-loop step (median / p95 / max)", f"{m['agents_extra']['control_loop_p50_ms']:.0f} / {m['agents_extra']['control_loop_p95_ms']:.0f} / {m['agents_extra']['control_loop_max_ms']:.0f} ms",
         "Wall time of one whole control step (all agents)"),
        ("Real-time factor", f"{o['real_time_factor']:.2f}", "Simulated seconds per wall-clock second (1 = real time)"),
        ("Wall-clock time", f"{o['wall_time_s']:.0f} s", "Time the computer needed for the run"),
        ("Process CPU (mean / p95)", f"{c['process_cpu_pct_mean']:.0f} / {c['process_cpu_pct_p95']:.0f} % of one core", "CPU use of the Python process; simulator excluded"),
        ("Memory (mean / peak)", f"{c['rss_mb_mean']:.0f} / {c['rss_mb_peak']:.0f} MB", "Resident memory of the Python process"),
        ("GPU", "not used", "No part of the pipeline runs on the GPU")], W))

    ab = d.get("abl")
    if ab:
        rows = ab["rows"]
        h.append("<h2>5. Component ablation (offline replay of the recorded run)</h2>")
        h.append("<p class='note'>The same recorded sensor data is replayed with one component switched off; the live robot is not changed.</p>")
        h.append(tbl(["Configuration", "ATE RMSE (cm)", "Final error (cm)", "Heading error (deg)", "Loop closures"],
                     [(r["variant"] if i == 0 else r["variant"].replace("without ", "Without "), f"{r['ate_rmse_cm']:.1f}", f"{r['final_err_cm']:.1f}",
                       f"{r['heading_err_deg']:.2f}", r["loop_closures"]) for i, r in enumerate(rows[:-1])], [200, 70, 70, 80, 70]))

    h.append("<h1>Comparison with published systems</h1>")
    h.append("<p class='note'>Published values are quoted from the cited papers and product pages; 'derived' means two published numbers were divided. "
             "Test scenes, sensors, ground truth and map representations differ from ours as listed in the Test conditions column.</p>")
    for title, fn in (("6. SLAM as a whole: pose accuracy, map accuracy and cost", C.slam_combined),
                      ("7. Pose accuracy in multi-room scenes", C.apartment),
                      ("8. Pose accuracy on legged robots", C.quadruped_pose),
                      ("9. Processing time per update", C.runtime),
                      ("10. Exploration speed", C.exploration)):
        hh, rr, ww = fn(d, x)
        h.append(f"<h2>{esc(title)}</h2>")
        h.append(tbl(hh, rr, ww, raw_cols=(0,)))
    h.append("<p class='note'>Map accuracy / completion / completion ratio: mean wall-to-wall distances and share of wall cells within 5 cm. "
             "Ours are measured on 2D grid wall cells, published values on 3D reconstructed surfaces. ATE: absolute trajectory error (RMSE). "
             "Path: distance travelled in the test.</p>")
    h.append("<h2>References</h2><p class='note'>" + "<br/>".join(esc(r) for r in C.ref_list()) + "</p>")
    return "".join(h)


def build(run_dir, out=None):
    d = load_run(run_dir)
    x = derived(d)
    out = out or os.path.join(ROOT, "docs", "RESULTS.pdf")
    return render(build_html(d, x), run_dir, out, f"Results, run {d['m']['run_id']}")


if __name__ == "__main__":
    build(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else None)
