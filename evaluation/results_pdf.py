#!/usr/bin/env python3
"""Shareable results PDF: measured values (one column per run) with a one-line definition of what each measures, then the
comparison with published systems. No causes, no classes, no fixes (those are in the diagnostic report).

  python -m evaluation.results_pdf runs/<run_id> [runs/<run_id> ...] [--out docs/results/RESULTS.pdf]
"""
import os
import sys

from evaluation import comparison as C
from evaluation.baseline_pdf import ROOT, derived, esc, load_run, render, tbl


def _t(d, *keys, scale=1.0, fmt="{:.1f}"):
    v = d.get("truth")
    for k in keys:
        if v is None:
            return "n/a"
        v = v.get(k)
    return "n/a" if v is None else fmt.format(v * scale)


cm = 100.0
SECTIONS = [
    ("1. Task and exploration", [
        ("Task completed", lambda d, x: "yes" if d["m"]["overall"]["task_success"] else "no", "Whether the mission reported every reachable frontier explored"),
        ("How the run ended", lambda d, x: d["m"]["overall"]["end_reason"], "Stop condition that ended the run"),
        ("Simulated time", lambda d, x: f"{d['m']['overall']['sim_time_s']:.0f} s", "Robot time of the run"),
        ("Distance travelled", lambda d, x: f"{x['dist']:.1f} m", "Path length of the robot from ground truth"),
        ("Average speed", lambda d, x: f"{x['speed']:.3f} m/s", "Distance travelled divided by simulated time"),
        ("Rooms completed / found", lambda d, x: f"{d['m']['overall']['rooms_completed']} / {d['m']['overall']['rooms_discovered']}", "Rooms marked fully explored / rooms the mapper segmented"),
        ("Real floor mapped as free", lambda d, x: _t(d, "floor_mapped_as_free_pct") + " %", "Share of the real floor area of the arena marked as free space in the map"),
        ("Frontiers remaining", lambda d, x: f"{d['m']['mapping']['frontiers_remaining']}", "Boundaries between known and unknown space left at the end"),
        ("Stuck events", lambda d, x: f"{d['m']['overall']['stuck_events']}", "Times the anti-stuck watchdog fired"),
        ("Unreachable targets", lambda d, x: f"{d['m']['overall']['unreachable_targets']}", "Targets the path planner could not reach"),
    ]),
    ("2. SLAM performance summary (localization and mapping together)", [
        ("Pose error, ATE RMSE", lambda d, x: f"{d['m']['localization']['ate_rmse_m'] * cm:.1f} cm", "Root-mean-square position error over the run"),
        ("Local pose error, RPE", lambda d, x: f"{d['m']['localization']['rpe_trans_pct']:.1f} % / {d['m']['localization']['rpe_rot_rmse_deg_per_m']:.2f} deg/m", "Translation and heading error of the motion over each 1 m of path"),
        ("Map accuracy vs real scene", lambda d, x: _t(d, "map_vs_scene", "accuracy_meters", scale=cm) + " cm", "Mean distance from mapped obstacle cells to the nearest real wall or object"),
        ("Map completion vs real scene", lambda d, x: _t(d, "map_vs_scene", "completion_meters", scale=cm) + " cm", "Mean distance from real wall and object cells to the nearest mapped obstacle cell"),
        ("Map F1 vs real scene", lambda d, x: _t(d, "map_vs_scene", "f1_score", scale=cm) + " %", "Agreement of mapped and real obstacle cells within 15 cm"),
        ("SLAM update time (median)", lambda d, x: f"{x['agents']['SLAMAgent']['p50_ms']:.1f} ms", "Time of one pose update"),
        ("Map update time (median)", lambda d, x: f"{x['agents']['MappingAgent']['p50_ms']:.1f} ms", "Time to integrate one depth frame into the map"),
        ("Map memory", lambda d, x: f"{d['m']['mapping']['map_memory_mb']:.2f} MB", "Size of the occupancy grid"),
    ]),
    ("3. Localization (against ground truth)", [
        ("ATE RMSE", lambda d, x: f"{d['m']['localization']['ate_rmse_m'] * cm:.1f} cm", "Root-mean-square position error over every control step (0.05 s), no alignment"),
        ("ATE median / maximum", lambda d, x: f"{d['m']['localization']['ate_median_m'] * cm:.1f} / {d['m']['localization']['ate_max_m'] * cm:.1f} cm", "Median and worst position error"),
        ("ATE as share of path", lambda d, x: f"{x['ate_pct_path']:.2f} %", "ATE RMSE divided by distance travelled"),
        ("Final error", lambda d, x: f"{d['m']['localization']['final_position_error_m'] * cm:.1f} cm", "Position error at the end of the run"),
        ("RPE translation", lambda d, x: f"{d['m']['localization']['rpe_trans_rmse_m_per_1m'] * cm:.1f} cm per 1 m", "Error of the motion over each 1 m of true path"),
        ("RPE rotation", lambda d, x: f"{d['m']['localization']['rpe_rot_rmse_deg_per_m']:.2f} deg per m", "Heading error of the motion over each 1 m of true path"),
        ("Heading error mean / maximum", lambda d, x: f"{d['m']['localization']['heading_err_mean_deg']:.2f} / {d['m']['localization']['heading_err_max_deg']:.1f} deg", "Orientation error against ground truth"),
        ("Loop closures", lambda d, x: f"{d['m']['localization']['loop_closures']}", "Revisits recognized and used to correct drift"),
        ("Scan-match acceptance", lambda d, x: f"{d['m']['localization']['icp_acceptance_pct']:.1f} %", "Share of SLAM updates in which the scan match passed its checks"),
    ]),
    ("4. Mapping (against the real scene and against a map built with the true pose)", [
        ("Accuracy vs real scene", lambda d, x: _t(d, "map_vs_scene", "accuracy_meters", scale=cm) + " cm", "Mapped obstacle cells to the nearest real wall or object (mean)"),
        ("Completion vs real scene", lambda d, x: _t(d, "map_vs_scene", "completion_meters", scale=cm) + " cm", "Real wall and object cells to the nearest mapped obstacle cell (mean)"),
        ("Precision / recall vs real scene", lambda d, x: _t(d, "map_vs_scene", "precision", scale=cm) + " / " + _t(d, "map_vs_scene", "recall", scale=cm) + " %", "Mapped cells near a real obstacle / real obstacle cells found, within 15 cm"),
        ("Free space mapped outside the arena", lambda d, x: _t(d, "free_mapped_outside_arena_m2", fmt="{:.2f}") + " m2", "Area marked free beyond the outer wall"),
        ("Accuracy vs true-pose map", lambda d, x: f"{d['m']['mapping']['wall_accuracy_m'] * cm:.1f} cm", "Same as above, against a map built from the same sensor with the true pose"),
        ("Completion ratio vs true-pose map", lambda d, x: f"{d['m']['mapping']['completion_ratio_5cm_pct']:.1f} %", "Share of true-pose map wall cells matched within 5 cm"),
        ("F1 vs true-pose map", lambda d, x: f"{d['m']['mapping']['f1_pct']:.1f} %", "Wall-cell agreement with the true-pose map within 15 cm"),
        ("Free-space IoU vs true-pose map", lambda d, x: f"{d['m']['mapping']['free_space_iou_pct']:.1f} %", "Overlap of mapped free space with true-pose free space"),
    ]),
    ("5. Object detection (against the real objects in the scene)", [
        ("Real objects in the scene", lambda d, x: _t(d, "detection", "real_objects", fmt="{:.0f}"), "Objects within the robot's sensing height (0.04-0.60 m)"),
        ("Detections reported", lambda d, x: _t(d, "detection", "detections", fmt="{:.0f}"), "Object entities the robot added to its knowledge graph"),
        ("Detections on a real object", lambda d, x: _t(d, "detection", "detection_precision_pct") + " %", "Share of detections within 0.5 m of a real object (precision)"),
        ("Real objects with a detection nearby", lambda d, x: _t(d, "detection", "objects_found", fmt="{:.0f}") + " / " + _t(d, "detection", "real_objects", fmt="{:.0f}"), "Real objects with any detection within 0.5 m"),
        ("'Chair' labels / real chairs", lambda d, x: _t(d, "detection", "labelled_chair", fmt="{:.0f}") + " / " + _t(d, "detection", "real_chairs", fmt="{:.0f}"), "Detections labelled chair / chairs in the scene"),
        ("'Chair' labels on the real chair", lambda d, x: _t(d, "detection", "chair_labels_on_the_real_chair", fmt="{:.0f}"), "Chair-labelled detections within 0.5 m of the real chair"),
    ]),
    ("6. Processing time and resources", [
        ("Control-loop step (median / p95 / max)", lambda d, x: f"{d['m']['agents_extra']['control_loop_p50_ms']:.0f} / {d['m']['agents_extra']['control_loop_p95_ms']:.0f} / {d['m']['agents_extra']['control_loop_max_ms']:.0f} ms", "Wall time of one whole control step (all agents)"),
        ("Real-time factor", lambda d, x: f"{d['m']['overall']['real_time_factor']:.2f}", "Simulated seconds per wall-clock second (1 = real time)"),
        ("Wall-clock time", lambda d, x: f"{d['m']['overall']['wall_time_s']:.0f} s", "Time the computer needed for the run"),
        ("Process CPU (mean)", lambda d, x: f"{d['m']['compute']['process_cpu_pct_mean']:.0f} % of one core", "CPU use of the Python process; simulator excluded"),
        ("Memory (mean / peak)", lambda d, x: f"{d['m']['compute']['rss_mb_mean']:.0f} / {d['m']['compute']['rss_mb_peak']:.0f} MB", "Resident memory of the Python process"),
        ("GPU", lambda d, x: "not used", "No part of the pipeline runs on the GPU"),
    ]),
]


def build_html(runs):
    labels = [lab for lab, _, _ in runs]
    d0 = runs[0][1]
    h = ["<h1>Quadruped SLAM and exploration</h1>"
         f"<p class='note'>Measured results and comparison with published systems | {len(runs)} run(s) | simulation</p>"]
    h.append("<h2>Test conditions</h2>")
    h.append(tbl(["Item", "Value"], [
        ("Platform", "CoppeliaSim simulation, multi-room arena (10 x 10 m outer walls, 15 inner wall segments, 13 objects)"),
        ("Robot and sensing", "quadruped; forward RGB-D camera converted to a 2D scan; gait odometry; simulated gyro"),
        ("Algorithm", "2D scan-matching SLAM with loop closure and a 2D occupancy-grid map; CPU only"),
        ("Ground truth", "simulator pose of the robot body; wall and object geometry read from the scene (scoring only)"),
        ("Computer", f"{d0['man'].get('cpu')}")] +
        [(lab, f"{d['m']['run_id']}: safety limit {d['man']['config'].get('max_time_s'):.0f} s, ran {d['m']['overall']['sim_time_s']:.0f} s, "
               f"{d['m']['overall']['end_reason']}") for lab, d, _ in runs], [110, 380]))

    W = [130] + [max(60, 230 // len(runs))] * len(runs) + [200]
    for title, rows in SECTIONS:
        h.append(f"<h2>{esc(title)}</h2>")
        h.append(tbl(["Metric"] + labels + ["What it measures"], [(m,) + tuple(f(d, x) for _, d, x in runs) + (dfn,) for m, f, dfn in rows], W))
        if title.startswith("6."):
            agents = [a["agent"] for a in d0["m"]["agents"]]
            h.append(tbl(["Agent (median / p95 / max, ms)"] + labels,
                         [(a,) + tuple(f"{x['agents'][a]['p50_ms']:.2f} / {x['agents'][a]['p95_ms']:.2f} / {x['agents'][a]['max_ms']:.0f}" for _, _, x in runs)
                          for a in agents], [150] + [140] * len(runs)))
            h.append("<p class='note'>Time of one call to each agent's step function.</p>")

    abl = [(lab, d) for lab, d, _ in runs if d.get("abl")]
    if abl:
        h.append("<h2>7. Component ablation (offline replay of each recorded run)</h2>")
        h.append("<p class='note'>The same recorded sensor data is replayed with one component switched off; the live robot is not changed. "
                 "Values: ATE RMSE (cm) / heading error (deg).</p>")
        names = [r["variant"] for r in abl[0][1]["abl"]["rows"][:-1]]
        h.append(tbl(["Configuration"] + [lab for lab, _ in abl],
                     [(nm,) + tuple(next((f"{r['ate_rmse_cm']:.1f} / {r['heading_err_deg']:.2f}" for r in d["abl"]["rows"] if r["variant"] == nm), "n/a")
                                    for _, d in abl) for nm in names], [220] + [120] * len(abl)))

    h.append("<h1>Comparison with published systems</h1>")
    h.append("<p class='note'>Published values are quoted from the cited papers and product pages; 'derived' means two published numbers were divided. "
             "Test scenes, sensors, ground truth and map representations differ from ours as listed in the Test conditions column.</p>")
    for title, fn in (("8. SLAM as a whole: pose accuracy, map accuracy and cost", C.slam_combined),
                      ("9. Pose accuracy in multi-room scenes", C.apartment),
                      ("10. Pose accuracy on legged robots", C.quadruped_pose),
                      ("11. Processing time per update", C.runtime),
                      ("12. Exploration speed", C.exploration),
                      ("13. Market systems: capabilities (from manufacturer pages; not measured by us)", C.market)):
        hh, rr, ww = fn(runs)
        h.append(f"<h2>{esc(title)}</h2>")
        h.append(tbl(hh, rr, ww, raw_cols=(0,)))
    h.append("<h2>14. Like-for-like reference: TUM RGB-D Pioneer robot sequences, ATE RMSE (m)</h2>")
    h.append("<p class='note'>Public dataset: Kinect on a wheeled Pioneer 3 robot looking forward, planar motion, motion-capture ground truth. "
             "Published results [12]; our system has not yet been run on these sequences.</p>")
    hh, rr, ww = C.tum_pioneer(runs)
    h.append(tbl(hh, rr, ww, raw_cols=(0,)))
    h.append("<p class='note'>Map accuracy / completion / completion ratio: mean obstacle-to-obstacle distances and share within 5 cm. Ours are measured "
             "on 2D grid cells against the real scene geometry, published values on 3D reconstructed surfaces. ATE: absolute trajectory error (RMSE).</p>")
    h.append("<h2>References</h2><p class='note'>" + "<br/>".join(esc(r) for r in C.ref_list()) + "</p>")
    return "".join(h)


def build(run_dirs, out=None):
    runs = []
    for i, rd in enumerate(run_dirs):
        d = load_run(rd)
        runs.append((f"Run {i + 1}", d, derived(d)))
    out = out or os.path.join(ROOT, "docs", "results", "RESULTS.pdf")
    return render(build_html(runs), run_dirs[0], out, "Results, runs " + ", ".join(d["m"]["run_id"] for _, d, _ in runs))


if __name__ == "__main__":
    args = sys.argv[1:]
    out = None
    if "--out" in args:
        i = args.index("--out")
        out = args[i + 1]
        args = args[:i] + args[i + 2:]
    build(args, out)
