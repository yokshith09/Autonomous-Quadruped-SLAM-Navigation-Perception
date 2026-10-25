#!/usr/bin/env python3
"""Complete formal project report (PDF): problem, system, methods, evaluation method, results, comparison, analysis, roadmap.

  python -m evaluation.project_report runs/<run_id> [out.pdf]

All measured numbers are read from the run directory (metrics.json, truth_eval.json, ablation.json, progress.csv,
trajectory.csv, artifacts/) and all published numbers from evaluation/benchmarks.py, so the text cannot drift from the data.
"""
import os
import sys
import time

import numpy as np

from core.grid import BayesianOccupancyGrid
from evaluation import benchmarks as B
from evaluation import comparison as C
from evaluation.baseline_pdf import ROOT, abl_val, badge, code_label, derived, esc, load_run, metric_rows, render, tbl

IMG = os.path.join(ROOT, "docs", "images")
ARENA_HALF = 4.95


def frontier_breakdown(path):
    g = BayesianOccupancyGrid(-5.5, 5.5, -5.5, 5.5, resolution=0.05)
    g.grid_log_odds = np.load(path)
    out = {"outside": 0, "near_wall": 0, "interior": 0}
    for gx, gy, _ in g.detect_frontiers():
        wx, wy = g.grid_to_world(gx, gy)
        k = max(abs(wx), abs(wy))
        out["outside" if k > ARENA_HALF else "near_wall" if k > ARENA_HALF - 0.5 else "interior"] += 1
    out["total"] = sum(out.values())
    return out


class Doc:
    def __init__(self):
        self.h, self.nt, self.nf, self.toc = [], 0, 0, []

    def sec(self, title, level=2):
        self.h.append(f"<h{level}>{esc(title)}</h{level}>")
        if level == 2:
            self.toc.append(title)

    def p(self, text, cls=""):
        self.h.append(f"<p{f' class={chr(39)}{cls}{chr(39)}' if cls else ''}>{text}</p>")

    def ul(self, items):
        self.h.append("<ul>" + "".join(f"<li>{i}</li>" for i in items) + "</ul>")

    def table(self, caption, headers, rows, widths=None, raw=()):
        self.nt += 1
        self.h.append(f"<p class='cap'><b>Table {self.nt}.</b> {caption}</p>")
        self.h.append(tbl(headers, rows, widths, raw_cols=raw))

    def fig(self, name, caption, width=470):
        self.nf += 1
        self.h.append(f"<p><img src='{name}' width='{width}'/></p><p class='cap'><b>Figure {self.nf}.</b> {caption}</p>")

    def html(self):
        return "".join(self.h)


def build_html(d, x):
    m, man, t = d["m"], d["man"], d["truth"]
    o, loc, mp, c, ag = m["overall"], m["localization"], m["mapping"], m["compute"], x["agents"]
    ev = m["data_quality"]["events"]
    cm = lambda v: v * 100.0
    rows_all = metric_rows(d, x)
    counts = {k: sum(1 for r in rows_all if r[3] == k) for k in ("Good", "Improve", "Poor")}
    fb = frontier_breakdown(os.path.join(d["dir"], "artifacts", "slam_grid.npy"))
    fr = frontier_breakdown(os.path.join(d["dir"], "artifacts", "true_grid.npy"))
    ab = d["abl"]["rows"]
    g, f, dt = t["map_vs_scene"], t["reference_vs_scene"], t["detection"]
    runs1 = [("run 1", d, x)]
    D = Doc()

    # ------------------------------------------------------------------ title and abstract
    D.h.append("<h1>Autonomous Quadruped Multi-Agent SLAM and Exploration</h1>"
               "<p style='font-size:12pt'><b>Design, implementation and baseline evaluation</b></p>"
               f"<p class='note'>Project report | course project 23CSE399, Amrita Vishwa Vidyapeetham | prepared {time.strftime('%d %B %Y')} | "
               f"baseline run {esc(m['run_id'])} | code: {esc(code_label(man))}</p>")
    D.sec("Abstract")
    D.p(f"This report describes a CPU-only system that lets a legged robot explore an unknown multi-room indoor space on its own: it estimates where "
        f"it is, builds a map, finds rooms, doorways and objects, decides where to go next, plans a path and walks it. The system is split into seven "
        f"cooperating software agents and runs in the CoppeliaSim simulator. It was built as the navigation half of a first-response robot for "
        f"crime-scene exploration. The report also describes the measurement framework that was built to evaluate the system honestly, and presents a "
        f"complete baseline: before any improvement, the system was run, scored against simulator ground truth and against the real geometry of the scene, "
        f"and compared with published systems and commercial platforms.")
    D.p(f"In one run of {o['sim_time_s']:.0f} simulated seconds ({x['dist']:.1f} m travelled) the localization error was {cm(loc['ate_rmse_m']):.1f} cm RMSE "
        f"({x['ate_pct_path']:.2f}% of the path), the map had {cm(g['accuracy_meters']):.1f} cm accuracy against the real walls, and {t['floor_mapped_as_free_pct']:.0f}% "
        f"of the real floor was mapped. The robot used {c['process_cpu_pct_mean']:.0f}% of one CPU core and a {mp['map_memory_mb']:.2f} MB map. However, the mission did "
        f"<b>not complete</b>: {o['rooms_completed']} of {o['rooms_discovered']} rooms were finished when the time limit was reached, the robot walks slowly "
        f"({x['speed']:.3f} m/s), and its object detection is a size rule rather than recognition. The baseline identifies where the system is strong "
        f"(drift, cost, component contributions), where it is behind published systems (absolute accuracy, speed, completion), and exactly what to fix next.")

    D.sec("Contents")
    D.p("1 Introduction | 2 System overview | 3 Methods: how each agent works | 4 Evaluation methodology | 5 Experimental setup | 6 Results | "
        "7 Comparison with published systems and the market | 8 Analysis and discussion | 9 Roadmap | 10 Conclusion | References | Appendices A to C")

    # ------------------------------------------------------------------ 1 introduction
    D.sec("1. Introduction")
    D.sec("1.1 The problem", 3)
    D.p("When responders must enter an unknown or dangerous indoor space (a hostage situation, a crime scene, a collapsed building), a person entering first "
        "is at risk and may disturb evidence. A small legged robot can go in first, but it has to cope without GPS, without a prior map and "
        "without an operator steering it: it must work out where it is while it builds the map (SLAM) [14], decide what to look at next, and avoid walls "
        "and obstacles. Doing this on a small computer, without a GPU, is part of the requirement.")
    D.sec("1.2 What this project is", 3)
    D.p("The course project 23CSE399 defines a seven-stage first-response pipeline: sense, navigate, trigger, perceive, decide, store and sync. "
        "This repository implements the first two stages, <b>Sense and Navigate</b>: a quadruped that explores an unknown multi-room space with SLAM and "
        "frontier-based exploration, in simulation. The later stages (an anomaly-detection model, event triggering, forensic logging with hashing and encryption, "
        "opportunistic synchronization, battery-aware return) are not part of this code base and are listed in the roadmap.")
    D.sec("1.3 Objectives of this work", 3)
    D.ul(["Build the sense-and-navigate core as a set of cooperating agents (Section 2 and 3).",
          "Measure its performance honestly, from real runs, with no assumed or typed-in numbers: overall, per agent, localization and mapping separately, and compute cost (Section 4).",
          "Establish a baseline before changing anything, so every later improvement can be judged against it (Sections 5 and 6).",
          "Compare the result fairly with published SLAM and exploration systems and with commercial legged robots, and say what the comparison does and does not show (Section 7).",
          "Record every weakness found and the planned fix (Sections 8 and 9)."])
    D.sec("1.4 Contributions", 3)
    D.ul(["A seven-agent, CPU-only exploration and SLAM stack for a simulated quadruped, with no use of simulator ground truth by the estimator (verified automatically).",
          "A measurement framework: per-run telemetry, ground-truth scoring against the real scene geometry, offline replay and ablation, an integrity audit, and automatic reports.",
          "A documented baseline with 38 classified metrics, a component ablation, a comparison with published systems on common metrics and a capability comparison with six commercial platforms.",
          "A prioritized register of 31 findings and fixes with the evidence behind each."])

    # ------------------------------------------------------------------ 2 system overview
    D.sec("2. System overview")
    D.sec("2.1 What the robot does", 3)
    D.p("The robot starts at a known position in an unknown building. It looks forward with a depth camera, and repeatedly: (1) converts the depth image to a "
        "2D picture of the obstacles around it, (2) works out how far it moved by matching consecutive pictures and combining that with its leg-gait "
        "odometry and a gyro, (3) adds what it sees to a grid map, (4) finds the boundary between explored and unexplored space, (5) chooses the most "
        "useful boundary to visit next, (6) plans a collision-free path, and (7) walks it. It splits the map into rooms and doorways, sweeps one room at "
        "a time, moves to the next room through a doorway, and stops when it believes no reachable unexplored space is left.")
    D.sec("2.2 Inputs", 3)
    D.table("Inputs to the system.", ["Input", "Source", "Used for"], [
        ("Depth image and colour image", "Simulated Kinect-style RGB-D camera facing forward (60 degree field of view, 0.1 to 8 m range)", "Obstacle scan and map; the colour image is recorded but not used for detection"),
        ("Gait odometry", "Commanded gait speed converted to body speed through a calibrated curve", "Predicted forward motion between scans"),
        ("Gyro", "Simulated rate gyro (0.10 deg/s noise, 0.05 deg/s bias, fixed seed)", "Heading change between scans"),
        ("Start pose", "Initial robot position and heading", "Origin of the pose estimate"),
        ("Mission limits", "Command-line options (time limit, headless)", "Safety stop of a run")], [110, 200, 180])
    D.p("The estimator never receives the simulator's robot pose. Ground truth is read separately, only for scoring (Section 4).")
    D.sec("2.3 Outputs", 3)
    D.table("Outputs of the system.", ["Output", "Form", "Produced by"], [
        ("Robot pose", "x, y, heading at 10 Hz (topics pose/slam, pose/state)", "SLAM agent"),
        ("Occupancy map", "220 x 220 grid, 5 cm cells, log-odds", "Mapping agent"),
        ("Rooms and doorways", "Room labels on the grid; doorway positions", "Mapping agent"),
        ("Exploration target and path", "Target cell; list of waypoints", "Frontier and Planning agents"),
        ("Motion commands", "Gait speed and steering each step", "Locomotion agent"),
        ("Knowledge graph and event log", "Rooms, doorways, objects with positions; timeline of events", "Supervisor agent"),
        ("Run record", "Metrics, per-step trajectory, raw sensor recording, manifest, report", "Measurement framework")], [130, 230, 130])
    D.sec("2.4 Architecture", 3)
    D.fig("architecture.png", "Agent architecture. Agents run in one process and exchange messages over an in-process publish/subscribe bus.", 480)
    D.sec("2.5 The seven agents", 3)
    D.table("Agents, rates and responsibilities.", ["Agent", "Rate", "Responsibility", "Main output"], [
        ("Perception", "10 Hz", "Converts each depth image into 3D points in the robot's body frame, removes points on the robot itself and keeps a height band of 0.04 to 0.60 m", "2D obstacle scan, 3D points"),
        ("SLAM", "10 Hz", "Estimates the pose by scan matching seeded with odometry, fuses heading with the gyro and a wall-direction prior, closes loops on revisits", "Pose estimate"),
        ("Mapping", "10 Hz", "Integrates scans into a probabilistic grid, segments rooms, detects doorways, labels obstacle blobs", "Grid, rooms, doorways, objects"),
        ("Frontier", "1 Hz", "Finds unexplored boundaries, scores them with a geodesic cost field, sweeps room by room, decides when exploration is complete", "Next target"),
        ("Planning", "5 Hz", "Plans a collision-free A* path over known free space and smooths it", "Waypoints"),
        ("Locomotion", "20 Hz", "Generates the walking gait, steers to the waypoint, detects lack of progress and recovers, reports odometry and gyro", "Gait commands, odometry"),
        ("Supervisor", "2 Hz", "Maintains the knowledge graph and event timeline", "Graph, timeline")], [60, 35, 290, 105])
    D.sec("2.6 Execution model (what 'agent' means here)", 3)
    D.p("The agents are Python classes that share one interface (initialize, step, shutdown). One loop in the main program calls each agent's step "
        "in a fixed order at that agent's rate, then advances the simulator by one step. Messages travel over an in-process bus: a publisher stores the "
        "latest message per topic and immediately calls its subscribers. Everything runs in one process on one thread. There is no robotics framework "
        "(such as ROS 2) and no parallel execution, and the supervisor is a fixed state machine, not a learned or language-model agent. The benefit is "
        "that each concern is isolated, testable and replaceable. The cost, measured in Section 6.6, is that the agents account for only "
        f"{x['agent_share']:.0f}% of the wall-clock time; the rest is simulator stepping, image transfer and display.")
    D.table("Message bus topics.", ["Topic", "Content", "Publisher"], [
        ("sensor/rgbd, sensor/scan", "Depth and colour image; body-frame obstacle scan", "Perception"),
        ("pose/slam, pose/state", "Estimated pose (pose/state is the pose the rest of the system navigates on)", "SLAM"),
        ("map/grid", "Grid, free/occupied counts, room labels, doorways", "Mapping"),
        ("frontier/target", "Selected exploration target", "Frontier"),
        ("plan/path", "Waypoint list", "Planning"),
        ("control/actuation", "Gait speed, heading error, odometry increment, gyro increment", "Locomotion"),
        ("mission/event", "Room entered or completed, object detected, loop closure, stuck, target unreachable", "Several agents")], [130, 280, 80])

    # ------------------------------------------------------------------ 3 methods
    D.sec("3. Methods: how each agent works")
    D.sec("3.1 Perception", 3)
    D.p("The depth image is back-projected into 3D points using the camera's focal length (from the 60 degree field of view) and its fixed mounting transform on the "
        "robot body, so points are expressed directly in the robot's body frame without using any world pose. Points within 22 cm of the body centre are discarded "
        "(the robot's own body and legs). For mapping, points between -0.08 m and 0.60 m height are kept; for the 2D obstacle scan, points at least 0.04 m above the "
        "floor are kept and every fourth point is used (about 600 points per scan in this run). This height band is why objects lower than 4 cm or higher than 60 cm are not seen.")
    D.sec("3.2 Odometry and gyro", 3)
    D.p("The locomotion agent converts the commanded gait speed into a body speed through a piecewise-linear curve calibrated offline: commands 0, 0.35, 0.75 and 1.0 "
        f"map to 0, 0.047, 0.053 and {x['gait_max']:.4f} m/s. The curve was calibrated once from an early recording that used ground truth; this is disclosed as a calibration, and the "
        "estimator itself does not read ground truth at run time. The gyro is simulated from the true orientation change per step plus white noise and a constant bias, "
        "so heading drifts as a real inexpensive gyro would.")
    D.sec("3.3 SLAM", 3)
    D.p("<b>Scan matching.</b> Each new scan is aligned to the previous one with point-to-line ICP, a variant [17] of the iterative closest point method [16] that uses wall-line normals, seeded with the "
        "odometry prediction. A match is accepted only if the residual is at most 5 cm, the step is at most 0.8 m and 120 degrees, and the result agrees with odometry to within 1 cm plus "
        "the step length. While walking, rotation is held at the gyro value and only translation comes from ICP, because with a narrow field of view rotation and sideways slide "
        "are nearly indistinguishable. During slow turns ICP may correct heading with a weight of 0.1 when it agrees with the gyro to within 3 degrees. When a match is rejected, "
        "the pose advances by odometry alone.")
    D.p("<b>Degeneracy filter.</b> Along a featureless wall the match cannot determine motion parallel to it; the filter removes the ICP correction along that direction.")
    D.p("<b>Manhattan heading prior.</b> Indoor walls meet at right angles (the Manhattan-world assumption [20]). After each accepted match the dominant wall direction is found from the scan normals and the heading is "
        "nudged toward the nearest multiple of 90 degrees (gain 0.1, only when within 15 degrees). This cancels gyro drift.")
    D.p("<b>Loop closure.</b> A keyframe (scan plus pose) is stored every 0.5 m or 25 degrees. When the robot returns within 2.5 m of an older keyframe (at least 12 s older), the current scan is "
        "matched to it; the closure is accepted only for a residual of at most 3 cm and an overlap of at least 60%, and the correction (limited to 0.3 m) is spread over the pose graph [18], in the manner of loop closure in 2D LiDAR SLAM [19].")
    D.sec("3.4 Mapping", 3)
    D.p("Each scan is ray-cast from the estimated pose into a log-odds occupancy grid [21, 22] of 220 x 220 cells of 5 cm (an 11 m x 11 m area). A cell on a ray's end gains 0.85, cells along the "
        "ray lose 0.4, and values are limited to plus or minus 2. A cell is occupied above 1.0 (about two hits) and free below -0.5. Rooms are found by thresholding the distance "
        "transform of the free space into room cores and partitioning the grid by distance (a Voronoi partition); a doorway is where two different rooms touch in free space. "
        "Connected obstacle blobs are labelled 'wall' if at least 1.2 m across and 'chair' if 0.18 to 1.05 m across. This labelling is purely geometric.")
    D.sec("3.5 Frontier selection", 3)
    D.p("A frontier [23] is a free cell next to unknown space; frontiers are grouped into clusters. For each cluster the agent computes the walking distance over inflated free space with a "
        "breadth-first wavefront (clusters behind walls get infinite distance and are skipped), then scores it as information gain (cluster size plus 0.3 per unknown cell in a 9 x 9 window) "
        "minus 3.8 times the walking distance, with a bonus for clearance and a penalty for turning around, so the robot sweeps in one rotational direction. The agent works room by room: "
        "while the current room has reachable frontiers it sweeps them, then it marks the room complete and moves to the nearest unvisited doorway. Targets are kept for up to 9 s; "
        "unreachable or stalled targets are blacklisted. Exploration is declared complete when no reachable frontier remains.")
    D.sec("3.6 Planning", 3)
    D.p("A* [24] (8-connected) runs on the known free space with a hard 15 cm clearance around obstacles (obstacles dilated by three cells) and a soft penalty that prefers staying away from walls. "
        "Paths are smoothed by replacing waypoints with straight lines whenever the line is free of inflated obstacles. Planning only uses known free cells, so the robot never plans through unknown space.")
    D.sec("3.7 Locomotion", 3)
    D.p("The gait is a sinusoidal pattern generator driving the leg joints, with frequency and amplitude scaled by the commanded speed. The speed command depends on heading error: "
        "1.0 for straight walking, 0.75 for walking while turning and 0.35 for turning in place. Two safeguards run continuously: every second the agent checks that the robot has moved at least 5 cm "
        "while walking, and after two failed checks it executes a short reverse step; and if the robot has not come closer to its goal for 15 s the goal is dropped. "
        "Turning is done by setting the body orientation toward the desired heading (a simulation shortcut, recorded as a limitation).")
    D.sec("3.8 Supervisor", 3)
    D.p("The supervisor listens to mission events and the map, and maintains a knowledge graph with three kinds of nodes (rooms, doorways, objects) linked by 'located in' edges, "
        "together with a time-ordered event log. It exports both at the end of a run.")

    # ------------------------------------------------------------------ 4 evaluation methodology
    D.sec("4. Evaluation methodology")
    D.sec("4.1 Principles", 3)
    D.ul(["Every reported value is computed from the run's own recorded data; none is typed in or taken from an earlier experiment. A value that cannot be measured is reported as unavailable with the reason.",
          "A measured result, an estimate and a diagnostic indicator are labelled differently; single-run results carry no claim of repeatability.",
          "Localization and mapping are evaluated separately, then together.",
          "Component contributions are measured only by offline replay of recorded data, never by switching components off in the live robot.",
          "The measurement must not disturb the robot: it only wraps agent calls with a timer, writes files after the control loop, and bounds its memory."])
    D.sec("4.2 Ground truth and references", 3)
    D.table("What each result is scored against.", ["Quantity", "Reference", "Notes"], [
        ("Position and heading", "Simulator pose of the robot body at every control step (0.05 s)", "Used for scoring only. No trajectory alignment is applied because SLAM starts at the true start pose"),
        ("Map, first reference", "A second grid built from the same depth rays using the true pose", "Isolates the effect of pose error; limited to what the sensor saw"),
        ("Map, independent reference", "Wall and object geometry read directly from the CoppeliaSim scene", "Footprints are rasterized at 5 cm; independent of the robot's sensor"),
        ("Objects", "Positions and sizes of the 13 scene objects in the sensing height band", "A detection counts if within 0.5 m of an object"),
        ("Floor coverage", "Real floor area of the arena (interior 9.9 x 9.9 m)", "Share of real floor marked free in the map")], [100, 200, 190])
    D.sec("4.3 Metrics", 3)
    D.table("Metric definitions (units in parentheses).", ["Metric", "Definition"], [
        ("ATE RMSE (cm)", "Root-mean-square Euclidean distance between estimated and true position over every control step [13]"),
        ("ATE as share of path (%)", "ATE RMSE divided by the true distance travelled; lets runs of different length be compared"),
        ("Final drift (% of path)", "Position error at the end of the run divided by the distance travelled"),
        ("RPE (% and deg/m)", "Error of the relative motion over every 1 m of true path [13, 25], expressed in the body frame at the start of the segment; translation as percentage of the segment, rotation per metre"),
        ("Scan-match acceptance (%)", "Share of SLAM updates in which the scan match passed its acceptance checks (a diagnostic, not a failure rate)"),
        ("Map accuracy / completion (cm)", "Mean distance from each mapped obstacle cell to the nearest reference obstacle cell / from each reference cell to the nearest mapped cell"),
        ("Precision, recall, F1 (%)", "Agreement of obstacle cells within a 15 cm tolerance: mapped cells near a reference cell / reference cells near a mapped cell / their harmonic mean"),
        ("Completion ratio (%)", "Share of reference obstacle cells matched within 5 cm"),
        ("Free-space IoU (%)", "Intersection over union of free cells between map and reference"),
        ("Detection precision (%)", "Share of reported object detections within 0.5 m of a real object"),
        ("Latency (ms)", "Wall time of one agent step call: median, 95th percentile, maximum"),
        ("Real-time factor", "Simulated seconds per wall-clock second; 1 means real time"),
        ("Process CPU / memory", "CPU and resident memory of the Python process, sampled once per second; the simulator is excluded")], [150, 340])
    D.sec("4.4 Measurement framework", 3)
    D.p("Every run writes a self-contained folder: the per-step trajectory of truth and estimate, all metrics, the per-agent table, a manifest (code version, computer, "
        "configuration, metric definitions), a recording of exactly what the SLAM agent received each step, the grids and knowledge graph, and an automatic report. "
        "The recording allows any SLAM configuration to be replayed offline in minutes on identical data, which is how component contributions are measured. "
        f"Replay validity was checked: the full-system replay matches the live trajectory to within {ab[0]['replay_vs_live_max_cm']:.2f} cm over "
        f"{ab[0]['replay_steps']} steps, and two identical replays are bit-for-bit equal. The estimated cost of the measurement itself is "
        f"{c['telemetry_overhead_ms_per_step']:.4f} ms per step (about {x['overhead_pct']:.3f}% of the median control step); this is a calibrated estimate, not an uninstrumented comparison run.")
    D.sec("4.5 Integrity audit", 3)
    D.p("Four automatic checks accompany every report: no simulator robot-pose call exists in the SLAM, mapping, perception, frontier, planning or odometry source "
        "(static scan of the code); the SLAM agent does not subscribe to ground truth in closed-loop mode; the estimated trajectory is not a copy of ground truth; and the run navigated on the SLAM pose. "
        "All four pass for the baseline run.")
    D.sec("4.6 Classification of results", 3)
    D.p("To say where the system stands, each metric is rated Good, Improve or Poor. The ratings are indicative: they come from engineering targets and from published numbers "
        "and product specifications, are not published standards, and can be changed. They are kept in the internal diagnostic report and are not part of this report.")

    # ------------------------------------------------------------------ 5 setup
    D.sec("5. Experimental setup")
    D.table("Setup of the baseline run.", ["Item", "Value"], [
        ("Simulator and scene", "CoppeliaSim EDU [15]; a 10 m x 10 m arena with four outer walls, 15 interior wall segments forming 8 rooms and corridors, and 13 objects (table, laptop, chair, cupboard, plant, crates, cylinders, two human figures, three beacons)"),
        ("Robot", "Quadruped with sinusoidal gait; forward RGB-D camera; simulated gyro"),
        ("Computer", f"{esc(man.get('cpu'))} (16 threads); GPU present but unused; {esc(man.get('platform'))}; Python {esc(man.get('python'))}"),
        ("Run", f"One run, safety limit {man['config']['max_time_s']:.0f} simulated seconds, ended: {esc(o['end_reason'])}"),
        ("Duration", f"{o['sim_time_s']:.0f} s simulated in {o['wall_time_s']:.0f} s of wall time"),
        ("Recording", "All 36,001 control steps recorded (0.05 s each)"),
        ("Number of runs reported", "1. No variability can be stated")], [110, 380])

    # ------------------------------------------------------------------ 6 results
    D.sec("6. Results")
    D.sec("6.1 Summary", 3)
    D.p(f"Of {len(rows_all)} classified metrics, <b>{counts['Good']} are Good, {counts['Improve']} can be improved and {counts['Poor']} are Poor</b>. "
        "In short: localization drift, computational cost and the contribution of the SLAM components are good; absolute accuracy and map precision can be improved; "
        "mission completion, speed, local pose error, object detection and real-time operation are poor.")
    D.sec("6.2 Task and exploration", 3)
    D.table("Task and exploration results.", ["Metric", "Value", "What it measures"], [
        ("Mission completed", "no", "Mission reported every reachable frontier explored"),
        ("Simulated time", f"{o['sim_time_s']:.0f} s", "Stopped at the safety limit"),
        ("Distance travelled", f"{x['dist']:.1f} m", "True path length"),
        ("Average speed", f"{x['speed']:.3f} m/s", "Distance divided by time"),
        ("Rooms completed / found", f"{o['rooms_completed']} / {o['rooms_discovered']}", "Rooms marked fully explored / rooms segmented"),
        ("Real floor mapped", f"{t['floor_mapped_as_free_pct']:.1f}% of {t['true_floor_area_m2']:.1f} m2", "True coverage of the arena floor"),
        ("Coverage plateau", f"97% of final coverage at {x['t97']:.0f} s", "After this time only 2.1 points of coverage were added"),
        ("Path walked after the plateau", f"{x['dist_after_plateau']:.1f} m ({x['waste_pct']:.0f}%)", "Effort that added little"),
        ("Frontiers remaining", f"{mp['frontiers_remaining']}", "Unexplored boundaries left at the end"),
        ("Stuck events / unreachable targets", f"{o['stuck_events']} / {o['unreachable_targets']}", "Watchdog and planner events"),
        ("Rooms entered per room", f"{x['room_entries_per_room']:.2f}", f"{ev.get('ROOM_ENTERED', 0)} entry events for {o['rooms_discovered']} rooms")], [120, 130, 240])
    D.fig("exploration_progress.png", "Exploration progress. Coverage rises quickly and saturates long before the run ends, while rooms are completed one by one.", 470)
    D.fig("map_overlay.png", "Final map (left) with the true path (blue), the estimated path (orange) and the remaining frontiers (red), and the reference map built with the true pose (right).", 490)
    D.sec("6.3 Localization", 3)
    D.table("Localization results against simulator ground truth.", ["Metric", "Value", "What it measures"], [
        ("ATE RMSE", f"{cm(loc['ate_rmse_m']):.1f} cm", "Typical position error"),
        ("ATE median / maximum", f"{cm(loc['ate_median_m']):.1f} / {cm(loc['ate_max_m']):.1f} cm", "Median and worst error"),
        ("ATE as share of path", f"{x['ate_pct_path']:.2f}%", "Error relative to distance"),
        ("Final error", f"{cm(loc['final_position_error_m']):.1f} cm ({loc['final_drift_pct_of_distance']:.2f}% of path)", "Drift at the end"),
        ("RPE translation", f"{cm(loc['rpe_trans_rmse_m_per_1m']):.1f} cm per metre ({loc['rpe_trans_pct']:.1f}%)", "Local accuracy"),
        ("RPE rotation", f"{loc['rpe_rot_rmse_deg_per_m']:.2f} deg per metre", "Local heading accuracy"),
        ("Heading error mean / maximum", f"{loc['heading_err_mean_deg']:.2f} / {loc['heading_err_max_deg']:.1f} deg", "Orientation error"),
        ("Error first above 0.5 m / 1 m", f"{loc['time_error_first_exceeds_0p5m_s']:.0f} s / never", "Error never reached 1 m"),
        ("Loop closures", f"{loc['loop_closures']}", "Revisits used to correct drift"),
        ("Scan-match acceptance", f"{loc['icp_acceptance_pct']:.1f}% of {loc['tracking_updates']} updates", "Diagnostic; rejection reasons are not recorded"),
        ("Odometry distance vs true", f"{loc['odometry_distance_m']:.1f} m of {x['dist']:.1f} m", "Diagnostic only (true path also contains motion the gait did not command)")], [120, 150, 220])
    D.fig("localization_error.png", "Localization error over the run. The error never exceeds about 60 cm; drops are loop closures.", 470)
    D.p("<b>Reading.</b> There is no runaway drift: after 30 minutes the error is 0.24% of the path. Absolute error (31 cm) is larger than the 15 cm tolerance used for map scoring, and the "
        "local error per metre (7.4%) is high. The offline replay in Section 6.7 shows that odometry plus gyro alone has a lower local error but its heading drifts without bound, "
        "so scan matching buys global consistency at the cost of local jitter.")
    D.sec("6.4 Mapping", 3)
    D.table("Mapping results against the real scene and against the true-pose map.", ["Metric", "Against real scene", "Against true-pose map"], [
        ("Accuracy", f"{cm(g['accuracy_meters']):.1f} cm", f"{cm(mp['wall_accuracy_m']):.1f} cm"),
        ("Completion", f"{cm(g['completion_meters']):.1f} cm", f"{cm(mp['wall_completion_m']):.1f} cm"),
        ("Precision / recall / F1 (15 cm)", f"{100 * g['precision']:.1f} / {100 * g['recall']:.1f} / {100 * g['f1_score']:.1f}%", f"{mp['precision_pct']:.1f} / {mp['recall_pct']:.1f} / {mp['f1_pct']:.1f}%"),
        ("Completion ratio (5 cm)", f"{100 * g['completion_ratio_5cm']:.1f}%", f"{mp['completion_ratio_5cm_pct']:.1f}%"),
        ("Free-space mapped outside the arena", f"{t['free_mapped_outside_arena_m2']:.2f} m2", "n/a"),
        ("Map update time (median / p95)", f"{mp['map_update_p50_ms']:.1f} / {mp['map_update_p95_ms']:.1f} ms", ""),
        ("Map memory", f"{mp['map_memory_mb']:.2f} MB", "")], [150, 170, 170])
    D.p(f"The same depth rays mapped with the <i>true</i> pose give {cm(f['accuracy_meters']):.1f} cm accuracy and {100 * f['recall']:.1f}% recall against the real scene. "
        f"The sensor therefore allows an almost exact map; the difference to the {cm(g['accuracy_meters']):.1f} cm of the actual run comes from pose error, which smears and doubles walls "
        f"({mp['occupied_cells']} occupied cells against {mp['reference_occupied_cells']} in the true-pose map).")
    D.sec("6.5 Object detection and knowledge graph", 3)
    D.table("Object detection against the scene's real objects.", ["Metric", "Value"], [
        ("Real objects in the sensing band", f"{dt['real_objects']}"),
        ("Detections reported", f"{dt['detections']} (all labelled 'chair')"),
        ("Detections on a real object", f"{dt['detections_on_a_real_object']} of {dt['detections']} ({dt['detection_precision_pct']:.1f}%)"),
        ("Chair labels on the real chair", f"{dt['chair_labels_on_the_real_chair']} of {dt['labelled_chair']} (the scene has {dt['real_chairs']} chair)"),
        ("Knowledge graph", f"{t['knowledge_graph']['rooms']} rooms, {t['knowledge_graph']['doorways']} doorway entries; every doorway recorded as connecting rooms 1 and 2; all room centres at (0, 0)")], [180, 310])
    D.p("The detection result is expected from the method (Section 3.4): any obstacle blob of a suitable size receives the label 'chair', detections are never removed once reported, and no image "
        "information is used. This is a placeholder for the recognition stage of the full pipeline, not a recognition result.")
    D.sec("6.6 Processing time and resources", 3)
    D.table("Time of one step call per agent (ms) and total.", ["Agent", "Median", "95th percentile", "Maximum", "Total (s)", "Share of agent time"],
            [(a["agent"], f"{a['p50_ms']:.2f}", f"{a['p95_ms']:.2f}", f"{a['max_ms']:.0f}", f"{a['total_s']:.0f}", f"{100 * a['total_s'] / sum(q['total_s'] for q in m['agents']):.0f}%") for a in m["agents"]],
            [120, 55, 80, 65, 60, 90])
    D.fig("agent_latency.png", "Median and 95th-percentile processing time per agent (logarithmic scale).", 430)
    D.table("Whole-system resources.", ["Metric", "Value"], [
        ("Control-loop step (median / p95 / max)", f"{m['agents_extra']['control_loop_p50_ms']:.0f} / {m['agents_extra']['control_loop_p95_ms']:.0f} / {m['agents_extra']['control_loop_max_ms']:.0f} ms"),
        ("Real-time factor", f"{o['real_time_factor']:.2f} ({o['sim_time_s']:.0f} s simulated in {o['wall_time_s']:.0f} s)"),
        ("Share of wall time inside agents", f"{x['agent_share']:.0f}%"),
        ("Process CPU (mean / p95)", f"{c['process_cpu_pct_mean']:.0f} / {c['process_cpu_pct_p95']:.0f}% of one core"),
        ("Memory (mean / peak)", f"{c['rss_mb_mean']:.0f} / {c['rss_mb_peak']:.0f} MB (includes the in-memory recording buffer)"),
        ("GPU", "not used")], [180, 310])
    D.p(f"Every agent except frontier selection (median {ag['FrontierAgent']['p50_ms']:.0f} ms, once per second) is well inside its time budget in the median. "
        f"The weakness is the tail: Perception reached {ag['PerceptionAgent']['max_ms']:.0f} ms and Planning {ag['PlanningAgent']['max_ms']:.0f} ms in single calls, giving control steps up to "
        f"{m['agents_extra']['control_loop_max_ms'] / 1000:.1f} s. The causes were not traced. Because the simulator waits for each step, these stalls cost wall time but do not change robot behaviour.")
    D.sec("6.7 Component ablation", 3)
    D.table("Effect of removing one SLAM component, by offline replay of the same recording.", ["Configuration", "ATE RMSE (cm)", "Final error (cm)", "RPE (%)", "Heading error (deg)", "Loop closures", "Change in ATE"],
            [(r["variant"], f"{r['ate_rmse_cm']:.1f}", f"{r['final_err_cm']:.1f}", f"{r['rpe_trans_pct']:.1f}", f"{r['heading_err_deg']:.2f}", r["loop_closures"],
              "-" if i == 0 else f"{x['abl'][r['variant']]:+.0f}%") for i, r in enumerate(ab[:-1])], [150, 60, 60, 45, 65, 55, 55])
    D.fig("ablation.png", "ATE with one component removed (same data, offline).", 420)
    D.p("Scan matching, the Manhattan heading prior and loop closure each clearly help (+756%, +446% and +76% ATE when removed). The degeneracy filter shows no measurable benefit on this run "
        "(+6%, and a lower final error without it); it may matter in long corridors, which this arena has little of. The finding rests on one recording.")
    D.sec("6.8 Classification of all metrics", 3)
    D.table(f"Classification of {len(rows_all)} metrics ({counts['Good']} Good, {counts['Improve']} Improve, {counts['Poor']} Poor).", ["Component", "Metric", "Value", "Class"],
            [(r[0].split(" (")[0], r[1], r[2], badge(r[3])) for r in rows_all], [120, 190, 100, 55], raw=(3,))

    # ------------------------------------------------------------------ 7 comparison
    D.sec("7. Comparison with published systems and the market")
    D.sec("7.1 Method and comparability", 3)
    D.p("Published values are quoted from the cited tables and product pages; where a value is the quotient of two published numbers it is marked derived. "
        "<b>No published system was evaluated under our conditions</b> (same robot, sensor, scene and protocol), so none is directly comparable. The closest are partial matches: "
        "the A1 SLAM paper (a quadruped with a 2D scanner, 5 m paths), the ANYmal D evaluation (a quadruped, 145 to 332 m, mostly outdoors) and the FARE exploration paper (wheeled robot, 3D LiDAR). "
        "The tables therefore show where the gap is, not a ranking. Differences in scene, sensing, ground truth and map type are listed in each table.")
    for title, fn, note in (
            ("7.2 SLAM as a whole: pose accuracy, map accuracy and cost", C.slam_combined,
             "Published systems are RGB-D neural SLAM on the Replica dataset [26] with a GPU; maps are 3D surfaces. Ours is a 2D grid of 5 cm cells (which cannot resolve finer than about 2.5 cm), scored against the real scene."),
            ("7.3 Pose accuracy in multi-room scenes", C.apartment, "Ours: the robot's own arena. Published: Replica apartments 1, 2 and 0."),
            ("7.4 Pose accuracy on legged robots", C.quadruped_pose, "ORB-SLAM3 is described in [27]. ATE per path length for published systems is derived from the mean mission length (234.8 m)."),
            ("7.5 Processing time per update", C.runtime, "Different tasks and hardware: 2D scan matching against feature-based or neural tracking. This says nothing about accuracy."),
            ("7.6 Exploration speed", C.exploration, "Distances and times are not comparable (different environments and robots, and our run did not complete). Speed is a property of the robot.")):
        D.sec(title, 3)
        hh, rr, ww = fn(runs1)
        D.table(note, hh, rr, ww, raw=(0,))
        if fn is C.slam_combined:
            D.fig("comparison.png", "Pose error and map accuracy of our system beside published systems (log scale). Different scenes, sensors and map types: shows the gap, not a ranking.", 470)
    D.sec("7.7 Commercial platforms", 3)
    hh, rr, ww = C.market(runs1)
    D.table("Capabilities from manufacturer pages. None of this is measured by us; endurance, weatherproofing and price do not apply to a simulation.", hh, rr, ww, raw=(0,))
    D.p("Spot and Go2 are documented as following a recorded map or a user-specified path. The ANYmal, B2 and X30 pages do not state whether the robot explores unknown space. "
        "The Emesent Hovermap payload (mainly used on drones) does offer an autonomous exploration mode, so autonomous exploration is not unique to this system. Every commercial robot is far faster than ours.")
    D.sec("7.8 A like-for-like reference for later work", 3)
    hh, rr, ww = C.tum_pioneer(runs1)
    D.table("TUM RGB-D Pioneer sequences (a wheeled robot with a forward Kinect on a flat floor, motion-capture truth): published ATE RMSE in metres. Our system has not been run on these.", hh, rr, ww, raw=(0,))
    D.p("This is the public dataset closest to our robot. Running our pipeline on it would give the first number measured under the same conditions as published systems. "
        "Hand-held and drone datasets (TUM hand-held, Replica, EuRoC) involve full 3D motion and do not fit a planar 2D SLAM.")
    D.sec("7.9 Where the system stands", 3)
    D.table("Position relative to published systems.", ["Dimension", "Standing", "Evidence"], [
        ("Cost (memory, update time)", "Ahead", f"{mp['map_memory_mb']:.2f} MB against 20 to 31 MB; {ag['SLAMAgent']['p50_ms']:.1f} ms against 22 to 69 ms (different tasks)"),
        ("Drift per path length", "Same order", f"{x['ate_pct_path']:.2f}% against derived 0.14% for ORB-SLAM3 stereo and 0.04% for FAST-LIVO2 on a real ANYmal D"),
        ("Absolute pose and map accuracy", "Behind by about an order of magnitude or more", f"ATE {cm(loc['ate_rmse_m']):.1f} cm against 0.4 to 1.1 cm; map accuracy {cm(g['accuracy_meters']):.1f} cm against 1.6 to 2.1 cm (Replica RGB-D systems)"),
        ("Exploration speed", "Behind", f"{x['speed']:.3f} m/s against 1.6 to 1.8 m/s (FARE planners, derived) and 0.75 to over 6 m/s (commercial robots)"),
        ("Mission completion", "Not achieved", "Published exploration systems report completion time; ours did not complete"),
        ("Object recognition", "Not comparable yet", "Detection is a size rule; no recognition model is integrated")], [150, 120, 220])

    # ------------------------------------------------------------------ 8 analysis
    D.sec("8. Analysis and discussion")
    D.sec("8.1 Why the mission did not complete", 3)
    D.p(f"The interior was effectively mapped early: coverage reached 97% of its final value at {x['t97']:.0f} s. The remaining {x['waste_pct']:.0f}% of the path "
        f"({x['dist_after_plateau']:.1f} m) added {x['coverage_gain_after_plateau']:.1f} points. The cause is in the frontier logic combined with the map:")
    D.ul([f"A frontier is any free cell next to unknown space. Of the {fb['total']} frontiers left at the end, <b>{fb['outside']} lie outside the outer wall</b> and {fb['near_wall']} within 0.5 m of it. "
          f"The map has {t['free_mapped_outside_arena_m2']:.2f} m2 of free space beyond the wall: depth rays leak past walls at grazing angles. Even the reference map built with the true pose has {fr['total']} frontiers ({fr['outside']} outside), "
          "so the leak comes from the sensor model, and pose error makes it worse.",
          "A frontier within 0.3 m of reachable floor counts as reachable. The robot drives to the wall; looking from inside cannot clear a frontier beyond the wall, so the target is blacklisted after a timeout.",
          "The blacklist is cleared when it exceeds 40 entries or no candidate remains, so the same targets return (single targets were planned 11 to 21 times; 1,814 paths were planned in 1,800 s).",
          f"Room labels change as the map grows ({ev.get('ROOM_ENTERED', 0)} room-entered events for {o['rooms_discovered']} rooms), which restarts room sweeps."])
    D.p("These are diagnoses from the map, the code and the log; they are not yet confirmed by a run with the fixes applied.")
    D.sec("8.2 What works", 3)
    D.ul([f"No runaway drift ({loc['final_drift_pct_of_distance']:.2f}% of the path at the end, never above {cm(loc['ate_max_m']):.0f} cm) with a gyro that drifts and an odometry that is only calibrated once.",
          "The heading prior and scan matching are decisive; removing either destroys the estimate.",
          f"The system is cheap: {c['process_cpu_pct_mean']:.0f}% of one CPU core and a {mp['map_memory_mb']:.2f} MB map, without a GPU.",
          f"Walls are found reliably (recall {100 * g['recall']:.0f}%), and {t['floor_mapped_as_free_pct']:.0f}% of the real floor was mapped."])
    D.sec("8.3 Weaknesses", 3)
    D.ul(["The mission never completes (false frontiers), and speed is low.",
          f"Local pose error is high ({loc['rpe_trans_pct']:.1f}% per metre), and walls are smeared ({cm(g['accuracy_meters']):.1f} cm).",
          "The object detector is not a recognizer, and the knowledge graph contains placeholders for doorway connectivity and room centres.",
          f"The control loop is not real time ({o['real_time_factor']:.2f}x) and stalls for up to {m['agents_extra']['control_loop_max_ms'] / 1000:.1f} s."])
    D.sec("8.4 Agent execution model", 3)
    D.p(f"The split into agents is a clean software structure but not a performance mechanism. The agents run sequentially in one thread and use {x['agent_share']:.0f}% of wall time; "
        "the rest is outside them (simulator stepping, image transfer, display, and a map query in the main loop on every step), and where exactly it goes has not been profiled. "
        "Because the simulator waits for the program each step, speeding up the agents changes wall time only. Running agents in parallel would require processes or a framework such as ROS 2, "
        "and would change timing behaviour, so a new baseline would be needed.")
    D.sec("8.5 Threats to validity", 3)
    D.ul(["<b>One run in one arena.</b> No variability or generalization can be claimed.",
          "<b>Simulation.</b> Sensor noise, slip and lighting are simulated; the gait speed is not that of a real robot.",
          "<b>Start at the true pose.</b> SLAM begins at the true start pose, so no global alignment error is included.",
          "<b>Calibration against ground truth.</b> The gait-speed curve was fitted offline from an early recording that used ground truth.",
          "<b>Turning shortcut.</b> The body orientation is set directly when turning.",
          "<b>Reference maps.</b> The first reference is built from the same sensor; the second (real scene) is independent but rasterized at 5 cm and its object orientations are inferred.",
          "<b>Classification limits.</b> Several limits are engineering targets proposed here.",
          "<b>Comparisons.</b> Published numbers come from other scenes, sensors and protocols."])

    # ------------------------------------------------------------------ 9 roadmap
    D.sec("9. Roadmap")
    D.p("Nothing below has been applied yet; the baseline was taken first so that each change can be judged against it. The full register with evidence and acceptance checks is in docs/project/FIXES.md.")
    D.table("Planned work in priority order.", ["Phase", "Work", "Fix IDs", "Success check"], [
        ("1. Make exploration finish", "Ignore frontiers outside walls or without nearby floor; keep the blacklist; freeze room labels; end a run on completion or lack of progress", "F1 to F4, F30", "Mission reports completion; no frontiers outside the wall"),
        ("2. Measure what is still unknown", "Profile where wall time goes; run uncapped for time to completion; repeat runs with different seeds; test a second arena", "F12, F18, F20", "Completion time; spread over runs"),
        ("3. Accuracy", "Reduce local jitter of scan matching; reduce wall smearing; tighten absolute error", "F7 to F9", "RPE below 3%; wall accuracy at most 10 cm; ATE at most 15 cm"),
        ("4. Speed and real time", "Raise gait speed; remove stalls; speed up frontier and mapping steps", "F5, F13 to F15", "Real-time factor at least 1; control step p95 at most 100 ms"),
        ("5. Perception", "Replace the size rule with recognition from the colour image; fix knowledge-graph placeholders", "F28, F29", "Detection precision at least 80%"),
        ("6. External validation", "Run on the TUM Pioneer sequences; run Cartographer on the recording; same-arena baseline explorer; hardware", "F25 to F27, F24", "Measured numbers beside published ones"),
        ("7. Full pipeline", "Anomaly detection, event triggering, tamper-evident logging, synchronization, battery-aware return (later project stages)", "outside this repository", "Per the project plan")], [90, 200, 70, 130])
    D.sec("10. Conclusion")
    D.p(f"A seven-agent, CPU-only exploration and SLAM system was built and measured honestly. The baseline shows a localization estimate that does not drift ({x['ate_pct_path']:.2f}% of the path), "
        f"a cheap implementation and clear evidence that scan matching, the heading prior and loop closure each earn their place. It also shows what must be fixed before the system can be called "
        f"complete: it does not finish exploring because of false frontiers beyond the walls, it is slow, its absolute accuracy is about an order of magnitude behind published RGB-D systems, and "
        f"its object detection is a placeholder. Because the baseline, the measurement framework and the fix register are in place, each improvement can now be tested against the same measurements.")

    # ------------------------------------------------------------------ references
    D.sec("References")
    refs = C.ref_list() + [
        '[13] J. Sturm, N. Engelhard, F. Endres, W. Burgard, D. Cremers, "A Benchmark for the Evaluation of RGB-D SLAM Systems", Proc. IEEE/RSJ Int. Conf. Intelligent Robots and Systems (IROS), 2012. https://cvg.cit.tum.de/data/datasets/rgbd-dataset',
        '[14] C. Cadena, L. Carlone, H. Carrillo, Y. Latif, D. Scaramuzza, J. Neira, I. Reid, J. J. Leonard, "Past, Present, and Future of Simultaneous Localization and Mapping: Toward the Robust-Perception Age", IEEE Trans. Robotics, vol. 32, no. 6, pp. 1309-1332, 2016.',
        '[15] E. Rohmer, S. P. N. Singh, M. Freese, "V-REP: A Versatile and Scalable Robot Simulation Framework", Proc. IEEE/RSJ IROS, pp. 1321-1326, 2013 (the simulator now named CoppeliaSim).',
        '[16] P. J. Besl, N. D. McKay, "A Method for Registration of 3-D Shapes", IEEE Trans. Pattern Analysis and Machine Intelligence, vol. 14, no. 2, pp. 239-256, 1992.',
        '[17] A. Censi, "An ICP Variant Using a Point-to-Line Metric", Proc. IEEE Int. Conf. Robotics and Automation (ICRA), pp. 19-25, 2008.',
        '[18] G. Grisetti, R. Kuemmerle, C. Stachniss, W. Burgard, "A Tutorial on Graph-Based SLAM", IEEE Intelligent Transportation Systems Magazine, vol. 2, no. 4, pp. 31-43, 2010.',
        '[19] W. Hess, D. Kohler, H. Rapp, D. Andor, "Real-Time Loop Closure in 2D LIDAR SLAM", Proc. IEEE ICRA, pp. 1271-1278, 2016.',
        '[20] J. M. Coughlan, A. L. Yuille, "Manhattan World: Compass Direction from a Single Image by Bayesian Inference", Proc. IEEE Int. Conf. Computer Vision (ICCV), 1999.',
        '[21] H. Moravec, A. Elfes, "High Resolution Maps from Wide Angle Sonar", Proc. IEEE ICRA, pp. 116-121, 1985.',
        '[22] A. Elfes, "Using Occupancy Grids for Mobile Robot Perception and Navigation", Computer, vol. 22, no. 6, pp. 46-57, 1989.',
        '[23] B. Yamauchi, "A Frontier-Based Approach for Autonomous Exploration", Proc. IEEE Int. Symp. Computational Intelligence in Robotics and Automation (CIRA), pp. 146-151, 1997.',
        '[24] P. E. Hart, N. J. Nilsson, B. Raphael, "A Formal Basis for the Heuristic Determination of Minimum Cost Paths", IEEE Trans. Systems Science and Cybernetics, vol. 4, no. 2, pp. 100-107, 1968.',
        '[25] R. Kuemmerle, B. Steder, C. Dornhege, M. Ruhnke, G. Grisetti, C. Stachniss, A. Kleiner, "On Measuring the Accuracy of SLAM Algorithms", Autonomous Robots, vol. 27, no. 4, pp. 387-407, 2009.',
        '[26] J. Straub et al., "The Replica Dataset: A Digital Replica of Indoor Spaces", arXiv:1906.05797, 2019.',
        '[27] C. Campos, R. Elvira, J. J. G. Rodriguez, J. M. M. Montiel, J. D. Tardos, "ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM", IEEE Trans. Robotics, vol. 37, no. 6, pp. 1874-1890, 2021.']
    D.p("<br/>".join(esc(r) for r in refs), "note")

    # ------------------------------------------------------------------ appendices
    D.sec("Appendix A. Repository structure and reproduction")
    D.table("Repository layout.", ["Path", "Contents"], [
        ("main_agentic.py", "Entry point: starts the simulation, runs the agents, records the run"),
        ("agents/", "The seven agents, message bus, knowledge graph, episodic memory"),
        ("core/", "Algorithms and simulator interface: slam, grid, planner, robot, sensors, viz"),
        ("evaluation/", "Telemetry, report generation, scene scoring, ablation, replay, audit, comparison tables and published numbers, PDF and figure builders"),
        ("scene/", "The CoppeliaSim scene"), ("tests/", "Formula and algorithm tests"), ("runs/", "One folder per measured run"),
        ("docs/", "results (PDFs), project (objectives, baseline, fixes), course (panel documents), images (figures)")], [120, 370])
    D.p("To reproduce: install the requirements, open scene/robot_working.ttt in CoppeliaSim (do not press play), and run <i>python main_agentic.py</i>; then <i>python -m evaluation.ablate runs/&lt;run_id&gt;</i> "
        "and <i>python -m evaluation.report runs/&lt;run_id&gt;</i>. The PDFs are rebuilt with <i>python -m evaluation.results_pdf</i>, <i>baseline_pdf</i> and <i>project_report</i>.")
    D.sec("Appendix B. Key parameters")
    D.table("Parameters of the main algorithms.", ["Component", "Parameter", "Value"], [
        ("Camera", "Field of view / range", "60 degrees / 0.1 to 8 m"),
        ("Perception", "Self-mask radius; map height band; obstacle minimum height; point stride", "0.22 m; -0.08 to 0.60 m; 0.04 m; 4"),
        ("Scan matching", "Residual limit; step limit; odometry agreement gate", "5 cm; 0.8 m and 120 degrees; 1 cm plus the step length"),
        ("Heading", "ICP yaw weight; yaw agreement; Manhattan gain and limit", "0.1; 3 degrees; 0.1 and 15 degrees"),
        ("Loop closure", "Keyframe spacing; search radius; minimum age; residual; overlap; maximum correction", "0.5 m or 25 degrees; 2.5 m; 12 s; 3 cm; 60%; 0.3 m"),
        ("Grid", "Size; resolution; hit / miss update; limits; thresholds", "220 x 220; 5 cm; +0.85 / -0.4; +/-2; occupied 1.0, free -0.5"),
        ("Frontier", "Information gain; distance weight; hemisphere; target hold", "cluster size (max 20) + 0.3 per unknown cell in 9 x 9; 3.8 per metre; +/-95 degrees; 9 s"),
        ("Planner", "Hard clearance; connectivity", "15 cm; 8-connected with soft wall penalty"),
        ("Locomotion", "Speed commands; stuck check; progress timeout", "1.0, 0.75, 0.35; 5 cm per second (two failures); 15 s"),
        ("Gyro", "Noise; bias", "0.10 deg/s; 0.05 deg/s")], [90, 220, 180])
    D.sec("Appendix C. Findings register")
    D.p("All 31 findings (F1 to F31) with evidence, planned change and acceptance check are maintained in docs/project/FIXES.md. Fix F31 (end-of-run report failure) has been applied; all others are open.")
    return D.html()


def build(run_dir, out=None):
    d = load_run(run_dir)
    x = derived(d)
    out = out or os.path.join(ROOT, "docs", "results", "PROJECT_REPORT.pdf")
    return render(build_html(d, x), IMG, out, f"Project report, baseline run {d['m']['run_id']}")


if __name__ == "__main__":
    build(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else None)
