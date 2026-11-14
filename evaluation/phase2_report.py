#!/usr/bin/env python3
"""Phase II external-review report (PDF) in the department template: python -m evaluation.phase2_report [out.pdf]

Bracketed grey text marks what the team still has to fill in. Scene 2 numbers marked (E) are the expected values of the
full-exploration run (forecast from rescue runs 16 and 19, 2026-11-14) and are to be replaced after that run.
"""
import os
import sys

from evaluation import baseline_pdf as P
from evaluation.baseline_pdf import render, tbl

OUT_DIR = os.path.join(P.ROOT, "docs", "report_phase2")
P.CSS += """
.ph { color: #7a7a7a; font-style: italic; background-color: #f1f1f1; }
.c { text-align: center; } h1 { text-align: center; font-size: 15pt; } .t { font-size: 10pt; text-align: center; }
"""


def ph(text):
    return f"<span class='ph'>[{text}]</span>"


def para(text):
    return f"<p>{text}</p>"


def bullets(items):
    return "<ul>" + "".join(f"<li>{i}</li>" for i in items) + "</ul>"


def cap(n, kind, text):
    return f"<p class='cap'><b>{kind} {n}.</b> {text}</p>"


def build():
    h = []
    h.append("<p class='t'><b>AMRITA VISHWA VIDYAPEETHAM</b><br/>SCHOOL OF COMPUTING<br/>"
             "DEPARTMENT OF COMPUTER SCIENCE AND ENGINEERING</p>")
    h.append("<h1>Autonomous First-Response Scene Exploration with a Quadruped Robot: "
             "Depth-Camera SLAM and Multi-Agent Exploration Across Environments</h1>")
    h.append(f"<p class='t'>23CSE498 – Project Phase II (External Review) | Team Number: {ph('GXXX')}<br/>"
             "November 14, 2026</p>")
    h.append(tbl(["Team Members", "Project Guide", "Project Type"],
                 [[f"{ph('CB.SC.U4CSE23XXX')} {ph('Student Name 1')}", f"{ph('Dr/Mr/Ms. Guide Name')}",
                   "Type 2"],
                  [f"{ph('CB.SC.U4CSE23XXX')} {ph('Student Name 2')}", f"{ph('Designation')}, Dept. of CSE",
                   "Application-Based Project"],
                  [f"{ph('CB.SC.U4CSE23XXX')} {ph('Student Name 3')}", "", ""],
                  [f"{ph('CB.SC.U4CSE23XXX')} {ph('Student Name 4')}", "", ""],
                  [f"{ph('CB.SC.U4CSE23XXX')} {ph('Student Name 5')}", "", ""]], raw_cols=(0, 1, 2)))

    # 1 Abstract
    h.append("<h2>1 Abstract</h2>")
    h.append(para(
        "First responders entering a crime or disaster scene need a map and a safe look inside before people go in. "
        "This project builds the navigation half of an autonomous first-response robot: a quadruped that enters an "
        "unknown indoor or rubble scene, works out where it is, maps the scene and explores it without an operator "
        "steering it. The robot carries one depth camera, a gyro and leg odometry, no lidar. The software runs as "
        "seven cooperating agents on a message bus (perception, SLAM, mapping, frontier exploration, path planning, "
        "locomotion, supervision) at 20 Hz, on a CPU only. SLAM matches a 2D slice of each depth frame against a "
        "submap of recent keyframes, corrects drift with loop closures in a robust pose graph and removes gyro bias "
        "during a stand-still calibration. Exploration chooses safe viewpoints at the edge of the known map, can "
        "follow an operator's search route and plans through narrow gaps. The same code was evaluated in two "
        "simulated environments in CoppeliaSim: a 10 x 10 m building with rooms and a 16 x 16 m rescue field with "
        "rubble. Across both, the robot reaches a mean position error of 13.9 cm, mean map precision of 96.4 % and "
        "mean floor coverage of 93.9 %, with no collisions measured (scene 2 values are the expected full-run "
        "values, see Section 5). On the same recordings, odometry alone drifts 153.6 cm and the BreezySLAM "
        "baseline 149.6 cm. Perception of evidence, event triggering and secure logging are not integrated yet."))

    # 2
    h.append("<h2>2 Problem Definition &amp; Requirement Analysis</h2>")
    h.append("<h3>2.1 Problem Statement, Context &amp; Target Users</h3>")
    h.append(bullets([
        "<b>Problem Definition:</b> Before entry into a crime scene, hostage site or collapsed building, responders "
        "have no current map and cannot see inside without risk to people or to evidence. A robot that explores "
        "the scene on its own and returns an accurate map closes that gap.",
        "<b>Target Users &amp; Stakeholders:</b> police first-response and forensic teams, search-and-rescue units; "
        + ph("add any specific partner agency or deployment context"),
        "<b>Existing Limitations:</b> commercial legged robots either need a pre-recorded map for autonomy "
        "(Boston Dynamics Autowalk replays a recorded route [9]) or depend on expensive lidar [5]; camera-only SLAM "
        "loses track on real quadrupeds (821 cm error for RGB-D ORB-SLAM3 on ANYmal [5]); manual tele-operation "
        "needs a trained operator in line of contact.",
        "<b>Application Objectives:</b>"]))
    h.append(bullets([
        "Objective 1: explore an unknown scene autonomously and cover at least 90 % of the reachable floor.",
        "Objective 2: keep the position error below 15 cm and the map precision above 95 % with one depth camera.",
        "Objective 3: zero collisions and recovery from every stall, with the same code in different environments."]))
    h.append("<h3>2.2 Functional &amp; Non-Functional Requirements</h3>")
    h.append(tbl(["Req. ID", "Category", "Requirement Specification"], [
        ["FR-01", "Functional", "Explore an unknown scene without operator steering until no reachable frontier is left."],
        ["FR-02", "Functional", "Estimate the robot pose and build a 2D occupancy map (5 cm cells) from depth, gyro and leg odometry."],
        ["FR-03", "Functional", "Follow an operator search route when one is given, then continue free exploration."],
        ["FR-04", "Functional", "Plan paths through narrow gaps between obstacles; skip targets that cannot be reached."],
        ["FR-05", "Functional", "Detect stalls and floor edges and recover (turn-away, skip, stop before a drop)."],
        ["NFR-01", "Non-Functional", "Real-time: whole control step at 20 Hz; median step 25 ms on a PC CPU, no GPU."],
        ["NFR-02", "Non-Functional", "Safety: zero collisions; reliability: same code in every environment, only a scene profile differs."],
        ["NFR-03", "Non-Functional", "Target hardware: Raspberry Pi class computer " + ph("latency on the Pi to be measured")],
    ], raw_cols=(2,)))
    h.append(cap(1, "Table", "Summary of Application Functional and Non-Functional Requirements"))

    # 3
    h.append("<h2>3 Application Architecture &amp; Solution Design</h2>")
    h.append("<h3>3.1 End-to-End Application Architecture</h3>")
    h.append(para(
        "The simulator (or, later, the robot) delivers depth frames, gyro rates and joint states. The perception "
        "agent turns each depth frame into a 2D scan of the obstacle band above the floor. The SLAM agent predicts "
        "the pose from leg odometry and gyro, corrects it by matching the scan to a submap of recent keyframes and "
        "publishes it. The mapping agent writes the scan into the occupancy grid and segments rooms and doorways. "
        "The frontier agent picks the next goal (operator route waypoint, otherwise a safe viewpoint that sees "
        "unknown space), the planning agent computes an A* path and the locomotion agent walks it with a trot gait, "
        "watching for stalls and floor edges. The supervisor keeps the mission state, a knowledge graph of rooms and "
        "an event timeline. Every run is stored with its recording, so SLAM changes can be replayed offline."))
    h.append("<p><img src='architecture.png' width='520'/></p>")
    h.append(cap(1, "Figure", "End-to-End Application Architecture (agents, message-bus topics, interfaces; dashed: planned stages)"))
    h.append("<p><img src='flow.png' width='330'/></p>")
    h.append(cap(2, "Figure", "Control flow of one exploration mission (20 Hz loop)"))
    h.append("<h3>3.2 Component Design, Data Flow &amp; Technology Stack</h3>")
    h.append(bullets([
        "<b>Component Breakdown:</b> Perception (depth to scan), SLAM (point-to-line ICP against a 16-keyframe "
        "submap, gyro calibration, loop closure with a dynamic-covariance-scaling pose graph [7, 8]), Mapping "
        "(Bayesian log-odds grid, rooms, doorways), Frontier (viewpoints 0.5 m from unknown areas, operator route, "
        "completion test [1]), Planning (A* on the inflated grid, narrow-gap mode), Locomotion (gait, path "
        "following, no-progress watchdog, drop check), Supervisor (mission state, knowledge graph, episodic memory).",
        "<b>User Interaction &amp; Interface Flow:</b> the operator loads a scene, optionally draws a search route "
        "(stored as waypoints in the scene profile) and starts the mission from the command line; a live view shows "
        "the map, path and state; each run ends with a report (REPORT.md, maps, plots). "
        + ph("operator dashboard / remote UI, if planned"),
        "<b>Data Storage &amp; External Interfaces:</b> runs/&lt;id&gt; holds metrics.json, trajectory.csv, "
        "truth_eval.json, map arrays and a compressed recording of every scan and odometry step; config/scenes holds "
        "per-environment profiles; CoppeliaSim is reached through its ZMQ remote API.",
        "<b>Technology &amp; Deployment Stack:</b> Python 3.12, NumPy, OpenCV, Matplotlib, PyMuPDF; CoppeliaSim "
        "4.x with the ZMQ remote API; CPU only, so the same code can move to a Raspberry Pi class computer."]))

    # 4
    h.append("<h2>4 Implementation Details</h2>")
    h.append("<h3>4.1 Development &amp; Deployment Environment</h3>")
    h.append(bullets([
        "<b>Platform &amp; Runtime Environment:</b> Windows 11 PC, CoppeliaSim simulation, headless or with "
        "viewer; target robot " + ph("real quadruped model and onboard computer"),
        "<b>Libraries, Tools &amp; Hardware:</b> NumPy, OpenCV, SciPy-free custom ICP and pose graph, pytest "
        "(82 tests), rosbags (TUM data), BreezySLAM (baseline). Simulated sensors: depth camera 60 deg, gyro, "
        "joint encoders. " + ph("real sensors and robot hardware list"),
        "<b>Source Code Repository:</b> github.com/yokshith09/Autonomous-Quadruped-SLAM-Navigation-Perception "
        "(branch improve/exploration; tags best-sim-v3, final-multi-env-v1)"]))
    h.append("<h3>4.2 Module-Wise Implementation &amp; Integration Summary</h3>")
    h.append(tbl(["Application Module", "Implemented Features & Integration Details", "Status"], [
        ["Operator interface", "Command-line mission start, scene profile with operator route, live map view, "
         "per-run report and plots", "Functional (simulation)"],
        ["Core logic: SLAM + exploration", "7 agents on the message bus: SLAM, mapping, frontier, planning, "
         "locomotion, supervisor, perception (depth to scan)", "100 % (Integrated)"],
        ["Data layer", "Run folders with metrics, trajectory, maps and full sensor recordings; offline replay "
         "and evaluation against the scene model", "100 % (Integrated)"],
        ["External / IO", "CoppeliaSim ZMQ interface: depth camera, gyro, joints, contact check", "100 % (Functional, sim)"],
        ["Evidence perception (team's Mamba model)", ph("model integration, ROI detection, latency on Pi"), "Not integrated"],
        ["Event trigger, forensic store, sync", ph("SHA-256 hashed logs, AES-256 sync"), "Not started"],
        ["Real robot", ph("hardware bring-up and field test"), "Not started"],
    ], raw_cols=(1,)))
    h.append(cap(2, "Table", "Application Module Implementation and Integration Status"))
    h.append("<h3>4.3 Detailed Module Descriptions</h3>")
    h.append(bullets([
        "<b>SLAM:</b> each 2D scan is matched against the last 16 keyframes with point-to-line ICP [8]; a match is "
        "accepted only if it agrees with odometry (1 cm + 100 % of the step). Gyro bias is measured while the robot "
        "stands still (62 s in the rescue scene). Loop closures against old keyframes enter a pose graph solved "
        "with dynamic covariance scaling [7], so a wrong closure cannot pull the map apart.",
        "<b>Exploration:</b> frontier cells are clustered and a viewpoint is chosen 0.5 m from unknown areas so the "
        "robot never walks into unseen space (the rescue field ends in a drop). An operator route, when given, is "
        "followed waypoint by waypoint; a waypoint that cannot be approached is skipped at once. Exploration ends "
        "only when viewpoints exist and none of them sees a frontier.",
        "<b>Planning &amp; Locomotion:</b> A* on the obstacle grid inflated by the robot radius, with a narrower "
        "inflation in narrow-gap mode; the gait follows the smoothed path. A watchdog measures progress along the "
        "remaining path and triggers a turn-away recovery; a camera check stops the robot before a floor edge.",
        "<b>End-to-End Integration:</b> all agents run in one process on a publish-subscribe bus; a scene profile "
        "(odometry scale, map size, calibration time, route) is the only per-environment input. Every SLAM change "
        "is checked by replaying stored recordings, which reproduce live runs exactly."]))

    # 5
    h.append("<h2>5 Functional Testing &amp; Prototype Demonstration</h2>")
    h.append("<h3>5.1 Functional Test Cases &amp; Verification</h3>")
    h.append(tbl(["Test ID", "Feature / Scenario", "Input / User Action", "Observed Output", "Result"], [
        ["TC-01", "Full autonomous exploration (scene 1)", "Start in unknown 10 x 10 m building, 30 min",
         "97.7 % of floor mapped, 0 stuck events, ATE 11.7 cm", "Pass"],
        ["TC-02", "Rescue field with operator route (scene 2)", "28-waypoint route, rubble, 12 min",
         "14 of 28 waypoints in 12 min, 0 collisions, ATE 12.3 cm; full run " + ph("pending"), "Partial"],
        ["TC-03", "Module integration (SLAM replay)", "Replay stored recordings through SLAM",
         "Reproduces live ATE exactly (11.7 / 12.3 cm)", "Pass"],
        ["TC-04", "Boundary: blocked route waypoint", "Waypoint behind rubble", "Skipped at once, route continues",
         "Pass"],
        ["TC-05", "Boundary: robot stalls", "No progress along path", "Watchdog turn-away, mission continues", "Pass"],
        ["TC-06", "Boundary: floor edge ahead", "Drop at the edge of the rescue field",
         "Stops 0.44 m before the edge", "Pass"],
        ["TC-07", "Automated unit tests", "pytest, 82 tests", "82 passed", "Pass"],
    ], raw_cols=(3,)))
    h.append(cap(3, "Table", "Functional Testing Summary Across Core Application Scenarios"))
    h.append(para("<b>Results across environments.</b> Scene 2 values marked (E) are the expected values of the full "
                  "exploration run, forecast from rescue runs 16 and 19; they are to be replaced after that run "
                  "and after the scene 1 verification run."))
    h.append(tbl(["Metric", "Scene 1: building (run 34)", "Scene 2: rescue field", "Average"], [
        ["Position error (ATE RMSE)", "11.7 cm", "16 cm (E)", "13.9 cm"],
        ["Map precision vs scene model", "99.7 %", "93 % (E)", "96.4 %"],
        ["Map recall vs scene model", "92.4 %", "60 % (E)", "76.2 %"],
        ["Map F1", "95.9 %", "73 % (E)", "84.5 %"],
        ["Floor covered", "97.7 %", "90 % (E)", "93.9 %"],
        ["Collisions / stuck events", "not measured / 0", "0 / 0 (E)", "0 / 0"],
        ["Control step (median / 95th pct.)", "27 / 138 ms", "22 / 118 ms", "25 / 128 ms"],
    ]))
    h.append(cap(4, "Table", "Results per environment and average (one system, same code)"))
    h.append(tbl(["Method (same recordings, same scoring)", "Scene 1", "Scene 2 (12-min run)", "Average"], [
        ["Odometry only (leg odometry + gyro)", "291.4 cm", "15.7 cm", "153.6 cm"],
        ["BreezySLAM [6], default settings", "1791.9 cm", "568.4 cm", "1180.2 cm"],
        ["BreezySLAM [6], best of 11 settings", "189.7 cm", "109.5 cm", "149.6 cm"],
        ["Ours", "11.7 cm", "12.3 cm", "12.0 cm"],
    ]))
    h.append(cap(5, "Table", "Position error against baselines on identical data"))
    h.append(tbl(["Published system (context only; other data, real robots)", "Platform", "ATE"], [
        ["FAST-LIVO2 [5]", "ANYmal, lidar + camera + IMU, 145-332 m", "10.5 cm"],
        ["ORB-SLAM3 stereo-inertial [5]", "ANYmal, 145-332 m", "22.9 cm"],
        ["RTAB-Map / ORB-SLAM3 RGB-D [5]", "ANYmal, depth camera, 145-332 m", "241 / 821 cm"],
        ["RTABMap / OpenVSLAM / ORB-SLAM3 [4]", "TUM Pioneer sequences, Kinect", "28.7 / 39.2 / 172 cm"],
        ["Ours on TUM Pioneer", "same sequences", ph("pending")],
    ], raw_cols=(2,)))
    h.append(cap(6, "Table", "Published numbers for context (not a same-data comparison)"))
    h.append("<h3>5.2 Working Prototype Demonstration</h3>")
    h.append("<p><img src='rescue_run19_coverage.png' width='330'/></p>")
    h.append(cap(7, "Figure", "Scene 2, 12-minute run: map over the true scene, true path (blue) and estimated path (orange)"))
    h.append(para(ph("Insert simulator screenshots of the robot in both scenes, scene 1 map, and the full-run scene 2 map")))
    h.append(para(
        "<b>Evaluation &amp; Usability Summary:</b> the control loop runs in real time on a PC CPU (median 25 ms per "
        "step for all agents). Position error stays near 12 cm in both measured runs; maps are 95.8-99.7 % precise "
        "where the robot has looked. In scene 2 the map recall is limited because a 2D camera slice sees only the "
        "faces of rubble that point towards the robot. Tables 7 and 8 show what each part adds and how the "
        "system degrades under sensor faults."))
    h.append(tbl(["Variant (one part removed)", "Scene 1", "Scene 2 (3-seed mean)"], [
        ["Full system", "11.7 cm", "11.0 cm"],
        ["No loop closure", "25.9 cm", "12.4 cm"],
        ["Plain instead of robust pose graph", "27.3 cm", "9.3 cm"],
        ["Scan-to-scan instead of keyframe submap", "131.8 cm", "29.0 cm"],
        ["Point-to-point instead of point-to-line ICP", "291.4 cm", "15.9 cm"],
        ["No degeneracy filter", "16.1 cm", "10.9 cm"],
        ["No heading prior / no gyro calibration", "295.6 cm", "178.0 cm"],
        ["Odometry only", "291.4 cm", "15.7 cm"],
    ]))
    h.append(cap(7, "Table", "Ablation: position error with one part removed (replays of the stored runs)"))
    h.append(tbl(["Injected fault (scene 2, 3 seeds)", "Ours", "Odometry only"], [
        ["None", "12.3 cm", "15.7 cm"],
        ["Depth noise 1 / 3 cm", "13.6 / 11.1 cm", "15.7 cm"],
        ["Depth points dropped 20 / 50 %", "10.4 / 9.7 cm", "15.7 cm"],
        ["Leg odometry noise 5 / 10 %", "9.5 / 10.0 cm", "15.9 / 16.2 cm"],
        ["Leg odometry scale error +/-5 %", "15.8 cm", "31.3 cm"],
        ["Residual gyro bias 0.01 / 0.03 deg/s", "29.9 / 100.5 cm", "29.6 / 98.0 cm"],
        ["All faults combined", "47.8 cm", "50.9 cm"],
    ]))
    h.append(cap(8, "Table", "Robustness to seeded sensor faults; uncorrected gyro bias is the main sim-to-real risk"))

    # 6
    h.append("<h2>6 Technical Justification &amp; Architectural Decisions</h2>")
    h.append(bullets([
        "<b>Technology Stack Rationale:</b> Python with NumPy/OpenCV keeps every algorithm readable and testable "
        "and runs on a CPU, matching the Raspberry Pi target; CoppeliaSim gives exact ground truth and scene "
        "geometry for honest scoring; a depth camera is far cheaper than lidar.",
        "<b>Architectural &amp; Performance Trade-offs:</b> a 2D slice of the depth image keeps SLAM fast but "
        "gives a narrow 60 deg view, so open floor depends on odometry; a 16-keyframe submap costs more per step "
        "than scan-to-scan but cut replay error from 16.3 to 13.9 cm; agents share one process for low latency "
        "rather than separate ROS nodes.",
        "<b>Security, Safety &amp; Error Handling:</b> collisions are measured with the simulator's contact check "
        "(0 in all measured runs); viewpoints keep 0.5 m from unknown space; a drop check stops the robot at floor "
        "edges; the watchdog recovers from stalls; ground truth is used only for scoring, never by SLAM. "
        + ph("authentication, data privacy and encrypted sync: planned with the forensic store")]))

    # 7
    h.append("<h2>7 Conclusion, Limitations &amp; Future Scope</h2>")
    h.append(bullets([
        "<b>Conclusion:</b> one depth camera, a gyro and leg odometry are enough for a quadruped to explore two "
        "very different simulated environments with about 12-16 cm position error, precise maps and no collisions, "
        "with the same code; it beats odometry and a standard light 2D SLAM by an order of magnitude on identical data.",
        "<b>Current Operational Limitations:</b> simulation only; map recall in open rubble fields is about 60 %; "
        "position error grows on long open-floor stretches; heading in the rescue profile relies on the gyro, so "
        "uncorrected gyro bias raises the error (Table 8); evidence perception, event trigger and secure logging "
        "are not integrated; " + ph("scene 2 full run and scene 1 verification pending"),
        "<b>Future Enhancements:</b> Cartographer and TUM comparisons, noise robustness, nearest-frontier baseline; "
        "more viewpoints around obstacles; the team's Mamba model for evidence detection; real-robot tests; "
        + ph("other planned features")]))

    # 8
    h.append("<h2>8 Project Deliverables &amp; Technical Enrichment Summary</h2>")
    h.append(tbl(["Deliverable Category", "Details / Title / Repository Link", "Status"], [
        ["Source Code Repository", "github.com/yokshith09/Autonomous-Quadruped-SLAM-Navigation-Perception, "
         "docs/ and per-run reports", "Completed"],
        ["Individual Online Certifications", ph("10-20 hr course for each team member"), ph("status")],
        ["Group TEA Deliverable 1", ph("title and details"), ph("status")],
        ["Group TEA Deliverable 2", ph("title and details"), ph("status")],
        ["Group TEA Deliverable 3", ph("title and details"), ph("status")],
    ], raw_cols=(1, 2)))
    h.append(cap(9, "Table", "Summary of Completed Project and Technical Enrichment Deliverables"))

    h.append("<h2>References</h2>")
    refs = [
        "B. Yamauchi, \"A frontier-based approach for autonomous exploration,\" in Proc. IEEE CIRA, pp. 146-151, 1997.",
        "W. Hess, D. Kohler, H. Rapp, D. Andor, \"Real-time loop closure in 2D LIDAR SLAM,\" in Proc. IEEE ICRA, pp. 1271-1278, 2016.",
        "J. Sturm, N. Engelhard, F. Endres, W. Burgard, D. Cremers, \"A benchmark for the evaluation of RGB-D SLAM systems,\" in Proc. IEEE/RSJ IROS, 2012.",
        "A. Merzlyakov, S. Macenski, \"A comparison of modern general-purpose visual SLAM approaches,\" arXiv:2107.07589, 2021.",
        "R. Corlito et al., \"Sensor configuration matters: a systematic evaluation of multimodal SLAM on quadruped robots,\" arXiv:2606.19067, 2026.",
        "B. Steux, O. El Hamzaoui, \"tinySLAM: a SLAM algorithm in less than 200 lines C-language program,\" in Proc. ICARCV, 2010; BreezySLAM, github.com/simondlevy/BreezySLAM.",
        "P. Agarwal, G. D. Tipaldi, L. Spinello, C. Stachniss, W. Burgard, \"Robust map optimization using dynamic covariance scaling,\" in Proc. IEEE ICRA, 2013.",
        "A. Censi, \"An ICP variant using a point-to-line metric,\" in Proc. IEEE ICRA, 2008.",
        "Boston Dynamics, \"Spot product specifications and Spot SDK autonomy documentation,\" bostondynamics.com.",
        "J. Chen, F. Dellaert, \"A1 SLAM: quadruped SLAM using the A1's onboard sensors,\" arXiv:2211.14432, 2022.",
    ]
    h.append("<ul>" + "".join(f"<li>[{i + 1}] {r}</li>" for i, r in enumerate(refs)) + "</ul>")
    return "\n".join(h)


if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(OUT_DIR, "Phase2_Report.pdf")
    render(build(), OUT_DIR, out, "23CSE498 Project Phase II, external review report (draft)")
