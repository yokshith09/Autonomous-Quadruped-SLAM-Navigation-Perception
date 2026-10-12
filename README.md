# Autonomous Quadruped Multi-Agent SLAM & Systematic Exploration Stack

[![Python 3.10+](https://img.shields.io/badge/python-3.10+-blue.svg)](https://www.python.org/)
[![Platform: CoppeliaSim](https://img.shields.io/badge/simulator-CoppeliaSim%20v4.5+-orange.svg)](https://www.coppeliarobotics.com/)
[![License: MIT](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)
[![Tests: 18](https://img.shields.io/badge/tests-18-blue.svg)](tests/)

Closed-loop, CPU-only 2D/3D SLAM, topological Voronoi room segmentation, semantic entity extraction (`chair` / `wall`), and hierarchical room-by-room exploration for a legged quadruped in CoppeliaSim (`robot_working.ttt`).

---

## 1. Setup on a New PC (Clone & Install)

```powershell
git clone https://github.com/yokshith09/Autonomous-Quadruped-SLAM-Navigation-Perception.git
cd Autonomous-Quadruped-SLAM-Navigation-Perception
pip install -r requirements.txt
```

Open **CoppeliaSim EDU v4.5+** and load `robot_working.ttt`.

---

## 2. Quick Start Commands

| Task | Command | Description |
| :--- | :--- | :--- |
| **Run Closed-Loop Multi-Agent Exploration** | `python main_agentic.py` | Main entrypoint: runs full 7-agent closed-loop SLAM, live HUD, and autonomous room-by-room exploration. |
| **Run Headless Exploration** | `python main_agentic.py --headless --max-time 1800` | No OpenCV window; stops when no reachable frontier is left (all rooms explored) or at the time limit, then evaluates. |
| **Re-evaluate the Last Run** | `python evaluate_all_metrics.py` | Recomputes trajectory and map metrics from `captured_matrices/`, compares against the published MNE-SLAM (CVPR 2025) tables, and runs the ground-truth leakage audit. |
| **Record a SLAM Dataset** | `python main_agentic.py --headless --record recordings/run.npz` | Saves exactly what SLAM receives each step (scan, gait odometry, gyro) plus ground truth for scoring. |
| **Replay SLAM Offline** | `python slam_replay.py recordings/run.npz --set icp_yaw_weight=0.2` | Re-runs SLAM on a recording in seconds and prints ATE / drift; use it to compare SLAM variants on identical data. |
| **Build the Shareable Benchmark Report** | `python build_report.py` | Writes `reports/benchmark_report.pdf`: our run vs. published quadruped SLAM, efficiency and capability benchmarks, each row with its source (needs Microsoft Word for the PDF export). |
| **Run Agent Unit Tests (9 tests)** | `python tests/test_agents.py` | Verifies message bus, knowledge graph, episodic memory, planning, locomotion, and semantic extraction. |
| **Run SLAM Accuracy Tests (9 tests)** | `python tests/test_slam_accuracy.py` | Verifies normal estimation, Point-to-Line ICP, loop closure (incl. rotated keyframes), degeneracy filtering, loop-error distribution, and recovery. |
| **Run Hardware & Scene Diagnostics** | `python diagnose_sim.py` | Inspects robot handles, sensor communication, and joint dynamics. |

---

## 3. How Ground Truth Map Evaluation Works (Dual Bayesian Grid Architecture)

We do **NOT** compare against a static screenshot or pre-rendered picture.

Our system maintains **two independent Bayesian Log-Odds Occupancy Grids** simultaneously in real time:

1. **`true_grid` (Ground Truth Reference):** As the robot moves, 3D depth rays are projected using the **exact simulator ground-truth pose** `(true_x, true_y, true_heading)` from the physics engine. This forms an unbiased, drift-free reference of everywhere the camera has seen.
2. **`mapping_agent.grid` (Active SLAM Grid):** The exact same depth rays are projected using the **estimated closed-loop SLAM pose** `(slam_x, slam_y, slam_yaw)` from `SLAMAgent` (fusing Point-to-Line ICP with dead-reckoned gait odometry and a noisy simulated gyro; the simulator pose is never read by the estimator).

At evaluation time, both are saved as raw numerical float32 matrices (`captured_matrices/true_grid.npy` and `captured_matrices/slam_grid.npy`, $220 \times 220$ cells at $5\text{ cm}$ resolution). [`accuracy_evaluator.py`](accuracy_evaluator.py) evaluates:
- **Mean Wall Chamfer Distance (cm):** Bidirectional Euclidean nearest-neighbor distance (via `scipy.spatial.cKDTree`).
- **95th-Percentile Hausdorff Distance (cm):** Worst-case geometric wall outlier error.
- **Obstacle F1 Score, Precision, Recall, and IoU (%):** Strict binary threshold matches at $\pm 15\text{ cm}$ spatial margin.

---

## 4. Core Algorithmic Novelties in Our Project

Standard frontier exploration (Yamauchi 1997 / Nearest Euclidean Frontier) was built for $360^\circ$ planar LiDARs on wheeled robots. On a quadruped with a forward-facing $57^\circ$ RGB-D camera, standard algorithms suffer from **front-to-back spinning oscillations** and **wall-crossing geodesic traps**. 

We invented and implemented **GW-NBV (Geodesic-Wavefront Topological Next-Best-View)** in [`agents/frontier_agent.py`](agents/frontier_agent.py):

1. **Obstacle-Inflated Geodesic Wavefront Cost-to-Go:** Computes an 8-connected BFS Geodesic Distance Field $D_{\text{geo}}(x, y)$ over the dilated free-space grid in $< 2\text{ ms}$. Frontiers behind solid walls ($D_{\text{geo}} = \infty$) are pruned *before* scoring, eliminating `PATH_UNREACHABLE` deadlocks.
2. **Hemisphere-First Monotonic Angular Sweep:** As long as reachable frontiers exist in the forward hemisphere ($|\Delta\theta| \le 95^\circ$), $180^\circ$ turn reversals are heavily penalized (`-18.0`). The robot sweeps walls monotonically in one rotational direction (CW or CCW) before turning around.
3. **Dynamic View-Cone Sliding:** When the Kinect camera's raycasting clears a committed target 1.5m ahead before the robot physically arrives, the target slides forward along the receding frontier in the same view cone rather than thrashing to an opposite wall.
4. **Doorway Penetration Look-Ahead Vector:** Projects transit waypoints $0.55\text{ m}$ *past* the doorway threshold along the room transit vector $\hat{\mathbf{u}}_{r \to d}$, pulling the robot cleanly into the next room without stalling on room boundaries.

---

## 5. Results

The shareable comparison is [`reports/benchmark_report.pdf`](reports/benchmark_report.pdf),
regenerated with `python build_report.py` after a run. Earlier README tables were removed: the odometry fed to SLAM
was derived from the simulator's true position, and the "CPU multi-SLAM" and ablation rows were hard-coded.

After a full exploration run, `accuracy_results/full_evaluation_report.txt` contains:
1. Localization metrics (ATE RMSE / mean / median / max, RPE, drift rate, heading error).
2. Mapping metrics (Acc., Comp., Comp. ratio < 5 cm, Chamfer, Hausdorff-95, wall F1, IoU).
3. Mission and CPU runtime (time to clear all frontiers, rooms, tracking/mapping/planning latency, memory).
4. Our row placed next to the published tables of MNE-SLAM (Deng et al., CVPR 2025; Tables 1, 2, 3, 5),
   with the caveat that those systems were evaluated on Replica / Apartment / ScanNet, not on our arena.

---

## 6. Result Validity Audit

`evaluate_all_metrics.py` runs these checks on every evaluation:

1. **Static audit:** `slam.py` and the SLAM, mapping, perception, frontier and planning agents contain no call to
   `sim.getObjectPosition / Orientation / Matrix / Pose / Velocity`; the odometry function in `robot.py` reads no position
   (its simulated gyro uses only the per-step orientation delta plus bias and noise).
2. **Runtime audit:** a closed-loop `SLAMAgent` is instantiated and its bus subscriptions are checked for `pose/ground_truth`.
3. **Trajectory independence:** SLAM and ground-truth trajectories must differ.
4. **Dual-grid independence:** `true_grid.npy` and `slam_grid.npy` must differ.
5. **Closed-loop flag:** the run must have navigated on `pose/slam`.

Known simulation shortcut: `LizardRobot.steer_heading` turns the body by setting its orientation directly (actuation,
not sensing). The turn amount is computed from the SLAM heading error.

---

## 7. Adding More Intelligence: Market Comparison & Next-Gen Roadmap

| Industry Pillar | Current Market / Top Labs (Spot, ANYmal, Go2, CVPR'25) | Our System Today | High-Impact Roadmap Upgrades |
| :--- | :--- | :--- | :--- |
| **Semantic Understanding** | Open-Vocabulary 3D Scene Graphs (Hydra, ConceptFusion, SayPlan). | Geometric heuristic spans (`chair`, `wall`). | Plug a lightweight VLM / YOLOv8-nano into `PerceptionAgent` to extract semantic class embeddings. |
| **Exploration Strategy** | Semantic Affordance Exploration (prioritize corridors over dead-end closets). | GW-NBV volumetric unknown information gain + geodesic cost. | Add semantic doorway prior weighting in `FrontierAgent` to explore primary circulation routes first. |
| **Executive Reasoning** | Asynchronous LLM Supervisor (LangChain, AutoGPT, ROS-LLM). | Deterministic state machine in `SupervisorAgent`. | Natural language voice/text query interface (*"Find chairs in Room 2"*). |
| **Scene Representation** | 3D Gaussian Splatting SLAM (SplaTAM, Photo-SLAM). | 2D Bayesian Log-Odds Grid ($0.37\text{ MB}$). | 2.5D Multi-Layer Elevation Grid (distinguish tables you can walk under vs. solid walls). |
| **Locomotion** | RL Locomotion (Isaac Gym / Walk These Ways). | Kinematic biomimetic sinusoidal CPG gait. | Dynamically modulate step lift height when traversing door sills. |

---

## 8. Repository Structure

```text
├── main_agentic.py            # Entry point: wires the 7 agents to CoppeliaSim, runs the mission, saves the run
├── main.py                    # Alias for main_agentic.py
├── evaluate_all_metrics.py    # Recomputes all metrics from the saved run + ground-truth leakage audit
├── slam_replay.py             # Offline SLAM benchmark on a recorded input stream
├── build_report.py            # Builds reports/benchmark_report.pdf
├── benchmarks.py              # Published benchmark numbers, each tied to its source
├── accuracy_evaluator.py      # Grid (Acc/Comp/Chamfer/F1/IoU) and trajectory (ATE/RPE) metrics
├── generate_separate_maps.py  # Localization, SLAM, ground-truth and overlay map figures
├── slam.py                    # Point-to-line ICP, gyro/odometry fusion, keyframe loop closure
├── grid.py                    # Bayesian log-odds occupancy grid and frontier extraction
├── planner.py                 # Inflated A* and line-of-sight path smoothing
├── robot.py                   # Quadruped gait, steering, and proprioceptive odometry (no ground truth)
├── sensors.py                 # Camera setup, depth back-projection into the robot frame
├── viz.py                     # Live 4-panel HUD
├── diagnose_sim.py            # Scene / sensor / joint diagnostics
├── robot_working.ttt          # CoppeliaSim multi-room arena
├── agents/                    # Perception, SLAM, Mapping, Frontier, Planning, Locomotion, Supervisor + message bus
├── tests/                     # 9 agent tests + 9 SLAM tests
├── reports/                   # Shareable benchmark report (pdf)
└── docs/                      # Panel review report/slides, architecture diagram, SLAM roadmap
```

Generated on every run (git-ignored): `captured_matrices/` (raw grids and trajectories), `accuracy_results/`
(metrics JSON, report text, knowledge graph, mission timeline), `evaluation_results_images/` (figures).
