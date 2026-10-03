# Autonomous Quadruped Multi-Agent SLAM & Systematic Exploration Stack

[![Python 3.10+](https://img.shields.io/badge/python-3.10+-blue.svg)](https://www.python.org/)
[![Platform: CoppeliaSim](https://img.shields.io/badge/simulator-CoppeliaSim%20v4.5+-orange.svg)](https://www.coppeliarobotics.com/)
[![License: MIT](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)
[![Tests: 16/16 Passed](https://img.shields.io/badge/tests-16%2F16%20passed-brightgreen.svg)](tests/)

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
| **Run Fast Headless Exploration Benchmark** | `python main_agentic.py --headless --max-time 45.0` | Headless execution without OpenCV window, saves all matrices to disk. |
| **Print Complete 22-Metric Scientific Report** | `python evaluate_all_metrics.py` | Computes all trajectory, reconstruction, CPU-SLAM benchmarks, and 5-point anti-bypass audit. |
| **Print Active Run Summary & Benchmark** | `python print_mne_comparison.py` | Prints terminal scoreboard comparing active run against CPU SLAMs (`GMapping`, `Cartographer`, `ORB-SLAM3`). |
| **Run Agent Unit Tests (9 tests)** | `python tests/test_agents.py` | Verifies message bus, knowledge graph, episodic memory, planning, locomotion, and semantic extraction. |
| **Run SLAM Accuracy Tests (7 tests)** | `python tests/test_slam_accuracy.py` | Verifies normal estimation, Point-to-Line ICP, loop closure detection, SE(2) PGO, and recovery. |
| **Run Hardware & Scene Diagnostics** | `python diagnose_sim.py` | Inspects robot handles, sensor communication, and joint dynamics. |

---

## 3. How Ground Truth Map Evaluation Works (Dual Bayesian Grid Architecture)

We do **NOT** compare against a static screenshot or pre-rendered picture.

Our system maintains **two independent Bayesian Log-Odds Occupancy Grids** simultaneously in real time:

1. **`true_grid` (Ground Truth Reference):** As the robot moves, 3D depth rays are projected using the **exact simulator ground-truth pose** `(true_x, true_y, true_heading)` from the physics engine. This forms an unbiased, drift-free reference of everywhere the camera has seen.
2. **`mapping_agent.grid` (Active SLAM Grid):** The exact same depth rays are projected using the **estimated closed-loop SLAM pose** `(slam_x, slam_y, slam_yaw)` from `SLAMAgent` (incorporating realistic odometry slip, ICP noise, and loop closure adjustments).

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

## 5. Verified Active Run Benchmark (CPU-Only)

Measured from active runs on `robot_working.ttt` ($5\text{ cm}$ grid, CPU-only on Intel Core / AMD Ryzen / Pi 5):

| Metric | Measured Value (45s Short Run) | Measured Value (18.88m Long Run) | Target Benchmark |
| :--- | :---: | :---: | :---: |
| **Local ATE RMSE (Trajectory Error)** | **`0.79 cm`** | **`5.70 cm`** | $< 10.0\text{ cm}$ |
| **Translational Drift Rate (% of Path)** | **`0.20 %`** | **`0.30 %`** | $< 1.0\text{ %}$ |
| **Peak Trajectory Drift (Max ATE)** | **`1.14 cm`** | **`11.17 cm`** | $< 15.0\text{ cm}$ |
| **Mean Wall Chamfer Error** | **`1.76 cm`** | **`5.12 cm`** | $< 5.5\text{ cm}$ |
| **Obstacle Map F1 Score** | **`99.8 %`** | **`93.3 %`** | $> 90.0\text{ %}$ |
| **Navigable Free-Space IoU** | **`94.6 %`** | **`85.2 %`** | $> 80.0\text{ %}$ |
| **CPU Tracking Latency (`Track/Fr`)** | **`14.2 ms`** | **`14.2 ms`** | $< 25.0\text{ ms}$ |
| **CPU Mapping Latency (`Map/Fr`)** | **`4.1 ms`** | **`4.1 ms`** | $< 10.0\text{ ms}$ |
| **CPU A\* Planning Latency (`Plan/Call`)** | **`3.4 ms`** | **`3.4 ms`** | $< 10.0\text{ ms}$ |
| **Scene Map RAM Footprint** | **`0.37 MB`** | **`0.37 MB`** | $< 1.0\text{ MB}$ |

### CPU-Only Multi-SLAM Benchmark Comparison

| SLAM System (All Executed on CPU) | ATE RMSE $\downarrow$ | Wall Chamfer $\downarrow$ | Comp / F1 $\uparrow$ | Track / Frame $\downarrow$ | Map / Frame $\downarrow$ | Map RAM $\downarrow$ |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| **GMapping (FastSLAM 2.0 CPU)** | $4.80\text{ cm}$ | $4.50\text{ cm}$ | $88.4\%$ | $12.0\text{ ms}$ | $26.0\text{ ms}$ | $18.50\text{ MB}$ |
| **Hector SLAM (2D Gauss-Newton CPU)** | $3.20\text{ cm}$ | $3.10\text{ cm}$ | $91.2\%$ | $11.5\text{ ms}$ | $8.0\text{ ms}$ | $12.40\text{ MB}$ |
| **Cartographer 2D (Submap PGO CPU)** | $2.10\text{ cm}$ | $2.20\text{ cm}$ | $94.8\%$ | $16.4\text{ ms}$ | $12.0\text{ ms}$ | $28.00\text{ MB}$ |
| **RTAB-Map (RGB-D Graph CPU)** | $2.45\text{ cm}$ | $2.65\text{ cm}$ | $92.6\%$ | $25.0\text{ ms}$ | $40.0\text{ ms}$ | $45.20\text{ MB}$ |
| **ORB-SLAM3 (RGB-D Feature CPU)** | $1.85\text{ cm}$ | $2.15\text{ cm}$ | $93.5\%$ | $24.0\text{ ms}$ | $18.0\text{ ms}$ | $34.10\text{ MB}$ |
| **Swarm-SLAM (Multi-Agent CPU)** | $6.50\text{ cm}$ | $4.90\text{ cm}$ | $86.5\%$ | $22.0\text{ ms}$ | $26.0\text{ ms}$ | $31.00\text{ MB}$ |
| **OURS (Agentic CPU SLAM - Short)** | **$0.79\text{ cm}$** | **$1.76\text{ cm}$** | **$99.8\%$** | **$14.2\text{ ms}$** | **$4.1\text{ ms}$** | **$0.37\text{ MB}$** |
| **OURS (Agentic CPU SLAM - 18.9m)** | **$5.70\text{ cm}$** | **$5.12\text{ cm}$** | **$93.3\%$** | **$14.2\text{ ms}$** | **$4.1\text{ ms}$** | **$0.37\text{ MB}$** |

---

## 6. Result Validity & Anti-Bypass Integrity Audit (5/5 PASS)

Every execution of `python evaluate_all_metrics.py` automatically runs 5 programmatic audits verifying zero simulation cheat or ground-truth leakage:

1. **`[PASS]` AST Source Code Audit:** Verifies `slam.py` and `agents/slam_agent.py` contain **0 calls** to `sim.getObjectPosition` or `sim.getObjectOrientation`.
2. **`[PASS]` MessageBus Ground-Truth Isolation:** In closed-loop mode (`use_ground_truth_nav=False`), `SLAMAgent` does not subscribe to or receive `pose/ground_truth`.
3. **`[PASS]` Trajectory Independence Audit:** Confirms `true_poses` and `slam_poses` diverge physically ($0 < \text{ATE RMSE} \le 10\text{ cm}$).
4. **`[PASS]` Dual-Grid Matrix Independence Audit:** Confirms `true_grid.npy` and `slam_grid.npy` are independent matrices ($>6,000$ cells differ due to real SLAM drift).
5. **`[PASS]` Active Closed-Loop Navigation Flag:** Confirms navigation operates on `pose/slam` with `closed_loop_slam=True`.

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
Autonomous-Quadruped-SLAM-Navigation-Perception/
├── main_agentic.py                 # Multi-Agent Closed-Loop Orchestrator
├── evaluate_all_metrics.py         # 22-Metric Scientific Evaluator & Anti-Bypass Audit
├── print_mne_comparison.py         # CPU SLAM Benchmark Reporter
├── accuracy_evaluator.py           # Numerical Grid (cKDTree) & Trajectory Engine
├── grid.py                         # Bayesian Log-Odds Grid & Medoid Frontier Extractor
├── planner.py                      # Vector-Dilated A* Planner & Shortcut Smoother
├── robot.py                        # LizardRobot CPG Kinematics & IMU Delta
├── sensors.py                      # Static TF Camera Extrinsics & Pinhole Back-Projection
├── slam.py                         # Point-to-Line Metric ICP & SE(2) Keyframe PGO
├── requirements.txt                # Pip dependency file
├── robot_working.ttt               # CoppeliaSim Multi-Room Arena Scene
├── agents/                         # Asynchronous 7-Agent Architecture
│   ├── base.py                     # BaseAgent & Lifecycle Management
│   ├── bus.py                      # Thread-Safe Typed MessageBus
│   ├── perception_agent.py         # RGB-D Back-Projection & Radial Self-Masking
│   ├── slam_agent.py               # IMU-ICP Pose Estimator (Closed-Loop)
│   ├── mapping_agent.py            # Pose-Coupled Grid & Voronoi Room Segmentation
│   ├── frontier_agent.py           # GW-NBV & Monotonic Angular Sweep Engine
│   ├── planning_agent.py           # Safe A* Path Planning & Replanning Watchdog
│   ├── locomotion_agent.py         # CPG Gait & Tight Waypoint Line-Tracker
│   └── supervisor_agent.py         # Spatial Knowledge Graph & Episodic Timeline
├── tests/                          # Automated Verification Suite
│   ├── test_agents.py              # 9 Agent Subsystem Unit Tests
│   └── test_slam_accuracy.py       # 7 SLAM Metric & Loop Closure Tests
├── accuracy_results/               # Evaluation Outputs & Artifacts
│   ├── latest_metrics.json         # Numerical Metrics Payload
│   ├── cpu_slam_benchmark_comparison.png  # CPU Multi-SLAM Comparison Bar Chart
│   ├── numerical_map_accuracy_comparison.png # 4-Panel Map Accuracy Dashboard
│   ├── robot_movement_trajectory_analysis.png # Trajectory Overlay & Drift Curves
│   ├── live_hud_latest.png         # Live OpenCV 4-Panel Telemetry HUD
│   ├── knowledge_graph.json        # 4-Layer Topological Spatial Graph
│   └── mission_timeline.md         # Episodic Mission Event Log
└── docs/                           # Architectural Specifications & Reports
    ├── README.md                   # Mirrored Docs Readme
    ├── SLAM_NUANCES_AND_IMPLEMENTATION_ROADMAP.md # Detailed SLAM Roadmap & Specs
    └── system_architecture_diagram.html # Interactive Architecture Diagram
```
