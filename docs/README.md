# Autonomous Quadruped Multi-Agent SLAM & Exploration Stack

Closed-loop 2D/3D SLAM, topological Voronoi room segmentation, semantic entity extraction (`chair` / `wall`), and hierarchical room-by-room exploration for a quadruped robot in CoppeliaSim (`robot_working.ttt`).

---

## 1. Setup on a New PC (Clone & Install)

```powershell
git clone <repo-url>
cd Agentic_pipeline
pip install -r requirements.txt
```

Open **CoppeliaSim EDU v4.5+** and load `robot_working.ttt`.

---

## 2. Quick Start Commands

| Task | Command |
| :--- | :--- |
| **Run Unit Test Suite (16 tests)** | `python -m unittest discover tests -v` |
| **Run Closed-Loop Multi-Agent SLAM (GUI + Terminal)** | `python main.py` |
| **Run Closed-Loop Headless Benchmark** | `python main.py --headless --max-time 30.0` |
| **Print MNE-SLAM (CVPR 2025) Benchmark Table** | `python print_mne_comparison.py` |
| **Re-evaluate Saved Grids & Trajectories** | `python accuracy_evaluator.py` |
| **Run Hardware / Scene Diagnostics** | `python diagnose_sim.py` |

---

## 3. Architecture & Documentation (`docs/`)

- **`sensors.py`** — Calibrates static camera-to-base extrinsics $T_{\text{base}}^{\text{cam}}$ once at startup and back-projects depth frames directly into the robot body frame (`depth_to_base_points`).
- **`slam.py`** — Point-to-Line Metric ICP fused with IMU gyro/gait odometry priors (`ICPPoseEstimator`) and harmonic SE(2) Pose Graph Optimization (`KeyframeLoopManager.optimize_pose_graph`).
- **`agents/`** — 7 asynchronous agents communicating over `MessageBus`:
  1. `PerceptionAgent`: Body-frame 3D/2D obstacle slicing (`z ∈ [0.04m, 0.60m]`, `r > 0.40m`).
  2. `SLAMAgent`: Closed-loop pose estimation (`use_ground_truth_nav=False` by default).
  3. `MappingAgent`: Pose-coupled Bayesian log-odds grid, Voronoi room segmentation, and `"chair"` / `"wall"` semantic entity extraction.
  4. `FrontierAgent`: Hierarchical intra-room sweep and doorway transition state machine.
  5. `PlanningAgent`: Safety-inflated $A^*$ path planner with line-of-sight smoothing.
  6. `LocomotionAgent`: Sinusoidal CPG quadruped gait, reactive obstacle repulsion, and anti-stuck recovery.
  7. `SupervisorAgent`: 4-layer `SpatialKnowledgeGraph` (`query_memory`) and `EpisodicMemory` custodian.

All specifications, reports, and architecture diagrams are organized under [`docs/`](docs/):
- [`docs/SLAM_NUANCES_AND_IMPLEMENTATION_ROADMAP.md`](docs/SLAM_NUANCES_AND_IMPLEMENTATION_ROADMAP.md)
- [`docs/system_architecture_diagram.html`](docs/system_architecture_diagram.html)
- [`docs/panel2_report.pdf`](docs/panel2_report.pdf)
- [`docs/panel_review2_ppt.pdf`](docs/panel_review2_ppt.pdf)
