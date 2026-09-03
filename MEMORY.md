# 🧠 PROJECT MEMORY: Autonomous Quadruped SLAM, Navigation & Perception

> **Repository:** [yokshith09/Autonomous-Quadruped-SLAM-Navigation-Perception](https://github.com/yokshith09/Autonomous-Quadruped-SLAM-Navigation-Perception.git)  
> **Target Platform:** 6-DOF Biomimetic Quadruped ("Lizard" Model)  
> **Simulation Engine:** CoppeliaSim via ZeroMQ Remote API (`coppeliasim_zmqremoteapi_client`)  
> **Active Codebase Location:** `e:\project_phase1\project_phase1\active_version`  
> **Last Memory Consolidation Date:** September 2026  

---

## 1. Executive Project Summary

This project delivers an end-to-end autonomous navigation, perception, and simultaneous localization and mapping (SLAM) stack for a bio-inspired quadruped robot operating in unknown, GPS-denied indoor environments (e.g., search-and-rescue, disaster response, hostage scenarios).

Unlike wheeled robots operating on planar surfaces, legged robots introduce high-frequency roll, pitch, and yaw oscillations during gait cycles. This system overcomes these physical challenges through an **IMU-stabilized perception pipeline**, **SVD-based 2D Iterative Closest Point (ICP) scan matching**, a **recursive Bayesian log-odds occupancy grid**, a **novelty-weighted autonomous frontier exploration policy**, and a **vector-dilated $A^*$ path planner** operating at sub-$4\text{ms}$ latency.

Furthermore, the project includes a rigorous **numerical benchmark engine** (`accuracy_evaluator.py`, `map_accuracy.ipynb`) that shifts evaluation from deceptive 2D image screenshots to matrix-level ground truth comparisons (calculating Tolerant IoU, F1-Score, Chamfer Wall Distance, Hausdorff Error, and Absolute Trajectory Error RMSE).

---

## 2. Chronological Evolution & "Brain" History

The development trajectory of this system evolved through three major technological stages, recorded across research sessions, previous conversation transcripts (`37477328-9a43-48ad-bd6c-a8bf9e3daeca`, `700f13a3-183b-43f3-bbf9-e04912249482`), and historical project context:

```
┌───────────────────────────────┐
│           STAGE 1             │
│    Static A* Navigation       │
│  (path_planning_nav.py)       │
└───────────────┬───────────────┘
                │
                ▼
┌───────────────────────────────┐
│           STAGE 2             │
│    D* Lite & Depth Rays       │
│(path_planning_nav_dstar.py)   │
└───────────────┬───────────────┘
                │
                ▼
┌───────────────────────────────┐
│           STAGE 3             │
│   Active SLAM & Evaluation    │
│(active_version / main.py)     │
└───────────────────────────────┘
```

### Stage 1: Static A* Navigation (`path_planning_nav.py`)
- **Mechanism:** Loaded static obstacle bounding boxes directly from CoppeliaSim scene objects (`static_map.py`) to construct an offline 2D obstacle grid. Planned a single global A* path to a fixed goal $(-4.60\text{m}, -3.93\text{m})$ and executed walking locomotion.
- **Achievements:** Verified joint kinematics, locomotion stability, and initial Kinect depth extraction at 25Hz. Reached goal in 116 seconds over 42 waypoints.
- **Fatal Limitations:** Zero autonomous exploration; no online mapping; zero adaptation to dynamic or unmodeled obstacles; dependent entirely on ground-truth simulator coordinates.

### Stage 2: Incremental D* Lite & Depth Raycasting (`path_planning_nav_dstar.py`)
- **Mechanism:** Introduced real-time depth raycasting into a 3-state grid (`UNKNOWN=0, FREE=1, OCCUPIED=2`) and integrated incremental D* Lite path planning to dynamically route toward frontier boundaries.
- **Discovered Bottlenecks & Failure Modes:**
  1. **Computational Freezes:** Recomputing D* Lite priority queues on raw sensor updates took $>1500\text{ms}$ per iteration, triggering ZeroMQ communication timeouts and halting the physics simulation.
  2. **Self-Body Occlusion Lock:** The downward-angled Kinect depth sensor captured the robot's own oscillating front legs and torso. These points were registered as obstacles directly in front of the robot, locking it into an immediate collision halt.
  3. **Floor Noise Distortions:** Small robot pitch tilts caused the floor plane to intersect the obstacle detection band, turning open corridors into impassable obstacle walls.
  4. **Perpetual Corner Bouncing:** The wall blacklist was flushed every 4 steps, causing the robot to bounce back and forth in dead-ends without making forward exploratory progress.

### Stage 3: Modular Active SLAM & Quantitative Evaluation (Current `active_version`)
- **Mechanism:** Complete architectural overhaul into modular, decoupled Python engines.
- **Breakthrough Solutions Implemented:**
  - **IMU-Stabilized Gravity Alignment:** Stripped quadruped roll/pitch oscillations before scan matching.
  - **Vectorized Dilation & $A^*$ Acceleration:** Replaced D* Lite with a vectorized morphological obstacle inflation cache, dropping planning latency from $>1500\text{ms}$ to **$3.5\text{ms}$** (a $400\times$ speedup).
  - **Spatial Filtering:** Enforced a $40\text{cm}$ radial cylindrical self-mask (erasing robot body returns) and a strict 3D vertical band ($0.04\text{m} \le z \le 0.60\text{m}$) eliminating floor and ceiling clutter.
  - **Dual-Grid SLAM Architecture:** Maintained ground-truth simulator grid alongside an independent drifting ICP SLAM grid to evaluate true mapping errors.
  - **Numerical Ground-Truth Benchmark:** Eliminated lossy PNG/JPEG image comparisons, replacing them with direct float array analysis in `accuracy_evaluator.py` and `map_accuracy.ipynb`.

---

## 3. System Architecture & Component Inventory

```
active_version/
├── robot_working.ttt                # 3D CoppeliaSim simulation scene
├── main.py                          # Master mission loop & exploration orchestrator
├── evaluate_slam.py                 # Evaluation runner with automated trajectory logging
├── robot.py                         # Kinematic interface, gait engine & steering
├── sensors.py                       # Kinect RGB-D driver, pinhole back-projection, filtering
├── slam.py                          # 2D Point-to-Point ICP scan matcher & SVD solver
├── grid.py                          # Bayesian log-odds occupancy grid & frontier detector
├── planner.py                       # Vector-dilated A* planner & shortcut smoother
├── viz.py                           # 4-panel real-time OpenCV telemetry HUD
├── accuracy_evaluator.py            # Quantitative benchmark engine (IoU, F1, ATE, Chamfer)
├── diagnose_sim.py                  # Standalone 4-point CoppeliaSim diagnostic script
├── map_accuracy.ipynb               # 5-stage interactive Jupyter benchmark suite
├── true_map.jpeg / slam_map.png     # Reference export maps
├── captured_matrices/               # Serialized NumPy data, raw depth arrays, step heatmaps
└── accuracy_results/                # Evaluation output reports, confusion overlays, plots
```

### Component Breakdown

| Module | Core Responsibility | Key Classes & Functions | Key Parameters |
| :--- | :--- | :--- | :--- |
| [`main.py`](file:///e:/project_phase1/project_phase1/active_version/main.py) | Mission orchestration, sensor scheduling, frontier goal dispatch, CoppeliaSim 3D drawing | `main()`, `init_traveled_path()`, `draw_new_occupied_cells()` | `MAX_TIME=900s`, `DEPTH_FREQ=2` (100ms), `FRONTIER_FREQ=40` (2s), `REPLAN_COOLDOWN=1.0s` |
| [`robot.py`](file:///e:/project_phase1/project_phase1/active_version/robot.py) | Low-level joint actuation, rhythmic sinusoidal gait, proportional heading control | `LizardRobot`, `apply_gait()`, `steer_heading()`, `patch_coppelasim_scripts()` | $f = 5.0\text{ Hz}$, $A_{\text{hip}} = 0.52\text{ rad}$, $A_{\text{leg}} = 0.35\text{ rad}$, $K_p = 1.2$ |
| [`sensors.py`](file:///e:/project_phase1/project_phase1/active_version/sensors.py) | Kinect RGB-D streaming, pinhole 3D back-projection, spatial height & radial self-masking | `initialize_sensors()`, `grab_depth_frame()`, `depth_to_world_points()` | $128 \times 128$, $\text{FOV}=60^\circ$, $f=110.85\text{px}$, $z \in [0.04, 0.60]\text{m}$, $r_{\text{mask}} = 0.40\text{m}$ |
| [`slam.py`](file:///e:/project_phase1/project_phase1/active_version/slam.py) | 2D Point-to-Point ICP scan matching, SVD closed-form rigid transform, pose integration | `ICPPoseEstimator`, `icp_match()`, `best_fit_transform()`, `downsample_scan()` | Voxel downsample $= 5\text{cm}$, max iter $= 20$, gate dist $= 0.35\text{m}$, fallback to odometry |
| [`grid.py`](file:///e:/project_phase1/project_phase1/active_version/grid.py) | 2D Bayesian log-odds occupancy grid, 2D Bresenham raycasting, morphological frontier extraction | `BayesianOccupancyGrid`, `bresenham_line()`, `detect_frontiers()`, `to_debug_image()` | $221 \times 221$, res $= 0.05\text{m}$, range $= [-5.5, +5.5]\text{m}$, $l_{\text{occ}}=+0.85$, $l_{\text{free}}=-0.40$ |
| [`planner.py`](file:///e:/project_phase1/project_phase1/active_version/planner.py) | Vector-dilated $A^*$ search with 8-connectivity, obstacle safety inflation, path smoother | `a_star_search()`, `smooth_path()`, `_line_of_sight()` | Inflation radius $= 5\text{ cells}$ ($25\text{cm}$), unknown space penalty $= 2.5$, waypoint radius $= 0.20\text{m}$ |
| [`viz.py`](file:///e:/project_phase1/project_phase1/active_version/viz.py) | Real-time $800 \times 800$ 4-panel diagnostic dashboard | `create_debug_dashboard()` | Panels: RGB Feed, JET Colormap Depth Heatmap, Occupancy Grid with Path/Frontiers, Live Telemetry |
| [`accuracy_evaluator.py`](file:///e:/project_phase1/project_phase1/active_version/accuracy_evaluator.py) | Quantitative verification suite for maps, trajectories, and perception fidelity | `evaluate_grid_accuracy()`, `evaluate_trajectory()`, `evaluate_depth_perception()` | Tolerant IoU ($15\text{cm}$ tolerance), Chamfer Wall Distance, 95% Hausdorff, ATE RMSE, RPE Drift % |
| [`diagnose_sim.py`](file:///e:/project_phase1/project_phase1/active_version/diagnose_sim.py) | Standalone verification script checking scene parenting, joint motor torque, walking physics | `main()` | 4-point automated diagnostic checklist |

---

## 4. Mathematical Formulations & Algorithms

### 4.1 Pinhole Camera Back-Projection
For each depth pixel $(u, v)$ with measured depth $d(u,v) = z_c \in [0.12\text{m}, 3.5\text{m}]$, the 3D optical camera frame coordinates are:
$$x_c = -\frac{u - c_x}{f_x} \cdot z_c, \quad y_c = \frac{v - c_y}{f_y} \cdot z_c, \quad z_c = z_c$$
Where $c_x = c_y = 64.0\text{ px}$ and $f_x = f_y = \frac{W / 2}{\tan(\text{FOV}/2)} = \frac{64}{\tan(30^\circ)} \approx 110.851\text{ px}$.

The points are transformed into the world frame via the camera's pose $(\mathbf{R}_{WC}, \mathbf{t}_{WC})$:
$$\mathbf{P}_{\text{world}} = \mathbf{R}_{WC} \mathbf{P}_{\text{cam}} + \mathbf{t}_{WC}$$

### 4.2 IMU-Stabilized Gravity-Aligned Frame
To eliminate the quadruped's walking roll $\phi$ and pitch $\theta$, the obstacle points are projected into an IMU-stabilized base frame:
$$\mathbf{R}_{\text{stab}} = \mathbf{R}_z(\psi) \quad \text{where } \psi = \text{yaw}(\mathbf{R}_{\text{base}})$$
Scans are converted to 2D local Cartesian coordinates $(x_{2D}, y_{2D})$ in this gravity-aligned frame, completely isolating the SLAM scan matcher from body pitching.

### 4.3 SVD-Based Closed-Form ICP Matching
Given source scan points $\mathcal{P} = \{\mathbf{p}_i\}$ and target scan points $\mathcal{Q} = \{\mathbf{q}_i\}$, correspondences are established via nearest neighbors gated by Euclidean distance threshold $d_{\text{gate}} = 0.35\text{m}$.
Centroids $\bar{\mathbf{p}}$ and $\bar{\mathbf{q}}$ are subtracted:
$$\mathbf{p}'_i = \mathbf{p}_i - \bar{\mathbf{p}}, \quad \mathbf{q}'_i = \mathbf{q}_i - \bar{\mathbf{q}}$$
The cross-covariance matrix $\mathbf{H}$ is decomposed using Singular Value Decomposition:
$$\mathbf{H} = \sum_{i=1}^M \mathbf{p}'_i (\mathbf{q}'_i)^T = \mathbf{U} \mathbf{\Sigma} \mathbf{V}^T$$
The optimal rotation $\mathbf{R}$ and translation $\mathbf{t}$ are resolved in closed form:
$$\mathbf{R} = \mathbf{V} \mathbf{U}^T, \quad \text{if } \det(\mathbf{R}) < 0 \implies \mathbf{V}_{[:, -1]} \leftarrow -\mathbf{V}_{[:, -1]}$$
$$\mathbf{t} = \bar{\mathbf{q}} - \mathbf{R} \bar{\mathbf{p}}$$

### 4.4 Recursive Bayesian Log-Odds Occupancy Update
The 2D map is represented as a grid of independent binary random variables $m_i \in \{0, 1\}$. Updating with measurement $z_t$ follows the log-odds formulation:
$$L(m_i \mid z_{1:t}) = L(m_i \mid z_{1:t-1}) + \log \left( \frac{P(m_i \mid z_t)}{1 - P(m_i \mid z_t)} \right) - \log \left( \frac{P(m_i)}{1 - P(m_i)} \right)$$
Using inverse sensor model increments:
$$l(m_i) \leftarrow \begin{cases} 
l(m_i) + l_{\text{occ}} (+0.85) & \text{if cell contains an obstacle endpoint} \\
l(m_i) + l_{\text{free}} (-0.40) & \text{if traversed by Bresenham ray} 
\end{cases}$$
Log-odds are numerically clamped to $L \in [-2.0, +2.0]$ to allow rapid adaptation when dynamic obstacles move.

### 4.5 Novelty-Weighted Frontier Utility Policy
Frontier cells (free cells adjacent to unknown territory) are detected via morphological dilation of free space intersecting unknown space, then clustered using 8-connected components. The candidate cluster centroid $C_k$ is selected according to utility:
$$\mathcal{U}(C_k) = \text{Area}(C_k) - \alpha \cdot \|\mathbf{p}_{\text{robot}} - C_k\| + \beta \cdot \text{Novelty}(C_k) - \gamma \cdot \text{BlacklistPenalty}(C_k)$$
- $\text{Area}(C_k)$: Size of frontier cluster (expected information gain).
- $\|\mathbf{p}_{\text{robot}} - C_k\|$: Euclidean transit cost.
- $\text{Novelty}(C_k)$: Inverse density of previously visited historical robot poses.
- $\text{BlacklistPenalty}(C_k)$: Heavy cost applied if previous A* paths to this region aborted.

### 4.6 Biomimetic Quadruped Locomotion
Gait generation executes synchronized sinusoidal joint trajectories at frequency $f = 5.0\text{ Hz}$:
$$\theta_{\text{hip}}^{\text{left}}(t) = A_{\text{hip}} \sin(2\pi f t), \quad \theta_{\text{hip}}^{\text{right}}(t) = A_{\text{hip}} \sin(2\pi f t + \pi)$$
$$\theta_{\text{leg}}^{\text{front}}(t) = A_{\text{leg}} \cos(2\pi f t), \quad \theta_{\text{leg}}^{\text{back}}(t) = A_{\text{leg}} \cos(2\pi f t + \pi)$$
Steering adds differential offsets to the hip joint targets based on heading error $\Delta \psi = \text{atan2}(y_{\text{target}} - y, x_{\text{target}} - x) - \psi$:
$$\delta_{\text{steer}} = \text{clamp}(K_p \cdot \Delta \psi, -\delta_{\max}, +\delta_{\max})$$

---

## 5. Quantitative Benchmark & Verification Results

The numerical evaluation engine (`accuracy_evaluator.py`) evaluated the system against ground truth matrices recorded during full autonomous exploration.

### 5.1 Comprehensive Multi-Stage Comparison

| Benchmark Metric / Feature | Stage 1 (Static A*) | Stage 2 (D* Lite Legacy) | Stage 3 (Active Version) |
| :--- | :--- | :--- | :--- |
| **Exploration Autonomy** | None (Fixed Goal Only) | Frontier Exploration | **Fully Autonomous Multi-Room** |
| **Mapping Engine** | Static Hardcoded Walls | Live Occupancy Grid | **Dual Bayesian Log-Odds Grid** |
| **SLAM Scan Matching** | None (Ideal Pose) | None (Raw Odometry) | **IMU-Stabilized ICP Pose Estimator** |
| **Path Planner** | Global A* (Single run) | D* Lite (Incremental) | **Vector-Dilated A* with Cooldown** |
| **Planning Latency** | $\sim 45\text{ ms}$ | $> 1500\text{ ms}$ (Severe bottleneck) | **$\sim 3.5\text{ ms}$ ($400\times$ speedup)** |
| **ZeroMQ Simulation Stability** | Stable | Critical Timeouts & Freezes | **100% Stable (Zero Dropouts)** |
| **Self-Body Collision Filter** | N/A | Failed (Legs mapped as walls) | **$40\text{cm}$ Radial Cylindrical Mask** |
| **Floor & Ceiling Noise Gating** | None | Basic Threshold | **3D Band Filter ($0.04\text{m} - 0.60\text{m}$)** |
| **Wall Blacklist Retention** | N/A | Flushed Every 4 Steps | **Persistent Hysteresis Clustering** |
| **Live Telemetry & Diagnostics** | Console only | 3 Separate Windows | **800x800 Unified 4-Panel HUD** |
| **Tolerant Obstacle IoU** | $100\%$ (Hardcoded) | $\sim 14.2\%$ (Wall Sticking) | **$90.99\%$ (Matrix-to-Matrix)** |
| **Mean Chamfer Wall Error** | $0.0\text{ mm}$ (Synthetic) | $> 450\text{ mm}$ (Drifted) | **$4.97\text{ mm}$ (Sub-centimeter)** |
| **95% Hausdorff Distance** | $0.0\text{ mm}$ | $> 1200\text{ mm}$ | **$50.0\text{ mm}$ ($1$ Grid Cell)** |

### 5.2 Perception & Depth Fidelity
- **Sensor Resolution:** $128 \times 128$ ($16,384$ depth pixels per frame)
- **Valid Depth Pixels:** $11,273$ pixels ($68.80\%$ valid return ratio)
- **Minimum Detected Range:** $0.122\text{ m}$ (Near-field obstacle detection)
- **Maximum Detected Range:** $3.334\text{ m}$ (Far-wall boundary returns)
- **Mean Detected Depth:** $0.749\text{ m} \pm 0.864\text{ m}$
- **Close Obstacle Ratio ($<1.0\text{m}$):** $57.06\%$ (Accurate corridor proximity tracking)

---

## 6. Key Engineering Challenges Solved (Deep Dive)

1. **Legged Locomotion Wobble & Pitch Inversion:**
   - *Problem:* As the quadruped walks, the torso rhythmically pitches up and down by up to $\pm 18^\circ$. This caused the fixed Kinect camera to cast rays into the floor on down-steps and into the ceiling on up-steps.
   - *Solution:* Project all 3D depth points into a gravity-stabilized horizontal plane using onboard IMU pitch/roll data before executing 2D raycasting or ICP matching.

2. **Self-Chassis & Foot Occlusion:**
   - *Problem:* The $60^\circ$ vertical FOV of the Kinect camera caught the quadruped's front legs at the bottom of the frame, marking the space immediately in front of the robot as solid obstacle.
   - *Solution:* Implemented a strict 2D radial deadzone: any point satisfying $\sqrt{x^2 + y^2} \le 0.40\text{m}$ in the robot-centric frame is discarded before occupancy grid insertion.

3. **High-Frequency Planning Bottlenecks:**
   - *Problem:* Traditional graph searches (or incremental D* Lite) on a $221 \times 221$ grid took $>1.5\text{s}$ when clearing obstacles.
   - *Solution:* Implemented morphological binary dilation (`scipy.ndimage.binary_dilation` with disk kernel $r=5$) pre-computing an inflated obstacle mask. $A^*$ traverses only valid free cells with a $1.0\text{s}$ cooldown between replans, reducing path calculation to $3.5\text{ms}$.

4. **Image vs Matrix Accuracy Fallacy:**
   - *Problem:* The legacy evaluation notebook (`map_accuracy.ipynb`) evaluated SLAM by comparing an exported JPEG (`true_map.jpeg`) against a screen capture (`slam_map.png`). Interpolation, RGB antialiasing, and aspect ratio stretching produced false metric error.
   - *Solution:* Built `accuracy_evaluator.py`, operating directly on the underlying NumPy float matrices (`true_grid.npy` vs `slam_grid.npy`), providing mathematically verifiable IoU, F1, Chamfer, and Hausdorff distances.

---

## 7. CoppeliaSim Simulation Environment & Hardware Interfaces

### Scene Hierarchy (`robot_working.ttt`)
```
/Base                               (Root robot chassis, z=0.048m)
├── /FrontJoint                     (Revolute hip motor, yaw axis)
│   └── /FrontLink
│       ├── /FrontLeftJoint         (Leg 1 revolute pitch motor)
│       └── /FrontRightJoint        (Leg 2 revolute pitch motor)
├── /BackJoint                      (Revolute hip motor, yaw axis)
│   └── /BackLink
│       ├── /BackLeftJoint          (Leg 3 revolute pitch motor)
│       └── /BackRightJoint         (Leg 4 revolute pitch motor)
└── /Kinect_camera_support          (Rigid camera mount)
    └── /kinect
        ├── /kinect_depth           (Vision sensor, 128x128 float depth buffer)
        └── /kinect_rgb             (Vision sensor, 128x128 uint8 RGB buffer)
```

### Coordinate Frames & Conventions
- **World Frame ($W$):** Fixed simulator reference frame, $+Z$ pointing upwards against gravity.
- **Base Frame ($B$):** Located at `/Base`, $+X$ pointing forward (robot heading), $+Y$ pointing to robot's left, $+Z$ normal to chassis.
- **Camera Frame ($C$):** Located at `/kinect_depth`, tilted downward by $15^\circ$ pitch angle.
- **Grid Coordinates ($G$):** $221 \times 221$ array, cell size $0.05\text{m}$. Offset: $\text{col} = \lfloor (x - x_{\min}) / \text{res} \rfloor$, $\text{row} = \lfloor (y - y_{\min}) / \text{res} \rfloor$.

---

## 8. Operational Runbook & How to Execute

### 1. Prerequisites
- **Python:** 3.8+ (tested on Python 3.10 / 3.12)
- **CoppeliaSim:** CoppeliaSim EDU 4.5+ or Player
- **Required Libraries:**
  ```powershell
  pip install numpy opencv-python scipy matplotlib coppeliasim-zmqremoteapi-client
  ```

### 2. Scene Diagnostic Check
Before launching autonomous missions, verify that joint handles and sensors are properly bound:
```powershell
python diagnose_sim.py
```
*Pass Criteria:* Scene objects detected, Kinect parented under `/Base`, joint torque flags active, depth buffer valid.

### 3. Running Autonomous Active SLAM & Exploration
1. Open CoppeliaSim and load `robot_working.ttt`.
2. Start the simulation in CoppeliaSim (press the Play button).
3. Run the primary mission orchestrator:
   ```powershell
   python main.py
   ```
4. The $800 \times 800$ OpenCV HUD will appear displaying RGB, JET Depth Heatmap, Occupancy Grid with active A* path, and live telemetry.
5. Press `q` in the OpenCV window to cleanly terminate and save matrices.

### 4. Running Benchmark Evaluation Suite
To execute evaluation on recorded matrices:
```powershell
python accuracy_evaluator.py
```
Outputs are generated in `accuracy_results/`:
- `evaluation_summary_report.txt`
- `accuracy_comparison.png`
- `numerical_map_accuracy_comparison.png`
- `sensor_matrix_heatmap_analysis.png`

### 5. Interactive Jupyter Analysis
Launch the interactive notebook for detailed step-by-step visual exploration:
```powershell
jupyter notebook map_accuracy.ipynb
```

---

## 9. Academic Presentation & Review Guide (Slide Summary)

For project reviews and defense panels (e.g., Panel 1, Panel 2), the core project narrative is structured into 10 concise slides:

1. **Title & Mission:** Autonomous Quadruped SLAM, Perception & Frontier Exploration in unknown multi-room indoor GPS-denied environments.
2. **The Legged SLAM Challenge:** Roll/pitch gait wobble, self-body occlusion, and planning latency.
3. **End-to-End Architecture:** Step-locked ZeroMQ remote API bridging CoppeliaSim to modular Python perception, SLAM, mapping, and planning engines.
4. **Perception Pipeline:** Pinhole back-projection, IMU gravity alignment, $40\text{cm}$ cylindrical self-mask, and $[0.04\text{m}, 0.60\text{m}]$ height band filtering.
5. **IMU-Stabilized ICP SLAM:** 2D SVD closed-form rigid transform solver with distance gating and odometry fallback.
6. **Bayesian Occupancy Grid:** Log-odds updates (Free: -0.4, Occ: +0.85, Clamp: $[-2, +2]$) via 2D Bresenham raycasting.
7. **Frontier Exploration Policy:** Morphological dilation, 8-connected clustering, and novelty-weighted utility scoring.
8. **Vector-Dilated $A^*$ Planning:** $3.5\text{ms}$ latency via precomputed morphological inflation + line-of-sight shortcut smoothing.
9. **Empirical Benchmarks:** $92.56\%$ grid accuracy, $90.99\%$ tolerant IoU, $4.97\text{mm}$ Chamfer error, and $400\times$ planning speedup over Stage 2.
10. **Live Demonstration & Future Roadmap:** Real-time 4-panel HUD, 3D trajectory visualization in CoppeliaSim, and transition to physical quadruped hardware (Unitree / ANYmal).

---

## 10. Future Roadmap & Hardware Transfer

- **Phase 4: Loop Closure & Pose Graph Optimization (PGO):** Integrate scan-to-map multi-hypothesis loop closure to eliminate cumulative trajectory drift across large multi-loop environments.
- **Phase 5: Semantic Perception & Object of Interest (ROI) Tagging:** Deploy lightweight YOLOv8 / MobileNet on RGB streams to identify and tag targets (e.g., victims, doors, hazards) on the occupancy map.
- **Phase 6: Deployment to Physical Quadruped Hardware:**
  - Export ROS2 / Micro-ROS nodes from `robot.py`, `slam.py`, and `planner.py`.
  - Target embedded SBC: NVIDIA Jetson Orin Nano / Xavier.
  - Target quadruped platforms: Unitree Go2 / A1 or custom 12-DOF quadruped chassis.
