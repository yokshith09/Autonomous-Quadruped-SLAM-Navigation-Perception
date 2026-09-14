# 🚀 How to Run the Project (Quick Start Guide)

This guide contains **all the exact commands and steps** to run every script and tool in this repository.

---

## ⚡ 1. Prerequisites & Installation

Open **PowerShell** or **Command Prompt** and install the required dependencies:

```powershell
pip install numpy scipy opencv-python matplotlib jupyter coppeliasim-zmqremoteapi-client
```

> **Note:** Make sure **CoppeliaSim EDU v4.5+** is installed on your computer.

---

## 🎮 2. Quick Execution Order (Run in Sequence)

```
┌────────────────────────────────────────────────────────────────────────┐
│                          EXECUTION STEPS                               │
│                                                                        │
│   Step 1: Open CoppeliaSim & Load 'robot_working.ttt'                  │
│     │                                                                  │
│     ▼                                                                  │
│   Step 2: Run Diagnostics (python diagnose_sim.py)                     │
│     │                                                                  │
│     ▼                                                                  │
│   Step 3: Run Autonomous SLAM (python main.py)                         │
│     │                                                                  │
│     ▼                                                                  │
│   Step 4: Run Evaluation Benchmark (python accuracy_evaluator.py)      │
│     │                                                                  │
│     ▼                                                                  │
│   Step 5: View Interactive Notebook (jupyter notebook)                 │
└────────────────────────────────────────────────────────────────────────┘
```

---

## 🛠️ Step 1: Open the Simulation Scene in CoppeliaSim

1. Launch **CoppeliaSim EDU**.
2. Click **File ➔ Open scene...**
3. Select:
   ```text
   robot_working.ttt
   ```
4. Keep CoppeliaSim open in the background (you do **not** need to manually click Play; `main.py` will start and step the simulation automatically).

---

## 🔍 Step 2: Verify Scene Setup (Diagnostics)

Before running the full mission, verify that the camera rig, joint motors, and depth sensors are responding correctly:

```powershell
python diagnose_sim.py
```

### Expected Output:
```text
=== CHECK 1: is the Kinect rig parented under /Base? ===
  depth_cam is a descendant of /Base: True
=== CHECK 2: are the leg/hip joints motor-enabled? ===
  motor_enabled=1, ctrl_enabled=1 (All joints OK)
=== CHECK 3: does the base move under gait alone? ===
  Physics IS moving the robot.
=== CHECK 4: is the depth sensor returning usable obstacle points? ===
  World points in obstacle band [0.04m, 0.60m]: OK
```

---

## 🤖 Step 3: Run Autonomous SLAM & Exploration (`main.py`)

Run the master autonomous exploration mission:

```powershell
python main.py
```

### What Happens:
1. Connects to CoppeliaSim via ZeroMQ.
2. The robot automatically starts walking and exploring unknown rooms.
3. An **800×800 4-Panel Real-Time HUD** appears:
   - **Top-Left:** Live RGB Camera Feed
   - **Top-Right:** Calibrated Depth Heatmap (JET Colormap)
   - **Bottom-Left:** Live 2D Occupancy Grid (Yellow path, Green traveled trail, Cyan true pose, Magenta SLAM pose)
   - **Bottom-Right:** Live Telemetry (Position drift, F1 accuracy, ICP status, % explored)

### Keyboard Controls:
- Press **`q`** in the OpenCV window to stop exploration, save all matrices, and automatically trigger the evaluation benchmark.

---

## 📊 Step 4: Run Quantitative Accuracy Evaluation (`accuracy_evaluator.py`)

To evaluate or re-evaluate the map and trajectory accuracy from saved matrices:

```powershell
python accuracy_evaluator.py
```

### Output Files Generated in `accuracy_results/`:
| File Name | Description |
| :--- | :--- |
| **`evaluation_summary_report.txt`** | Full numerical report (F1 score, IoU, Chamfer error, ATE) |
| **`numerical_map_accuracy_comparison.png`** | 4-panel visual comparison: Ground truth vs SLAM vs difference |
| **`robot_movement_trajectory_analysis.png`** | Trajectory overlay, drift curves, and heading error vs time |
| **`sensor_matrix_heatmap_analysis.png`** | Depth distribution histogram and RGB/depth sensor views |

---

## 📓 Step 5: Run Interactive Jupyter Notebook (`map_accuracy.ipynb`)

To inspect maps, matrices, and plots interactively:

```powershell
jupyter notebook map_accuracy.ipynb
```

Click **Kernel ➔ Restart & Run All** to run all evaluation cells.

---

## 🏃 Alternative: Run Dedicated Evaluation Script (`evaluate_slam.py`)

If you want to run a dedicated benchmark session with automated trajectory and matrix logging:

```powershell
python evaluate_slam.py
```

---

## 📁 Where Are Output Files Saved?

- **`captured_matrices/`**
  - `true_grid.npy`: Ground-truth occupancy log-odds matrix
  - `slam_grid.npy`: SLAM-generated occupancy log-odds matrix
  - `trajectory_data.npz`: Poses $(x, y, \psi)$ and timestamps
  - `matrix_data.npz`: Latest raw depth & RGB matrices
  - `depth_matrix.txt`: Raw metric distance numbers in plain text

- **`accuracy_results/`**
  - All generated benchmark charts, difference overlays, and evaluation text reports.

---

## ❓ Quick Troubleshooting

| Issue | Cause | Fix |
| :--- | :--- | :--- |
| **Connection Refused / ZMQ Timeout** | CoppeliaSim is not running | Open CoppeliaSim and load `robot_working.ttt` first. |
| **`Depth camera NOT found`** | Camera path mismatch | Run `python diagnose_sim.py` to inspect the tree. |
| **Robot steps but does not translate** | Scene scripts blocking physics | `main.py` automatically patches this. Make sure you run `python main.py` rather than clicking Play in CoppeliaSim without the script. |
| **OpenCV window doesn't respond** | Window needs focus | Click inside the `NavPulse Debug Dashboard` window, then press **`q`** to exit cleanly. |
