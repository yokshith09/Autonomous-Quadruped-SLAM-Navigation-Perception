# Autonomous Quadruped Multi-Agent SLAM and Exploration Stack

CPU-only 2D SLAM, room segmentation and room-by-room exploration for a legged robot in CoppeliaSim
(`robot_working.ttt`). Seven cooperating agents run in one Python process and talk over a message bus.

Current focus: **measure the system honestly** (`docs/OBJECTIVES.md`), then improve it.
Baseline results: [`runs/20261024_210638/REPORT.md`](runs/20261024_210638/REPORT.md), interpretation in
[`docs/BASELINE.md`](docs/BASELINE.md).

## Setup
```powershell
pip install -r requirements.txt
```
Open CoppeliaSim EDU 4.5+ and load `robot_working.ttt` (do not press play; the script starts the simulation).

## Commands
| Task | Command |
|---|---|
| Run the robot and measure it | `python main_agentic.py` (add `--headless` to skip the HUD window, `--max-time S` to change the safety cap, default 3600 simulated seconds) |
| Component ablation by offline replay of that run | `python ablate.py runs/<run_id>` |
| Rebuild the report (after the ablation) | `python report.py runs/<run_id>` |
| Replay SLAM variants on a recording | `python slam_replay.py runs/<run_id>/recording.npz --set icp_yaw_weight=0.2` |
| Tests | `python tests/test_agents.py`, `python tests/test_slam_accuracy.py`, `python tests/test_run_telemetry.py` |

Each run writes `runs/<run_id>/`: `REPORT.md` (the 7-section report), `metrics.json`, `agents.csv`, `trajectory.csv`
(ground truth and estimate per step), `manifest.json` (commit, hardware, configuration, metric definitions),
`recording.npz` (raw SLAM input, git-ignored), `artifacts/` (grids, knowledge graph) and `terminal.log`.
The terminal shows a progress line every 30 simulated seconds and one summary at the end.

## How it works
- **Agents** (`agents/`): Perception (depth to body-frame points), SLAM (point-to-line ICP, gyro-led yaw, Manhattan
  heading prior, keyframe loop closure), Mapping (Bayesian log-odds grid, room segmentation), Frontier (choice of the
  next target), Planning (A* on known free space), Locomotion (gait and steering, stuck watchdog), Supervisor
  (knowledge graph, event log).
- **Odometry** is dead-reckoned from the commanded gait plus a simulated noisy gyro. The estimator never reads the
  simulator pose; `audit.py` checks this on every report.
- **Map evaluation** keeps two grids: the SLAM grid (estimated pose) and a reference grid built from the same depth
  rays with the true pose. The difference isolates the effect of pose error. It is not an independent floor plan.
- **Known simulation shortcut:** `LizardRobot.steer_heading` turns the body by setting its orientation directly
  (actuation, not sensing).

## Repository layout
```text
main_agentic.py       entry point: wires the agents to CoppeliaSim, runs the mission, records the run
run_telemetry.py      per-run measurement, metrics, manifest, terminal summary
report.py             builds runs/<id>/REPORT.md from a run's files
ablate.py             offline component ablation + replay-consistency checks
audit.py              ground-truth independence checks
slam_replay.py        offline SLAM replay on a recording
accuracy_evaluator.py grid (accuracy/completion/F1) and trajectory (ATE/RPE) metrics
benchmarks.py         published third-party numbers with their sources (used only for the later comparison phase)
slam.py grid.py planner.py robot.py sensors.py viz.py   algorithm and simulator-interface code
agents/               the seven agents and the message bus
tests/                formula and algorithm tests
docs/                 OBJECTIVES.md, BASELINE.md, panel review documents, architecture diagram
```
`captured_matrices/` and `accuracy_results/` hold the latest run's raw grids and legacy output files (git-ignored).
