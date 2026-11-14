# The project from A to Z: a guide for the panel review

Plain-language explanation of everything in this project: what it does, how each part works, why each decision
was made, where it stands against other systems, what does not work yet, and the questions a panel is likely to ask.
Numbers are from the repository as of 2026-11-15.

---

## 1. The project in one minute

Police, forensic and rescue teams often have to enter a building without knowing what is inside. We are building
the *navigation brain* of a four-legged robot that goes in first. It explores an unknown place by itself, works
out where it is at every moment, draws an accurate map, avoids hitting anything, and comes back with the map.

- The robot sees with **one depth camera** (like a Kinect), feels its own turning with a **gyro**, and counts its
  steps (**leg odometry**). No laser scanner (lidar), no GPS.
- The software is split into **seven agents**, each with one job, talking over a message bus.
- We test it in a robot simulator (**CoppeliaSim**) in two very different places: a 10 x 10 m building with rooms,
  and a 16 x 16 m rescue field full of rubble. Same code in both.
- Result (measured, same code in both scenes): 14.8 cm position error in the building and 28.8 cm over a 40-minute
  run in the rubble field (21.8 cm average), 90 % of the floor covered, map F1 78 %, **no collisions**, and the robot
  passes **through a damaged building** in the rubble field. On the same data it is about 10x more accurate than
  step-counting alone, BreezySLAM and Google Cartographer; it explores 11 points more floor than classic
  nearest-frontier exploration with 4x fewer stuck events. On a public real-robot dataset (TUM Pioneer) it averages
  25.9 cm, slightly better than the published RTABMap (28.7 cm).

---

## 2. The problem and the bigger plan

**Problem.** Before people enter a crime scene, hostage site or collapsed building, they have no current map and
cannot see inside without risk. Sending a person risks lives and disturbs evidence.

**The full plan (Phase I report, 7 stages):** Sense -> Navigate -> Trigger (react to an event) -> Perceive (spot
evidence with the team's own Mamba-based vision model) -> Decide -> Store (tamper-proof logs, SHA-256) ->
Sync (encrypted upload, AES-256).

**What is built now:** the first two stages, *Sense* and *Navigate*: exploration, localisation (knowing where it
is), mapping and safe movement. These are the foundation: evidence detection is useless if the robot cannot reach
places safely and say where it found things.

**Not built yet:** the event trigger, evidence perception, forensic storage and sync. Say this clearly if asked.

---

## 3. The robot and the simulator

| Item | What it is | Why |
|---|---|---|
| CoppeliaSim | A professional robot simulator; our Python code talks to it over its ZMQ remote API | Safe, repeatable, and it knows the *true* position of the robot, so we can measure our error exactly |
| Quadruped | Four legs, 12 joints, trot gait (diagonal legs move together), ~0.09 m/s | Legs handle rubble and steps better than wheels |
| Depth camera | 60 degree field of view, reliable to about 6.6 m | Cheap, light, also gives images for the later evidence stage |
| Gyro | Measures how fast the robot turns | Heading (which way the robot faces) |
| Leg odometry | Distance estimated from the gait (how far each step moves the body) | Forward/sideways motion |
| Contact check | The simulator reports if the robot touches any object | Honest collision count |

Two scenes:
- **Scene 1 (building):** 10 x 10 m, walls at right angles, rooms, doorways, a table.
- **Scene 2 (rescue field):** 16 x 16 m open floor, about 60 rubble blocks, a damaged building, a beam, barriers,
  and **no wall at the edge** - the floor ends in a drop.

**Integrity point (important).** In October we found that an early version secretly used the simulator's true
position inside SLAM ("ground-truth leak"). It was removed on 2026-10-11 and all earlier numbers were declared
void. Now the true position is used **only for scoring**, never by the robot.

---

## 4. Why we call them "agents"

An **agent** here is an independent software module that:
1. has **one responsibility** (e.g. "build the map"),
2. runs its own **loop at its own rate** (e.g. 10 or 20 times per second),
3. **does not call other agents directly**; it only publishes messages to and reads messages from a shared
   **message bus** (publish-subscribe),
4. keeps its own **state** and makes its own decisions from what it receives.

So the robot is a *team* of specialists rather than one big program. Benefits: each agent can be tested,
replaced or improved alone (we swapped the SLAM matcher many times without touching exploration); a crash or slow
step in one does not freeze the others' design; it maps naturally onto ROS nodes for a real robot later.

**They are not AI language-model agents.** No ChatGPT/LLM is involved. "Agent" is the classic robotics and
multi-agent-systems meaning: an autonomous component that senses (messages in), decides, and acts (messages out).
Expect this question.

---

## 5. The seven agents, one by one

The pipeline, top to bottom (see the architecture figure in the report):

| # | Agent | Input | Output (bus topic) | What it does, simply |
|---|---|---|---|---|
| 0 | **Supervisor** | events from everyone | mission state | The "manager": mission state, knowledge graph of rooms, timeline of events, decides when the mission is done |
| 1 | **Perception** | depth frame | `sensor/scan` | Turns the 3D depth picture into a 2D "slice": the distance to obstacles in a band just above the floor (like a laser scan). Also detects floor edges (drops) |
| 2 | **SLAM** | scan, odometry, gyro | `pose/slam` | Works out where the robot is (see section 6) |
| 3 | **Mapping** | scan + pose | `map/grid` | Paints the map: each 5 x 5 cm cell is free, occupied or unknown; finds rooms and doorways |
| 4 | **Frontier** | map | `frontier/target` | Decides where to go next: the next operator route point, else the best "frontier viewpoint" |
| 5 | **Planning** | target + map | `plan/path` | Finds a safe path with A* |
| 6 | **Locomotion** | path | joint commands; `control/actuation` (odometry, gyro) | Walks the path with the trot gait; watchdog for getting stuck; stops before drops |

Supporting modules: knowledge graph (rooms and how they connect), episodic memory (what happened when), spatial
memory, evaluation tools.

**One control step (20 times a second):** camera frame -> scan -> pose -> map update -> (re)decide target ->
(re)plan -> move legs -> odometry back to SLAM. Median time for the whole step: **25 ms** on a normal PC CPU.

---

## 6. Every concept, in simple words

### Localisation and SLAM
- **SLAM** (Simultaneous Localisation And Mapping): building a map of an unknown place while at the same time
  working out where you are in it. Chicken-and-egg: you need the map to know where you are and your position to
  draw the map. SLAM solves both together.
- **Pose**: position (x, y) plus heading (which way the robot faces, yaw).
- **Odometry / dead reckoning**: estimating motion by adding up your own movement (steps, wheel turns). Like walking
  with eyes closed and counting steps. Small errors **add up forever** - this is **drift**.
- **Gyro bias**: a gyro reads a tiny turning rate even when standing still. Over minutes this makes the heading
  slowly rotate. **Gyro calibration**: the robot stands still at the start (62 s in scene 2), measures that false
  rate, and subtracts it afterwards.
- **Scan**: the 2D slice of distances from the perception agent.
- **Scan matching / ICP** (Iterative Closest Point): slide and rotate the new scan until it lines up best with what
  was seen before. The shift needed = how the robot really moved. Corrects odometry drift.
- **Point-to-line ICP**: matches each scan point to the *nearest wall line* (not the nearest point). Works much
  better on walls; our ablation shows point-to-point fails completely.
- **Keyframe**: a saved scan + pose, stored every 0.5 m or 25 degrees of turning.
- **Submap**: the last N keyframes put together (8 in scene 1, 16 in scene 2). Matching against a submap instead of
  only the previous scan gives a narrow 60-degree camera more structure to lock onto. Biggest single gain: removing
  it raises error from 11.7 cm to 131.8 cm in scene 1.
- **Odometry agreement gate**: a scan match is accepted only if it roughly agrees with odometry (within 1 cm + 100 %
  of the step). Stops a bad match from teleporting the robot.
- **Degeneracy**: in a long corridor or open floor the scan cannot tell how far along you are (it is "degenerate"
  in that direction). The **degeneracy filter** trusts odometry in that direction instead.
- **Manhattan / heading prior** (scene 1 only): buildings mostly have walls at right angles; the robot gently pulls
  its heading so walls line up with 0/90 degrees. Great indoors, wrong in the rescue field and in the TUM hall, so it
  is switched off there.
- **Loop closure**: when the robot comes back to a place it has already seen, it recognises it, measures how far
  its estimate has drifted, and corrects the whole path. Like realising "I'm back at the door I started from, so my
  path drawing must be shifted by 20 cm".
- **Pose graph**: all keyframe poses are nodes; odometry and scan matches are springs between consecutive nodes; a
  loop closure is an extra spring between old and new nodes. Solving the graph spreads the correction along the path.
- **Robust pose graph (DCS, dynamic covariance scaling)**: a wrong loop closure would bend the whole map. DCS
  automatically weakens springs that disagree strongly with the rest, so a bad closure cannot tear the map.

### Mapping
- **Occupancy grid**: the map is a grid of 5 cm cells. Each holds a belief: free, occupied, or unknown.
- **Log-odds (Bayesian) update**: every observation nudges a cell's belief up or down; repeated observations make it
  confident; a single noisy reading does not.
- **Rooms and doorways**: found by image processing on the grid; used to explore room by room in buildings.

### Exploration
- **Frontier**: the border between known free space and unknown space. Going to frontiers is how a robot explores.
- **Nearest-frontier exploration (Yamauchi 1997)**: the classic method - always go to the closest frontier. Our
  planned baseline.
- **Viewpoint with standoff**: we do not drive *into* the frontier; we choose a spot 0.5 m back from unknown space
  that can *see* it. Essential in the rescue field, where unknown space may be a drop.
- **Operator route**: a person can draw a search route; the robot follows it waypoint by waypoint, skips points it
  cannot reach, then explores freely.
- **Completion**: exploration ends only when viewpoints exist and none of them sees any frontier.

### Planning and movement
- **A***: the standard shortest-path search on a grid.
- **Inflation / clearance**: obstacles are "grown" by the robot's size (4 cells = 0.20 m) so the path keeps a safe gap.
- **Narrow-gap mode**: if no path exists, try again with a tighter clearance (0.15 m) to pass gaps of about 0.35 m.
- **Watchdog**: if the robot makes no progress along its path for a while, it turns away and re-plans; a blocked
  target is skipped.
- **Drop check**: the camera looks for the floor ending ahead and stops the robot.

### How we measure
- **Ground truth**: the simulator's exact robot position and the scene's exact 3D geometry.
- **ATE** (Absolute Trajectory Error, RMSE): average distance between where the robot thought it was and where it
  really was, over the whole run. Our main accuracy number.
- **RPE** (Relative Pose Error): drift per metre walked.
- **Precision**: of the obstacles in our map, what share are really there (are we drawing fake walls?).
- **Recall**: of the real obstacles, what share did we map (did we miss things?).
- **F1**: one number combining precision and recall. Tolerance 15 cm.
- **Observable recall**: recall counting only obstacle faces that can actually be seen from the floor.
- **Replay**: every run records all sensor data; we can re-run SLAM on it offline in seconds. Lets us test changes
  on identical data.
- **Seeds / chaos**: in scene 2, tiny differences change the result by several cm, so we average 3-9 replays with
  small random perturbations ("seeds") before trusting a change.
- **Ablation**: remove one part, measure again, to prove each part is needed.
- **Robustness test**: add sensor faults (noise, missing points, gyro bias) and see how much worse it gets.

---

## 7. Current settings

| Setting | Scene 1 (building) | Scene 2 (rescue) | Why |
|---|---|---|---|
| Odometry scale | 1.0 | 0.701 | Gait moves less per step on that floor (calibrated) |
| Gyro calibration (stand still) | 0 s | 62 s | Long open stretches need an accurate heading |
| Heading prior (Manhattan) | on (0.1) | off | Rubble field has no right-angle walls |
| ICP heading weight | 0.1 | 0 | Same reason |
| Submap keyframes | 8 | 16 | Scene 2 replay mean 16.3 -> 13.9 cm |
| Narrow-gap mode | off | on | Rubble gaps |
| Room-by-room sweep | on | off | Open field has no rooms |
| Map window | 11 x 11 m | 16.5 x 16.5 m | Scene size |
| Operator route | none | route v3: 70 waypoints, building crossing first (in through the south, interior, east room, out of the north doorway), then the drawn route | Main aim: pass through the building; then cover the field |

Everything else is identical code. The per-scene file is `config/scenes/<scene>.json`.

---

## 8. Results (measured, current code, 2026-11-15)

| Metric | Scene 1 (30 min) | Scene 2 (40 min, through the building) | Average |
|---|---|---|---|
| Position error (ATE) | 14.8 cm | 28.8 cm | 21.8 cm |
| Map precision | 93.1 % | 67.7 % | 80.4 % |
| Map recall | 84.9 % | 66.4 % | 75.7 % |
| Map F1 | 88.8 % | 67.1 % | 78.0 % |
| Floor covered | 94.2 % | 85.8 % | 90.0 % |
| Collisions | 0 | 0 | 0 |
| Stuck events (all recovered) | 1 | 5 | 3 per run |

**Scene 2 highlight:** the robot entered the damaged building from the south at 218 s, visited its east room and left
through the 1 m north doorway at 594 s, with no collisions. 59 of the 70 route waypoints were reached.

**Earlier best runs (kept, older conditions):** scene 1 run 34: 11.7 cm, F1 95.9 % (older code and a start heading 2
degrees different - from today's start, run 34's own code gives 32.0 cm, so today's code is better); scene 2 run 19:
12.3 cm, but only 12 minutes and 59 % of the floor.

**Why scene 2 error is higher:** error stays near 11 cm for the first 17 minutes, then grows. In the rubble field the
heading comes from the gyro alone (no right-angle walls to correct it); about 2 degrees of drift builds up over 40
minutes. We tested scan-based heading correction on the recording: it made things worse (65-110 cm), so it stays off.

What the numbers mean: the robot knows where it is to about a hand's width in the building and two hands' width after
40 minutes in the rubble field; most of what it draws is really there; it misses the hidden sides of rubble blocks.

---

## 9. Benchmarks - where we stand and why

### 9.1 Inside our own system
- **Ablation** (scene 1): no loop closure 25.9 cm; scan-to-scan instead of submap 131.8 cm; point-to-point ICP
  291 cm; no heading prior 296 cm; full system 11.7 cm. Scene 2: no gyro calibration 178 cm. Every part earns its place.
- **Robustness** (scene 2): depth noise, 50 % missing points and 10 % leg-odometry noise barely matter (9-14 cm).
  A +/-5 % leg scale error is halved by SLAM (16 cm vs 31 cm for odometry). **Gyro bias left after calibration is
  the weak spot**: 0.01 deg/s -> 30 cm, because in scene 2 the heading comes from the gyro alone.

### 9.2 Other systems run on our data (same input, same scoring)
| Method | Scene 1 | Scene 2 | Average |
|---|---|---|---|
| Odometry only | 291 cm | 15.7 cm | 154 cm |
| BreezySLAM (best of 11 settings) | 190 cm | 110 cm | 150 cm |
| Cartographer (Google), best of 8 settings | 342 cm | 85 cm | 213 cm |
| **Ours** | **11.7 cm** | **12.3 cm** | **12.0 cm** |

**BreezySLAM** is an open-source version of CoreSLAM/tinySLAM, a classic lightweight 2D laser SLAM. It searches for
the best scan position by random small changes (hill climbing), has no loop closure, and is built for 360-degree
laser scanners. With our 60-degree camera slice it has too little to match against and drifts.

**Cartographer** is Google's open-source SLAM, an industry standard for 2D laser SLAM: it matches each scan to small
local maps (submaps) and closes loops in a pose graph, much like ours. We ran it in Docker (ROS 2) on exactly our scan
and odometry. It starts correctly (1 cm error at 90 s) and then drifts, because it is designed for dense 360-degree
laser scans and accepts wrong loop closures on our 58-degree camera slice. Best of 8 settings: 342 cm in scene 1,
85 cm in scene 2 - worse than odometry alone. This is the strongest argument for our design: standard laser SLAM
does not work with a single narrow depth camera; ours does.

### 9.3 Public benchmark: TUM RGB-D Pioneer
TUM (Technical University of Munich) RGB-D is a standard public dataset: a Kinect on a Pioneer wheeled robot in a
large hall, with exact motion-capture ground truth and a standard scoring tool. It is the closest public match to
our robot (ground robot, depth camera, odometry).

| System | Average ATE (4 sequences) |
|---|---|
| ORB-SLAM3 (published) | 172.0 cm |
| OpenVSLAM (published) | 39.2 cm |
| RTABMap (published) | 28.7 cm |
| Pioneer wheel odometry alone | 28.7 cm |
| **Ours, scene-2 settings** | **25.9 cm** |
| Ours, scene-1 settings | 40.5 cm |

**Why we do well:** we combine odometry with scan matching and never "lose track"; the visual systems sometimes fail
completely on a sequence (OpenVSLAM 122 cm on slam2, RTABMap 72 cm on slam3).
**Why it is not a clean win:** we use the robot's wheel odometry, they use images only; and on real Kinect data we
are only slightly better than wheel odometry alone (and worse on two sequences). Our scan matcher was tuned on clean
simulated depth; real depth is noisier.

### 9.4 Exploration comparison (nearest-frontier, Yamauchi 1997)
Same robot, SLAM, planner and time limit; only the rule for where to go next differs.

| Measure | Ours (scene 1 / 2) | Nearest-frontier (scene 1 / 2) |
|---|---|---|
| Floor covered | 94.2 / 85.8 % | 84.9 / 72.4 % |
| Map F1 | 88.8 / 67.1 % | 89.1 / 52.1 % |
| Stuck events | 1 / 5 | 18 / 7 |
| Position error | 14.8 / 28.8 cm | 17.5 / 36.2 cm |
| Through the building | - / yes | - / no |

Why ours wins: it picks viewpoints that see the most unknown space and keeps a safe standoff, so it wastes less
walking and gets stuck less; the operator route takes it through the building. Honest note: in the building (scene 1)
the final map quality is the same - the gain there is coverage and reliability (1 vs 18 stuck events).

---

## 10. What is failing or limited (be honest about these)

1. **Simulation only.** No real robot yet; real sensors are noisier (TUM shows this).
2. **Scan 2 recall ~60 %.** A 2D slice from one camera sees only the side of each rubble block facing the robot.
3. **Error grows on long open stretches** where there is nothing to match; position then depends on odometry and gyro.
4. **Gyro bias** not removed by calibration goes straight into the heading in scene 2.
5. **On real data (TUM) the scan matcher adds little over good wheel odometry.**
6. **Scene 2 run-to-run variation** of several cm; we use averaged replays to decide changes.
7. **Error in scene 2 grows after ~17 minutes** (11 cm -> 29 cm RMSE over 40 min) from about 2 degrees of gyro heading drift.
8. **Later stages** (evidence detection, trigger, secure logs) are not integrated.

---

## 11. Major decisions and why

| Decision | Why |
|---|---|
| Depth camera instead of lidar | Cost, weight, and the same camera serves the evidence stage later |
| 2D slice SLAM instead of full 3D | Runs in real time on a CPU; floors are flat in our scenes |
| Agents on a message bus | Modular, testable, replaceable; maps to ROS later |
| Keyframe submap matching | Narrow 60-degree view needs more structure than one previous scan |
| Robust pose graph (DCS) | One wrong loop closure must not ruin the map |
| Gyro stand-still calibration | Removes most heading drift in open spaces |
| Viewpoint standoff 0.5 m | Never walk into unseen space; the rescue floor ends in a drop |
| Operator route option | Responders often know where to search first |
| Scoring against the scene model | Independent ground truth, not the robot's own data |
| Record every run, replay to decide | Same-data comparisons; avoids chasing lucky runs |
| Average several seeds in scene 2 | Single runs vary by several cm |
| Same code, per-scene profile only | Shows the system generalises |
| Remove the ground-truth leak, void old numbers | Integrity |

---

## 12. Existing systems and their limits

| System | What it is | Limitation relevant to us |
|---|---|---|
| ORB-SLAM2/3 | Feature-based visual SLAM (camera images) | Loses track with few textures or fast turns; 172 cm on TUM Pioneer |
| OpenVSLAM | Visual SLAM, similar idea | Fails on some sequences (122 cm on pioneer_slam2) |
| RTAB-Map | RGB-D SLAM with strong loop closure | Heavier; 72 cm on pioneer_slam3 |
| Cartographer | Google 2D/3D laser SLAM | Built for 360-degree lidar; struggles with a narrow camera slice |
| GMapping / BreezySLAM | Classic 2D laser SLAM | Need wide laser scans; BreezySLAM has no loop closure |
| FAST-LIVO2 etc. | Lidar + camera + IMU SLAM | Very accurate but needs expensive lidar |
| Nearest-frontier (Yamauchi) | Classic exploration | Greedy, walks back and forth, no safety standoff |
| TARE, DSVP, FARE | Modern exploration planners | Designed for fast wheeled robots with 3D lidar |
| Boston Dynamics Spot (Autowalk) | Commercial legged robot | Autonomy replays a recorded route; does not explore unknown space |
| ANYmal | Commercial legged robot | Lidar-based, expensive |

---

## 13. Questions the panel may ask (with answers)

**Q: Why simulation and not a real robot?**
To develop and measure safely and exactly: the simulator gives the true position and true geometry, so every
number is checkable. The next phase moves to hardware; the TUM test already checks our SLAM on real sensor data.

**Q: Why are they called agents? Are you using AI agents / LLMs?**
No LLMs. Each agent is an autonomous module with one job, its own loop, and only message-based communication - the
classic multi-agent robotics meaning (section 4).

**Q: What is novel?**
A legged robot exploring unknown, cluttered places autonomously with only one depth camera, gyro and leg odometry,
the same code working in two very different environments, with safety rules for unseen drops, an operator route
option, and an honest evaluation against an independent scene model.

**Q: How do you know your numbers are not cherry-picked?**
Every run is recorded and committed; changes are judged on averaged replays; the scene model is independent of the
robot; we report our weak results too (TUM scene-1 settings 40.5 cm, recall 60 %).

**Q: Why not lidar?**
Cost and weight, and the camera is needed for evidence detection anyway. The price is a narrow 60-degree view,
which we compensate with submap matching and loop closure.

**Q: What happens if a loop closure is wrong?**
The robust pose graph (DCS) down-weights constraints that disagree with the rest, so a single wrong closure cannot
distort the map. Removing it made scene 1 error 27.3 cm instead of 11.7 cm.

**Q: How does the robot avoid falling off an edge?**
Viewpoints stay 0.5 m from unknown space, and the perception agent checks the floor ahead and stops before a drop.

**Q: Why is recall lower in the rescue field?**
A 2D slice from one camera only sees the faces of rubble blocks pointing at the robot; the back sides stay unseen
unless the robot walks around every block.

**Q: Why compare with BreezySLAM and not only big systems?**
It takes exactly the same kind of input (2D scan + odometry), so it is a fair same-data comparison; Cartographer
(the industry standard) is being added the same way.

**Q: Is the TUM comparison fair?**
Same sequences, same scoring. Two differences we state openly: we estimate 2D position (they 3D, nearly equal on a
flat floor), and we use wheel odometry (they use images only).

**Q: What fails today?**
Section 10. The main risks for a real robot are gyro bias and noisier depth.

**Q: What is the update rate and can it run on a small computer?**
20 Hz control loop, 25 ms median per step on a PC CPU with no GPU.

**Q: How did you choose the settings?**
On recorded runs with replay, averaging several seeds; each choice is in the decision log (PROGRESS.md) with the
measured gain.

---

## 14. Status and what comes next

Done: both scenes running, scene 1 best result, scene 2 runs, ablation, robustness, odometry and BreezySLAM
baselines, TUM benchmark, report draft with diagrams.

Done since: Cartographer on our recordings (Table above); scene 1 verification on current code (14.8 cm, 0 collisions).

Next: reduce heading drift on long open stretches (periodic stand-still gyro re-calibration); improve scan matching on
real depth data (TUM); then hardware and the evidence-detection stage.

Where the details live: `docs/report_phase2/Phase2_Report.pdf`, `docs/results/` (each benchmark), `docs/project/
PROGRESS.md` (decision log), `FIXES.md` (every fix).
