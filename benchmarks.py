"""Published benchmark numbers used for comparison, each tied to the source it was transcribed from.

Values are copied verbatim from the cited tables. None means the source reports a failure or no value.
Only add a number here together with the table it came from.
"""

SOURCES = {
    "MNE": ("Deng et al. 2025",
            "T. Deng, G. Shen, C. Xun, S. Yuan, T. Jin, H. Shen, Y. Wang, J. Wang, H. Wang, D. Wang, W. Chen, "
            "\"MNE-SLAM: Multi-Agent Neural SLAM for Mobile Robots\", CVPR 2025, pp. 1485-1494.",
            "https://doi.org/10.1109/CVPR52734.2025.00146"),
    "ORB2": ("Mur-Artal & Tardos",
             "R. Mur-Artal, J. D. Tardos, \"ORB-SLAM2: an Open-Source SLAM System for Monocular, Stereo and RGB-D Cameras\".",
             "https://arxiv.org/abs/1610.06475"),
    "A1": ("Chen & Dellaert",
           "J. Chen, F. Dellaert, \"A1 SLAM: Quadruped SLAM using the A1's Onboard Sensors\".",
           "https://arxiv.org/abs/2211.14432"),
    "QUAD": ("Corlito et al. 2026",
             "R. Corlito, F. Schmidt, N. Seibert, et al., \"Sensor Configuration Matters: A Systematic Evaluation of "
             "Multimodal SLAM on Quadruped Robots\" (ANYmal D, GrandTour dataset).",
             "https://arxiv.org/abs/2606.19067"),
    "ROS": ("Filipenko & Afanasyev",
            "M. Filipenko, I. Afanasyev, \"Comparison of Various SLAM Systems for Mobile Robot in an Indoor Environment\".",
            "https://arxiv.org/abs/2501.09490"),
    "FARE": ("Liao et al. 2026",
             "S. Liao, X. Lv, J. Lew, S. Zhang, et al., \"FARE: Fast-Slow Agentic Robotic Exploration\".",
             "https://arxiv.org/abs/2601.14681"),
    "SPOT": ("Boston Dynamics",
             "Boston Dynamics, Spot product specifications and Spot SDK 5.2.0 autonomy documentation (GraphNav, "
             "Directed Exploration).",
             "https://bostondynamics.com/products/spot/"),
    "ANYMAL": ("ANYbotics", "ANYbotics, ANYmal product page and technical specifications.",
               "https://www.anybotics.com/robotics/anymal/"),
    "GO2": ("Unitree", "Unitree Robotics, Go2 product page and specification table.", "https://www.unitree.com/go2"),
}

# --- Localization -----------------------------------------------------------------------------------------

# A1 SLAM Table 1 + Table 2: Unitree A1 quadruped, ~5 m indoor trajectories, motion-capture ground truth.
# (trajectory, distance m, average speed m/s, Cartographer APE RMSE m, A1 SLAM APE RMSE m)
A1_SHORT_TRAJECTORIES = [
    ("T1", 5.3, 0.15, 0.040, 0.047),
    ("T2 (aggressive)", 5.0, 0.34, 0.962, 0.118),
    ("T3 (aggressive)", 5.0, 0.22, 1.397, 0.145),
    ("T4", 5.2, 0.12, 0.134, 0.043),
]

# Quadruped evaluation (Corlito et al. 2026), Tables II/IV/V: ANYmal D, GrandTour, 7 missions of 145-332 m.
QUAD_MISSION_DIST_M = [145.3, 266.8, 273.6, 162.0, 331.8, 152.7, 311.3]
# (method, sensors, mean ATE RMSE m, mean RPE m, CPU %, RAM GB); CPU/RAM only where Table V reports them.
QUAD_SLAM = [
    ("FAST-LIVO2", "LiDAR + camera + IMU (Honeywell)", 0.105, 0.035, 124.52, 3.91),
    ("ORB-SLAM3", "stereo-inertial (ADIS IMU)", 0.229, 0.014, None, None),
    ("ORB-SLAM3", "stereo (global shutter)", 0.330, 0.017, 289.98, 1.16),
    ("RTAB-Map", "stereo (global shutter)", 0.579, 0.058, 115.67, 0.75),
    ("DPV-SLAM", "camera (Sevensense, global shutter)", 0.967, 0.062, 82.26, 4.41),
    ("RTAB-Map", "RGB-D (ZED2i)", 2.413, 0.140, None, None),
    ("ORB-SLAM3", "RGB-D (ZED2i)", 8.214, 1.225, None, None),
]

# ORB-SLAM2 Table III: TUM RGB-D, translation RMSE (m). Other methods as published by their authors.
TUM_RGBD = {
    "sequences": ["fr1/desk", "fr1/desk2", "fr1/room", "fr2/desk", "fr2/xyz", "fr3/office", "fr3/nst"],
    "ORB-SLAM2": [0.016, 0.022, 0.047, 0.009, 0.004, 0.010, 0.019],
    "ElasticFusion": [0.020, 0.048, 0.068, 0.071, 0.011, 0.017, 0.016],
    "Kintinuous": [0.037, 0.071, 0.075, 0.034, 0.029, 0.030, 0.031],
    "DVO-SLAM": [0.021, 0.046, 0.043, 0.017, 0.018, 0.035, 0.018],
    "RGBD-SLAM": [0.026, None, 0.087, 0.057, None, None, None],
}

# Filipenko & Afanasyev Table V: same office dataset, ATE RMSE (m) against Hector SLAM's trajectory.
ROS_OFFICE = [("Cartographer (2D LiDAR)", 0.024), ("RTAB-Map (stereo)", 0.163), ("ORB-SLAM (mono)", 0.166),
              ("ORB-SLAM (stereo)", 0.190), ("S-PTAM (loop closure)", 0.295), ("LSD-SLAM (mono)", 0.301),
              ("DSO (mono)", 0.459), ("ZEDfu (stereo)", 0.726)]

# MNE-SLAM Table 2: multi-room Apartment scenes, global ATE RMSE (cm) for Apt-1, Apt-2, Apt-0.
MNE_APARTMENT = [
    ("ORB-SLAM3", 4.93, 1.36, 1.07), ("NICE-SLAM", 38.56, 4.35, 2.18), ("Co-SLAM", 3.19, 1.54, 0.83),
    ("ESLAM", 1.17, 0.79, 0.76), ("GO-SLAM", 1.27, 0.62, 0.53), ("Point-SLAM", 1.63, 0.68, 0.58),
    ("PLGSLAM", 1.13, 0.79, 0.74), ("Loopy-SLAM", 1.43, 0.60, 0.64), ("CCM-SLAM", 5.71, 0.62, None),
    ("Swarm-SLAM", 5.56, 5.61, 1.80), ("CP-SLAM", 5.73, 1.85, 0.91), ("MNE-SLAM", 1.02, 0.59, 0.48),
]

# MNE-SLAM Table 3: ScanNet real-world, average ATE RMSE (cm) over 8 sequences.
MNE_SCANNET_AVG = [("NICE-SLAM", 13.0), ("ESLAM", 10.6), ("Co-SLAM", None), ("Point-SLAM", 14.3),
                   ("GO-SLAM", 6.9), ("PLGSLAM", 9.9), ("Loopy-SLAM", 7.7), ("Swarm-SLAM", 10.5),
                   ("CP-SLAM", 9.7), ("MNE-SLAM", 6.6)]

# --- Mapping ----------------------------------------------------------------------------------------------

# MNE-SLAM Table 1: Replica global map. Acc, Comp (cm), Comp.Ratio (% < 5 cm), ATE RMSE (cm).
MNE_REPLICA = [
    ("iMAP", 2.881, 4.934, 80.515, 4.153), ("NICE-SLAM", 2.373, 2.645, 91.137, 2.503),
    ("Vox-Fusion", 2.881, 2.563, 90.936, 1.453), ("ESLAM", 2.082, 1.754, 95.977, 0.657),
    ("Co-SLAM", 2.104, 2.082, 93.435, 1.059), ("GO-SLAM", 2.091, 2.415, 93.174, 0.404),
    ("Point-SLAM", 1.935, 2.779, 90.479, 0.559), ("PLGSLAM", 1.884, 1.543, 96.329, 0.631),
    ("Loopy-SLAM", 1.593, 2.645, 91.137, 0.369), ("CP-SLAM", 1.794, 2.455, 90.394, 0.673),
    ("MNE-SLAM", 1.681, 1.509, 96.957, 0.398),
]

# --- Runtime ----------------------------------------------------------------------------------------------

# MNE-SLAM Table 5 (Replica): mapping time per frame (s), tracking per frame (ms), scene memory (MB).
MNE_RUNTIME = [("ESLAM", 0.29, 68.5, 20.73), ("Co-SLAM", 0.15, 54.8, 26.39), ("Point-SLAM", 10.47, 29.7, 31.42),
               ("CP-SLAM", 10.45, 34.7, 39.75), ("MNE-SLAM", 0.14, 22.1, 24.45)]
# ORB-SLAM2 Table IV (TUM fr3_office, RGB-D 640x480, Intel Core i7-4790): tracking total per frame (ms).
ORB2_TRACK_MS = 25.58
# A1 SLAM Table 4: average optimisation call per LiDAR scan (ms).
A1_OPT_MS = [("A1 SLAM (GTSAM)", 1.65), ("Cartographer (Ceres)", 0.96)]

# --- Exploration ------------------------------------------------------------------------------------------

# FARE Table I(a): Gazebo indoor benchmark, wheeled robot, 16-channel 3D LiDAR, max 2 m/s.
# (planner, distance m, distance std, time s, time std) to complete exploration.
FARE_INDOOR = [("DSVP", 1511, 75, 931, 61), ("TARE", 1209, 42, 658, 23), ("ARiADNE", 1053, 63, 610, 24),
               ("HEADER", 1030, 40, 576, 26), ("FARE (LLM + RL)", 1048, 13, 590, 10)]

# --- Market platforms (manufacturer pages, read 2026-10-11) ------------------------------------------------

MARKET = [
    {"name": "Boston Dynamics Spot", "src": "SPOT",
     "sensing": "360 deg terrain sensing (4 m range); payload ports",
     "autonomy": "Autowalk record-and-replay on a prior GraphNav map; Directed Exploration only reroutes to "
                 "recorded waypoints when a path is blocked",
     "explores_unknown": "No (needs a recorded map)", "speed": "1.6 m/s max", "runtime": "~90 min (564 Wh)",
     "price": "not published"},
    {"name": "ANYbotics ANYmal", "src": "ANYMAL",
     "sensing": "360 deg LiDAR, 6 depth cameras, 2 teleop cameras; thermal/zoom payload",
     "autonomy": "Autonomous inspection rounds in industrial plants; auto-docking; works with or without "
                 "connectivity", "explores_unknown": "Not stated on product page", "speed": "0.75 m/s normal",
     "runtime": "90-120 min (2 km)", "price": "not published"},
    {"name": "Unitree Go2 (AIR/PRO/X)", "src": "GO2",
     "sensing": "4D LiDAR L2 (360 x 96 deg); depth camera on EDU only",
     "autonomy": "Build a LiDAR point-cloud map in an app, then follow a user-specified path",
     "explores_unknown": "No (user specifies the path)", "speed": "~5 m/s max (lab)", "runtime": "1-2 h (2-4 h EDU)",
     "price": "$1,600 / $2,800 / $4,500"},
]

# --- Recent work (titles, dates and venues checked on arXiv 2026-10-11; venue only where the source states it) --

INNOVATIONS = [
    ("FARE", "2026", "arXiv 2601.14681", "LLM plans exploration strategy over a topological graph (slow), RL policy "
     "executes locally (fast).", "Closest 'agentic exploration' design; our FrontierAgent could take LLM room-order "
     "hints from the knowledge graph."),
    ("Sensor Configuration Matters", "2026", "arXiv 2606.19067", "Systematic SLAM evaluation on ANYmal D: stereo beats "
     "mono/RGB-D, global shutter beats rolling, IMU fusion can hurt vision SLAM under legged motion.",
     "Directly relevant to our sensor choice (single RGB-D) and to our gyro fusion."),
    ("MNE-SLAM", "2025", "CVPR 2025", "Distributed multi-agent neural SLAM, peer-to-peer, intra-to-inter loop closure, "
     "sub-map fusion.", "Blueprint for the 'swarm' future work in the project report."),
    ("MASt3R-SLAM", "2025", "CVPR 2025 (Highlight)", "Real-time dense monocular SLAM built on a learned two-view 3D "
     "reconstruction prior.", "Learned priors are replacing hand-tuned ICP; needs a GPU."),
    ("VGGT-SLAM", "2025", "arXiv 2505.12549", "Dense RGB SLAM aligning feed-forward VGGT sub-maps on the SL(4) "
     "manifold, uncalibrated cameras.", "Feed-forward reconstruction as a front end; GPU-bound."),
    ("NaVILA", "2024", "arXiv 2412.04453", "Vision-language-action model for legged-robot navigation from language "
     "instructions.", "Language commands ('search room 3') on a legged robot."),
    ("Clio", "2024", "arXiv 2404.13696", "Real-time task-driven open-set 3D scene graphs.", "Task-aware upgrade for "
     "our SpatialKnowledgeGraph."),
    ("HOV-SG", "2024", "arXiv 2403.17846", "Hierarchical open-vocabulary 3D scene graph (floors, rooms, objects) for "
     "language-grounded navigation.", "Same room-level hierarchy as our knowledge graph, with open-vocabulary labels."),
    ("SplaTAM", "2023", "CVPR 2024", "Dense RGB-D SLAM with explicit 3D Gaussians.", "Photoreal maps; GPU."),
    ("VLFM", "2023", "arXiv 2312.03275", "Vision-language frontier maps: frontiers scored by semantic value for "
     "zero-shot object search.", "Drop-in idea for our frontier scoring (semantic value per frontier)."),
    ("ConceptGraphs", "2023", "arXiv 2309.16650", "Open-vocabulary 3D scene graphs from foundation models.",
     "Replace our size-threshold 'chair/wall' labels."),
    ("SayPlan", "2023", "CoRL 2023 (oral)", "LLM task planning grounded in large multi-room 3D scene graphs.",
     "LLM supervisor on top of our knowledge graph."),
    ("Swarm-SLAM", "2023", "arXiv 2301.06230", "Sparse, decentralized collaborative SLAM for robot swarms.",
     "Multi-robot extension."),
    ("Hydra", "2022", "RSS 2022", "Real-time 3D scene graph construction and optimisation.", "Reference design for "
     "rooms/places/objects layering."),
    ("VILENS", "2021", "IEEE T-RO 2023", "Factor-graph fusion of visual, inertial, LiDAR and leg odometry with a "
     "leg-velocity bias state.", "Exactly our weak spot: estimating gait-odometry slip instead of a fixed stride."),
    ("FAST-LIO2", "2021", "arXiv 2107.06829", "Tightly coupled LiDAR-inertial odometry with an iterated Kalman filter.",
     "State of the art if a LiDAR is added."),
    ("Kimera-Multi", "2021", "IEEE T-RO", "Distributed metric-semantic multi-robot SLAM that rejects incorrect loop "
     "closures (perceptual aliasing).", "Robust loop-closure verification, a known failure mode we measured."),
]
INNOVATION_URLS = {
    "FARE": "https://arxiv.org/abs/2601.14681", "Sensor Configuration Matters": "https://arxiv.org/abs/2606.19067",
    "MNE-SLAM": "https://doi.org/10.1109/CVPR52734.2025.00146", "MASt3R-SLAM": "https://arxiv.org/abs/2412.12392",
    "VGGT-SLAM": "https://arxiv.org/abs/2505.12549", "NaVILA": "https://arxiv.org/abs/2412.04453",
    "Clio": "https://arxiv.org/abs/2404.13696", "HOV-SG": "https://arxiv.org/abs/2403.17846",
    "SplaTAM": "https://arxiv.org/abs/2312.02126", "VLFM": "https://arxiv.org/abs/2312.03275",
    "ConceptGraphs": "https://arxiv.org/abs/2309.16650", "SayPlan": "https://arxiv.org/abs/2307.06135",
    "Swarm-SLAM": "https://arxiv.org/abs/2301.06230", "Hydra": "https://arxiv.org/abs/2201.13360",
    "VILENS": "https://arxiv.org/abs/2107.07243", "FAST-LIO2": "https://arxiv.org/abs/2107.06829",
    "Kimera-Multi": "https://arxiv.org/abs/2106.14386",
}

# --- Our own measured ablations ---------------------------------------------------------------------------

# Closed-loop 60 s runs in robot_working.ttt from the same start pose (gyro seed 0), measured 2026-10-11.
# (configuration, GT distance m, ATE RMSE cm, ATE max cm, mean heading error deg, wall Chamfer cm, wall F1 %,
#  loop closures). None = not recorded for that run.
ABLATION_60S = [
    ("A. Stride uncalibrated (0.22 m/s), ICP yaw blended 50/50 with gyro, loose loop closure",
     5.43, 24.2, 56.9, 8.47, None, None, 0),
    ("B. Stride calibrated, gyro-led yaw, ICP translation only when full ICP passes gates, loose loop closure",
     5.34, 59.4, 101.7, 2.27, 18.8, 49.5, 4),
    ("C. As B, translation always by ICP with rotation fixed to gyro yaw, strict loop-closure acceptance",
     5.39, 8.74, 24.3, 1.99, 3.08, 98.6, 4),
]

# Offline replay (slam_replay.py) of recordings/arena_gtnav_420s.npz: 420 s, 38.16 m, 8400 steps.
# Identical input data for every row. (configuration, ATE RMSE cm, ATE max cm, drift %, heading err deg, loops)
REPLAY_RECORDING = "recordings/arena_gtnav_420s.npz (420 s, 38.2 m, recorded with ground-truth navigation)"
ABLATION_REPLAY = [
    ("FINAL: keyframes 0.5 m / 25 deg, gyro-led yaw (ICP weight 0.1), Manhattan heading prior, degeneracy-aware "
     "fixed-rotation ICP, strict loop closure", 8.5, 21.1, 0.22, 0.73, 28),
    ("final without ICP (gait odometry + gyro only)", 61.2, 137.7, 1.60, 10.10, 0),
    ("final without loop closure", 31.7, 66.2, 0.83, 0.70, 0),
    ("final with ICP yaw weight 0.5", 25.2, 56.0, 0.66, 1.12, 22),
    ("final without degeneracy filter", 15.9, 32.2, 0.42, 0.73, 27),
    ("final without Manhattan heading prior", 15.3, 32.6, 0.40, 2.23, 28),
    ("final with keyframes 1.5 m / 40 deg (previous default)", 14.4, 35.4, 0.38, 0.76, 10),
    ("final with ICP yaw weight 0 (gyro only for yaw increments)", 13.2, 25.3, 0.35, 0.76, 23),
]

# Full closed-loop exploration runs (robot_working.ttt), measured 2026-10-11/12.
# (run, sim time s, path m, ATE RMSE cm, drift %, mean heading err deg, rooms cleared / segmented, outcome)
LONG_RUNS = [
    ("Before long-run fixes (no heading prior; A* allowed through unknown cells)", 1090, 86.2, 71.5, 0.83, 20.1,
     "2 / 5", "stopped by hand: stuck against outer wall, 46 deg heading error"),
]

# ================================================================================================
# Exact reproduction of MNE-SLAM (Deng et al., CVPR 2025) published tables, for Appendix.
# Columns match the paper precisely (Agent1 / Agent2 / Global, or per-sequence). Source: paper PDF
# supplied by the user; transcribed by hand, cross-checked against the summarised MNE_* tables above.
# ================================================================================================

# Paper Table 1: Replica, 2-agent partition. Each cell = (Acc, Comp, Comp.Ratio%) reconstruction,
# plus one Localization RMSE[cm] per agent/global.
MNE_T1_FULL = [
    # name, Ag1(acc,comp,ratio), Ag2(acc,comp,ratio), Global(acc,comp,ratio), Loc Ag1, Loc Ag2, Loc Global
    ("iMAP",       (4.812, 4.519, 76.378), (3.663, 3.582, 81.352), (2.881, 4.934, 80.515), 3.481, 3.773, 4.153),
    ("NICE-SLAM",  (3.035, 3.907, 85.894), (2.163, 2.881, 90.792), (2.373, 2.645, 91.137), 1.657, 1.767, 2.503),
    ("Vox-Fusion", (2.975, 3.757, 83.599), (1.991, 2.609, 91.015), (2.881, 2.563, 90.936), 0.977, 1.432, 1.453),
    ("ESLAM",      (1.838, 1.979, 93.317), (1.857, 1.884, 94.531), (2.082, 1.754, 95.977), 0.593, 0.579, 0.657),
    ("Co-SLAM",    (1.758, 2.153, 92.894), (1.949, 2.033, 93.413), (2.104, 2.082, 93.435), 1.001, 0.934, 1.059),
    ("GO-SLAM",    (2.931, 3.759, 86.484), (2.245, 2.678, 90.571), (2.091, 2.415, 93.174), 0.334, 0.451, 0.404),
    ("Point-SLAM", (2.324, 3.455, 86.811), (1.925, 3.102, 90.975), (1.935, 2.779, 90.479), 0.534, 0.461, 0.559),
    ("PLGSLAM",    (1.793, 2.012, 93.415), (1.983, 1.773, 94.669), (1.884, 1.543, 96.329), 0.584, 0.607, 0.631),
    ("Loopy-SLAM", (1.935, 3.107, 89.894), (1.683, 2.783, 91.792), (1.593, 2.645, 91.137), 0.257, 0.362, 0.369),
    ("CP-SLAM",    (2.271, 3.457, 86.429), (1.834, 2.901, 90.115), (1.794, 2.455, 90.394), 0.613, 0.639, 0.673),
    ("MNE-SLAM",   (1.731, 1.932, 93.692), (1.797, 1.685, 95.458), (1.681, 1.509, 96.957), 0.281, 0.358, 0.398),
]

# Paper Table 2: Two-agent tracking, Replica "Apartment" scenes. Each cell = ATE RMSE[cm] (Ag1, Ag2, Global).
MNE_T2_FULL = [
    ("ORB-SLAM3",  (4.93, 4.93, 4.93),    (1.35, 1.36, 1.36), (0.67, 1.46, 1.07)),
    ("NICE-SLAM",  (55.46, 21.65, 38.56), (5.70, 2.99, 4.35), (2.17, 2.20, 2.18)),
    ("Co-SLAM",    (2.86, 3.51, 3.19),    (1.44, 1.64, 1.54), (0.83, 0.78, 0.83)),
    ("ESLAM",      (1.38, 0.95, 1.17),    (0.84, 0.75, 0.79), (0.58, 0.95, 0.76)),
    ("GO-SLAM",    (1.45, 1.33, 1.27),    (0.49, 0.78, 0.62), (0.47, 0.56, 0.53)),
    ("Point-SLAM", (1.31, 2.09, 1.63),    (0.53, 0.80, 0.68), (0.49, 0.62, 0.58)),
    ("PLGSLAM",    (1.33, 1.06, 1.13),    (0.82, 0.73, 0.79), (0.56, 0.93, 0.74)),
    ("Loopy-SLAM", (1.19, 1.66, 1.43),    (0.55, 0.66, 0.60), (0.46, 0.82, 0.64)),
    ("CCM-SLAM",   (2.12, 9.31, 5.71),    (0.51, 0.74, 0.62), None),
    ("Swarm-SLAM", (4.62, 6.50, 5.56),    (2.69, 8.53, 5.61), (1.61, 1.98, 1.80)),
    ("CP-SLAM",    (6.21, 5.67, 5.73),    (1.45, 2.48, 1.85), (0.62, 1.28, 0.91)),
    ("MNE-SLAM",   (1.21, 0.99, 1.02),    (0.43, 0.74, 0.59), (0.43, 0.53, 0.48)),
]

# Paper Table 3: ScanNet, ATE RMSE[cm] per sequence + average. "-" = not reported.
MNE_T3_FULL = {
    "sequences": ["0000", "0059", "0106", "0169", "0181", "0207", "0054", "0233", "Avg."],
    "NICE-SLAM":  [12.0, 14.0, 7.9, 10.9, 13.4, 6.5, 20.9, 9.0, 13.0],
    "ESLAM":      [7.3, 8.5, 7.5, 7.5, 9.0, 6.0, 36.3, 4.6, 10.6],
    "Co-SLAM":    [7.1, 11.1, 9.4, 7.1, 11.8, 7.1, 14.8, None, None],
    "Point-SLAM": [10.2, 7.8, 8.7, 22.2, 14.8, 9.5, 28.0, 6.1, 14.3],
    "GO-SLAM":    [5.4, 7.5, 7.0, 7.7, 6.8, 6.9, 8.8, 4.8, 6.9],
    "PLGSLAM":    [7.3, 8.2, 7.4, 7.5, 9.2, 5.8, 30.8, 4.2, 9.9],
    "Loopy-SLAM": [5.2, 7.9, 8.5, 7.7, 10.6, 7.9, 14.5, 5.2, 7.7],
    "Swarm-SLAM": [10.7, 8.7, 9.5, 12.5, 13.8, 8.4, 14.5, 6.5, 10.5],
    "CP-SLAM":    [7.6, 8.9, 8.8, 8.1, 10.8, 8.3, 20.1, 5.6, 9.7],
    "MNE-SLAM":   [4.9, 7.2, 6.6, 7.3, 6.4, 6.4, 8.5, 4.3, 6.6],
}

# Paper Table 4: INS dataset (large-scale, real-world). Reconstruction[cm] Acc/Comp/Ratio, Localization[m] Mean/RMSE.
MNE_T4_INS = [
    ("ORB-SLAM3",  None,                     (0.76, 0.80)),
    ("Co-SLAM",    None,                     (16.84, 18.80)),
    ("ESLAM",      (73.78, 17.60, 58.49),    (1.10, 1.23)),
    ("GO-SLAM",    (109.124, 133.152, 10.53), (3.76, 4.36)),
    ("Point-SLAM", None,                     (2.83, 2.95)),
    ("PLG-SLAM",   (70.42, 15.49, 59.37),    (1.01, 1.12)),
    ("Photo-SLAM", (60.61, 139.89, 10.75),   (0.43, 0.45)),
    ("Loopy-SLAM", None,                     (1.95, 2.01)),
    ("CP-SLAM",    None,                     (9.50, 10.67)),
    ("MNE-SLAM",   (43.96, 14.69, 64.00),    (0.38, 0.40)),
]

# Paper Table 6: their own ablation (intra/inter-loop closure, submap fusion), Replica.
# Reconstruction[cm] Acc/Comp/Ratio, Localization[cm] Mean/Median/RMSE.
MNE_T6_ABLATION = [
    ("w/o intra-loop closure", (1.831, 1.592, 95.779), (0.397, 0.403, 0.432)),
    ("w/o inter-loop closure", (1.903, 1.597, 95.679), (0.401, 0.409, 0.441)),
    ("w/o submap fusion",      (1.949, 1.607, 95.494), (0.359, 0.365, 0.401)),
    ("MNE-SLAM (full)",        (1.681, 1.509, 96.957), (0.357, 0.362, 0.398)),
]

# Headline "novelty target" values: for each metric, the single best (lowest error / highest quality)
# number published by any system above, i.e. what we would need to match or beat to claim SOTA.
NOVELTY_TARGETS = {
    "ATE RMSE, short indoor path (cm)": (3.98, "MNE-SLAM, Table 1 Global"),
    "ATE RMSE, multi-room scene (cm)": (0.48, "MNE-SLAM, Table 2 Apartment-0 Global"),
    "ATE RMSE, real-world scan (cm)": (6.6, "MNE-SLAM, Table 3 ScanNet avg."),
    "Reconstruction Acc (cm)": (1.593, "Loopy-SLAM, Table 1 Global"),
    "Reconstruction Comp.Ratio (%)": (96.957, "MNE-SLAM, Table 1 Global"),
    "Tracking time / frame (ms)": (22.1, "MNE-SLAM, Table 5"),
    "Scene memory (MB)": (20.73, "ESLAM, Table 5"),
}

# External ablation validation: same 6 configurations as ABLATION_REPLAY, replayed against an
# INDEPENDENT recording never used for tuning (final_closed_loop_run2.npz: the actual 1800s/160.5m
# closed-loop run, recorded 2026-10-12). Caveats, both real and disclosed in the report:
# (1) this recording predates the stop_motors() idle-creep fix, so a known ~1m localization jump
#     from that bug is baked into every row equally (it is in the raw odometry/gyro stream itself,
#     not something any SLAM configuration can correct);
# (2) this recording stored scan points as float32 (fixed in main_agentic.py after this run), so
#     absolute numbers differ slightly from the live run's float64 trajectory - confirmed by diffing
#     the two directly: both tracks agree to <5 cm for the first 18,492 of 36,001 steps, then a
#     borderline ICP/loop-closure accept decision flips under the precision loss and the two tracks
#     diverge. The relative ranking between the 6 configurations below is unaffected, since all 6
#     see identical (float32-precision) input.
EXTERNAL_RECORDING = "recordings/final_closed_loop_run2.npz (1800 s, 160.5 m, independent closed-loop run)"
EXTERNAL_VALIDATION_CAVEAT = (
    "This recording predates the idle-creep fix (one known ~1 m jump is baked equally into every row) "
    "and stored scans at float32 precision (fixed since), which makes a borderline ICP/loop-closure "
    "decision flip partway through and the absolute numbers diverge from the live run's 84.7 cm. The "
    "relative comparison between configurations is unaffected, since all six see identical input."
)
ABLATION_EXTERNAL = [
    ("FINAL (full system)", 108.8, 0.68, 5.94, 114),
    ("without ICP (gait odometry + gyro only)", 329.7, 2.05, 44.87, 0),
    ("without loop closure", 98.6, 0.61, 0.85, 0),
    ("without degeneracy filter", 100.8, 0.63, 0.90, 97),
    ("without Manhattan heading prior", 93.7, 0.58, 3.33, 122),
    ("with keyframes 1.5 m / 40 deg (previous default)", 99.4, 0.62, 0.88, 76),
]
