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
    "B2": ("Unitree", "Unitree Robotics, B2 product page and specification table.", "https://www.unitree.com/b2"),
    "X30": ("DEEP Robotics", "DEEP Robotics, X30 product page and specification table.",
            "https://en.deeprobotics.cn/en/index/product3.html"),
    "HOVERMAP": ("Emesent", "Emesent, Hovermap autonomy page.", "https://www.emesent.com/autonomy"),
    "MM": ("Merzlyakov & Macenski",
           "A. Merzlyakov, S. Macenski, \"A Comparison of Modern General-Purpose Visual SLAM Approaches\".",
           "https://arxiv.org/abs/2107.07589"),
}

# --- Like-for-like reference: TUM RGB-D Pioneer robot sequences -------------------------------------------
# Kinect on an ActivMedia Pioneer 3 wheeled robot, mounted horizontally looking forward, planar motion, motion-capture
# ground truth (Sturm et al., IROS 2012). Merzlyakov & Macenski Table IV, ATE RMSE (m), read 2026-10-25.
TUM_PIONEER_SEQS = ["fr2/pioneer_360", "fr2/pioneer_slam", "fr2/pioneer_slam2", "fr2/pioneer_slam3"]
TUM_PIONEER = [  # (method, ATE RMSE m per sequence in TUM_PIONEER_SEQS order, average as published)
    ("ORB-SLAM3 (RGB-D)", (1.059, 2.118, 1.709, 1.992), 1.720),
    ("OpenVSLAM (RGB-D)", (0.048, 0.188, 1.224, 0.106), 0.392),
    ("RTABMap (RGB-D)", (0.038, 0.351, 0.044, 0.716), 0.287),
]

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


# MNE-SLAM Table 2: multi-room Apartment scenes, global ATE RMSE (cm) for Apt-1, Apt-2, Apt-0.
MNE_APARTMENT = [
    ("ORB-SLAM3", 4.93, 1.36, 1.07), ("NICE-SLAM", 38.56, 4.35, 2.18), ("Co-SLAM", 3.19, 1.54, 0.83),
    ("ESLAM", 1.17, 0.79, 0.76), ("GO-SLAM", 1.27, 0.62, 0.53), ("Point-SLAM", 1.63, 0.68, 0.58),
    ("PLGSLAM", 1.13, 0.79, 0.74), ("Loopy-SLAM", 1.43, 0.60, 0.64), ("CCM-SLAM", 5.71, 0.62, None),
    ("Swarm-SLAM", 5.56, 5.61, 1.80), ("CP-SLAM", 5.73, 1.85, 0.91), ("MNE-SLAM", 1.02, 0.59, 0.48),
]


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
     "price": "not published", "compute": "not stated", "protection": "IP54"},
    {"name": "ANYbotics ANYmal", "src": "ANYMAL",
     "sensing": "360 deg LiDAR, 6 depth cameras, 2 teleop cameras; thermal/zoom payload",
     "autonomy": "Autonomous inspection rounds in industrial plants; auto-docking; works with or without "
                 "connectivity", "explores_unknown": "Not stated on product page", "speed": "0.75 m/s normal",
     "runtime": "90-120 min (2 km)", "price": "not published", "compute": "2x Intel Core i7", "protection": "IP67"},
    {"name": "Unitree Go2 (AIR/PRO/X)", "src": "GO2",
     "sensing": "4D LiDAR L2 (360 x 96 deg); depth camera on EDU only",
     "autonomy": "Build a LiDAR point-cloud map in an app, then follow a user-specified path",
     "explores_unknown": "No (user specifies the path)", "speed": "~5 m/s max (lab)", "runtime": "1-2 h (2-4 h EDU)",
     "price": "$1,600 / $2,800 / $4,500", "compute": "8-core CPU (EDU: optional Orin)", "protection": "not stated"},
]

# Read 2026-10-25 from the manufacturer pages in SOURCES. "Not stated" = the page does not say.
MARKET_2 = [
    {"name": "Unitree B2", "src": "B2", "sensing": "3D LiDAR x1, depth cameras x2, optical cameras x2 (varies by configuration)",
     "autonomy": "3D mapping, inspection and rescue applications; part of the functions need human operation or secondary development",
     "explores_unknown": "Not stated", "speed": "> 6 m/s (special configuration; limited in practice)", "runtime": "4-6 h",
     "compute": "Intel Core i5 / i7; optional Orin NX", "protection": "IP67", "price": "not published"},
    {"name": "DEEP Robotics X30", "src": "X30", "sensing": "integrated perception (sensors not listed)",
     "autonomy": "autonomous navigation in darkness, strong light and flicker; inspection, investigation, surveying and mapping",
     "explores_unknown": "Not stated", "speed": ">= 4 m/s", "runtime": "2.5-4 h", "compute": "not stated", "protection": "IP67",
     "price": "not published"},
    {"name": "Emesent Hovermap (payload, mainly drones)", "src": "HOVERMAP", "sensing": "LiDAR",
     "autonomy": "LiDAR SLAM with modes: pilot assist, autonomous waypoint, autonomous exploration; GPS-denied; onboard processing",
     "explores_unknown": "Yes (autonomous exploration mode)", "speed": "not stated", "runtime": "not stated",
     "compute": "onboard (not specified)", "protection": "not stated", "price": "not published"},
]
