"""Comparison tables: our measured values next to published numbers quoted from benchmarks.py (each tied to its source).

Every function returns (headers, rows, widths). A first column starting with '<b>' marks our own row. 'derived' marks a value
obtained by dividing two published numbers. Nothing here is typed in from memory: ours come from the run, theirs from benchmarks.py.
"""
from evaluation import benchmarks as B

REF = {"MNE": 1, "ORB2": 2, "A1": 3, "QUAD": 4, "FARE": 5, "SPOT": 6, "ANYMAL": 7, "GO2": 8}


def ref_list():
    return [f"[{n}] {B.SOURCES[k][1]} {B.SOURCES[k][2]}" for k, n in REF.items()]


def _ours(d, x):
    m = d["m"]
    return m["localization"], m["mapping"], x["agents"], m["compute"]


def slam_combined(d, x):
    """Pose + map + cost together: the SLAM comparison against RGB-D systems evaluated on Replica."""
    loc, mp, ag, c = _ours(d, x)
    runtime = {n: (ms, mem, trk) for n, ms, trk, mem in B.MNE_RUNTIME}
    rows = [("<b>Ours</b>", "simulated 8-room arena, 158 m; forward RGB-D to 2D scan + gait odometry + gyro; 2D occupancy grid; CPU [this work]",
             f"{loc['ate_rmse_m'] * 100:.1f}", f"{mp['wall_accuracy_m'] * 100:.1f} / {mp['wall_completion_m'] * 100:.1f} / {mp['completion_ratio_5cm_pct']:.1f}",
             f"{ag['SLAMAgent']['p50_ms']:.1f}", f"{ag['MappingAgent']['p50_ms'] / 1000:.3f}", f"{mp['map_memory_mb']:.2f}")]
    for name in ("MNE-SLAM", "Loopy-SLAM", "GO-SLAM", "ESLAM", "Co-SLAM", "Point-SLAM"):
        n, acc, comp, ratio, ate = next(r for r in B.MNE_REPLICA if r[0] == name)
        rt = runtime.get(name)
        rows.append((name, "Replica scenes, RGB-D, neural 3D map, GPU [1]", f"{ate:.3f}", f"{acc:.3f} / {comp:.3f} / {ratio:.1f}",
                     f"{rt[2]:.1f}" if rt else "-", f"{rt[0]:.2f}" if rt else "-", f"{rt[1]:.2f}" if rt else "-"))
    h = ["System", "Test conditions", "Pose: ATE RMSE (cm)", "Map: accuracy / completion (cm) / completion ratio (%)",
         "Tracking per frame (ms)", "Mapping per frame (s)", "Map memory (MB)"]
    return h, rows, [70, 150, 44, 92, 44, 44, 44]


def apartment(d, x):
    loc, _, _, _ = _ours(d, x)
    rows = [("<b>Ours</b>", "own 8-room arena, 158 m [this work]", f"{loc['ate_rmse_m'] * 100:.1f}")]
    for name in ("MNE-SLAM", "ESLAM", "Loopy-SLAM", "ORB-SLAM3", "Swarm-SLAM", "CP-SLAM"):
        _, a1, a2, a0 = next(r for r in B.MNE_APARTMENT if r[0] == name)
        rows.append((name, "Replica multi-room apartments 1 / 2 / 0 [1]", " / ".join("-" if v is None else f"{v:.2f}" for v in (a1, a2, a0))))
    return ["System", "Test conditions", "ATE RMSE (cm)"], rows, [90, 250, 120]


def quadruped_pose(d, x):
    loc, _, _, c = _ours(d, x)
    mean_dist = sum(B.QUAD_MISSION_DIST_M) / len(B.QUAD_MISSION_DIST_M)
    a1 = B.A1_SHORT_TRAJECTORIES
    rows = [("<b>Ours</b>", "simulation, simulator pose [this work]", f"{x['dist']:.0f}", f"{loc['ate_rmse_m'] * 100:.1f}",
             f"{x['ate_pct_path']:.2f}", f"{c['process_cpu_pct_mean']:.0f} / {c['rss_mb_mean'] / 1000:.1f}"),
            ("A1 SLAM", "real Unitree A1, lab, motion capture [3]", f"{min(r[1] for r in a1):.1f}-{max(r[1] for r in a1):.1f}",
             f"{min(r[4] for r in a1) * 100:.1f}-{max(r[4] for r in a1) * 100:.1f}",
             f"{min(r[4] / r[1] for r in a1) * 100:.1f}-{max(r[4] / r[1] for r in a1) * 100:.1f}", "-"),
            ("Cartographer", "real Unitree A1, lab, motion capture [3]", f"{min(r[1] for r in a1):.1f}-{max(r[1] for r in a1):.1f}",
             f"{min(r[3] for r in a1) * 100:.1f}-{max(r[3] for r in a1) * 100:.1f}",
             f"{min(r[3] / r[1] for r in a1) * 100:.1f}-{max(r[3] / r[1] for r in a1) * 100:.1f}", "-")]
    for meth, sens, ate, _, cpu, ram in B.QUAD_SLAM:
        if (meth, sens) in (("FAST-LIVO2", "LiDAR + camera + IMU (Honeywell)"), ("ORB-SLAM3", "stereo (global shutter)"),
                            ("RTAB-Map", "stereo (global shutter)"), ("RTAB-Map", "RGB-D (ZED2i)"), ("ORB-SLAM3", "RGB-D (ZED2i)")):
            note = " (successful runs only)" if (meth, sens) == ("RTAB-Map", "RGB-D (ZED2i)") else ""
            rows.append((f"{meth}, {sens}", "real ANYmal D, mostly outdoor, 7 missions [4]", f"{min(B.QUAD_MISSION_DIST_M):.0f}-{max(B.QUAD_MISSION_DIST_M):.0f}",
                         f"{ate * 100:.1f}{note}", f"{ate / mean_dist * 100:.2f}", f"{cpu:.0f} / {ram:.2f}" if cpu else "-"))
    return (["System", "Test conditions", "Path (m)", "ATE RMSE (cm)", "ATE / path (%, derived)", "CPU (%) / RAM (GB)"], rows, [120, 120, 50, 80, 60, 60])


def runtime(d, x):
    _, _, ag, c = _ours(d, x)
    rows = [("<b>Ours</b>", "SLAM update, median", f"{ag['SLAMAgent']['p50_ms']:.1f} ms", f"CPU only, {d['man'].get('cpu')} [this work]"),
            ("ORB-SLAM2 (RGB-D)", "tracking per frame", f"{B.ORB2_TRACK_MS} ms", "Intel Core i7-4790, TUM fr3_office [2]"),
            ("MNE-SLAM", "tracking per frame", "22.1 ms", "authors' hardware [1]"),
            ("A1 SLAM / Cartographer", "optimization per scan", "1.65 / 0.96 ms", "optimizer call only [3]")]
    return ["System", "Quantity", "Value", "Hardware / note"], rows, [110, 100, 80, 190]


def exploration(d, x):
    o = d["m"]["overall"]
    rows = [("<b>Ours</b>", "quadruped, forward RGB-D, 10 x 10 m arena [this work]", f"{x['dist']:.0f}", f"{o['sim_time_s']:.0f} (not completed)", f"{x['speed']:.3f}")]
    for n, dist, _, t, _ in B.FARE_INDOOR:
        rows.append((n, "wheeled robot, 16-channel 3D LiDAR, Gazebo indoor, max 2 m/s [5]", f"{dist}", f"{t}", f"{dist / t:.2f} (derived)"))
    rows += [("ANYmal (product page)", "quadruped, 360 deg LiDAR + depth cameras [7]", "-", "-", "0.75 normal walking"),
             ("Spot (product page)", "quadruped, 360 deg terrain sensing [6]", "-", "-", "1.6 maximum"),
             ("Go2 (product page)", "quadruped, 4D LiDAR [8]", "-", "-", "about 5 maximum (lab)")]
    return ["System", "Test conditions", "Distance (m)", "Time (s)", "Average speed (m/s)"], rows, [100, 220, 50, 70, 70]
