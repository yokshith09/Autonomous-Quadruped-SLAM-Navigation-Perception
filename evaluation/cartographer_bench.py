#!/usr/bin/env python3
"""Cartographer (Hess et al., ICRA 2016) on our recordings, same inputs and scoring as evaluation/slam_replay.py.

  python -m evaluation.cartographer_bench runs/<run>/recording.npz [--calibrate-s 62] [--rate 4]

1. The recording becomes a ROS 2 bag: /scan (sensor_msgs/LaserScan, the same 2D scan our SLAM gets, binned over the
   camera field of view) and /odom (nav_msgs/Odometry: leg odometry + gyro, gyro bias from the stand-still
   calibration, exactly the dead-reckoning baseline).
2. cartographer_node (ROS 2 Humble, Docker image carto-humble) runs in 2D with odometry, no IMU, on simulated time
   while `ros2 bag play` feeds the bag; `ros2 bag record` keeps the map->base_link pose it publishes (online pose).
3. The pose is interpolated at our timestamps, moved into the world frame by the start pose, and scored with
   evaluate_trajectory (no alignment, like ours).
"""
import argparse
import math
import os
import shutil
import subprocess

import numpy as np

from evaluation.accuracy_evaluator import evaluate_trajectory
from evaluation.baselines import _odometry, _fov_deg

T0 = 1_700_000_000.0
N_BINS = 290
IMAGE = "carto-humble"

LUA = """include "map_builder.lua"
include "trajectory_builder.lua"
options = {
  map_builder = MAP_BUILDER, trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map", tracking_frame = "base_link", published_frame = "base_link", odom_frame = "odom",
  provide_odom_frame = false, publish_frame_projected_to_2d = true, use_pose_extrapolator = true,
  use_odometry = true, use_nav_sat = false, use_landmarks = false,
  num_laser_scans = 1, num_multi_echo_laser_scans = 0, num_subdivisions_per_laser_scan = 1, num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2, submap_publish_period_sec = 0.3, pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3, rangefinder_sampling_ratio = 1., odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1., imu_sampling_ratio = 1., landmarks_sampling_ratio = 1.,
}
MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.min_range = 0.3
TRAJECTORY_BUILDER_2D.max_range = %(rmax).2f
TRAJECTORY_BUILDER_2D.missing_data_ray_length = %(rmax).2f
%(extra)s
return options
"""

RUN_SH = """#!/bin/bash
source /opt/ros/humble/setup.bash
cd /data
ros2 run cartographer_ros cartographer_node -configuration_directory /data -configuration_basename ours.lua \\
  --ros-args -p use_sim_time:=true > carto.log 2>&1 &
C=$!
rm -rf out; ros2 bag record -o out /tf > rec.log 2>&1 &
R=$!
sleep 4
ros2 bag play in --clock 200 --rate %(rate)s > play.log 2>&1
sleep 4
kill -INT $R; sleep 2; kill -INT $C; sleep 1
echo DONE
"""


def write_bag(rec, path, calibrate_s):
    from rosbags.rosbag2 import Writer
    from rosbags.typesys import Stores, get_typestore
    ts = get_typestore(Stores.ROS2_HUMBLE)
    Scan, Odom = ts.types["sensor_msgs/msg/LaserScan"], ts.types["nav_msgs/msg/Odometry"]
    H, Time = ts.types["std_msgs/msg/Header"], ts.types["builtin_interfaces/msg/Time"]
    G = ts.types["geometry_msgs/msg/PoseWithCovariance"], ts.types["geometry_msgs/msg/TwistWithCovariance"]
    Pose, Point, Quat = ts.types["geometry_msgs/msg/Pose"], ts.types["geometry_msgs/msg/Point"], ts.types["geometry_msgs/msg/Quaternion"]
    Twist, Vec = ts.types["geometry_msgs/msg/Twist"], ts.types["geometry_msgs/msg/Vector3"]
    t, pts, offs = rec["t"], rec["scan_points"], rec["scan_offsets"]
    fov, rmax = _fov_deg(rec)
    edges = np.linspace(-fov / 2, fov / 2, N_BINS + 1)
    fwd, lat, dyaw, _ = _odometry(rec, calibrate_s)
    if os.path.exists(path):
        shutil.rmtree(path)
    x = y = yaw = 0.0
    cov = np.zeros(36)
    with Writer(path, version=8) as w:
        cs = w.add_connection("/scan", Scan.__msgtype__, typestore=ts)
        co = w.add_connection("/odom", Odom.__msgtype__, typestore=ts)
        for k in range(len(t)):
            st = T0 + float(t[k])
            sec = int(st)
            stamp = Time(sec=sec, nanosec=int((st - sec) * 1e9))
            q = pts[offs[k]:offs[k + 1]]
            rng = np.full(N_BINS, np.inf, np.float32)
            if len(q):
                a = np.degrees(np.arctan2(q[:, 1], q[:, 0]))
                i = np.clip(np.searchsorted(edges, a) - 1, 0, N_BINS - 1)
                np.minimum.at(rng, i, np.hypot(q[:, 0], q[:, 1]).astype(np.float32))
            msg = Scan(header=H(stamp=stamp, frame_id="base_link"), angle_min=math.radians(-fov / 2),
                       angle_max=math.radians(fov / 2), angle_increment=math.radians(fov / N_BINS),
                       time_increment=0.0, scan_time=0.05, range_min=0.3, range_max=float(rmax), ranges=rng,
                       intensities=np.zeros(0, np.float32))
            w.write(cs, int(st * 1e9), ts.serialize_cdr(msg, Scan.__msgtype__))
            om = Odom(header=H(stamp=stamp, frame_id="odom"), child_frame_id="base_link",
                      pose=G[0](pose=Pose(position=Point(x=x, y=y, z=0.0),
                                          orientation=Quat(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))),
                                covariance=cov),
                      twist=G[1](twist=Twist(linear=Vec(x=0.0, y=0.0, z=0.0), angular=Vec(x=0.0, y=0.0, z=0.0)),
                                 covariance=cov))
            w.write(co, int(st * 1e9), ts.serialize_cdr(om, Odom.__msgtype__))
            yaw += dyaw[k]
            x += fwd[k] * math.cos(yaw) - lat[k] * math.sin(yaw)
            y += fwd[k] * math.sin(yaw) + lat[k] * math.cos(yaw)
    return rmax


def read_poses(bag_dir):
    from rosbags.rosbag2 import Reader
    from rosbags.typesys import Stores, get_typestore
    ts = get_typestore(Stores.ROS2_HUMBLE)
    out = []
    with Reader(bag_dir) as r:
        for c, _, raw in r.messages():
            m = ts.deserialize_cdr(raw, c.msgtype)
            for tf in m.transforms:
                if tf.header.frame_id == "map" and tf.child_frame_id == "base_link":
                    s = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9 - T0
                    q = tf.transform.rotation
                    out.append((s, tf.transform.translation.x, tf.transform.translation.y,
                                math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))))
    return np.array(sorted(out))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("recording")
    ap.add_argument("--calibrate-s", type=float, default=0.0)
    ap.add_argument("--rate", type=float, default=4.0)
    ap.add_argument("--lua", action="append", default=[], help="extra Cartographer Lua line, e.g. TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true")
    ap.add_argument("--work", default="E:/project_phase1/third_party/cartographer/work")
    a = ap.parse_args()
    rec = np.load(a.recording)
    os.makedirs(a.work, exist_ok=True)
    rmax = write_bag(rec, os.path.join(a.work, "in"), a.calibrate_s)
    open(os.path.join(a.work, "ours.lua"), "w", newline="\n").write(LUA % {"rmax": rmax, "extra": "\n".join(a.lua)})
    open(os.path.join(a.work, "run.sh"), "w", newline="\n").write(RUN_SH % {"rate": a.rate})
    subprocess.run(["docker", "run", "--rm", "-v", f"{os.path.abspath(a.work)}:/data", IMAGE, "bash", "/data/run.sh"],
                   check=True)
    p = read_poses(os.path.join(a.work, "out"))
    t = rec["t"]
    x0, y0, yaw0 = rec["initial_pose"]
    px, py = np.interp(t, p[:, 0], p[:, 1]), np.interp(t, p[:, 0], p[:, 2])
    pyaw = np.interp(t, p[:, 0], np.unwrap(p[:, 3]))
    c, s = math.cos(yaw0), math.sin(yaw0)
    est = np.stack([x0 + c * px - s * py, y0 + s * px + c * py, yaw0 + pyaw], 1)
    m = evaluate_trajectory(rec["gt"], est, timestamps=t, save_plots=False)
    cover = (p[-1, 0] - p[0, 0]) / (t[-1] - t[0])
    print(f"  cartographer | poses {len(p)} covering {cover * 100:.0f} % of the run | ATE RMSE "
          f"{m['ate_rmse_meters'] * 100:.1f} cm | max {m['ate_max_meters'] * 100:.1f} cm", flush=True)


if __name__ == "__main__":
    main()
