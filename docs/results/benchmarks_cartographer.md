# Cartographer on our recordings (benchmark 3f, 2026-11-15)

Runner: `python -m evaluation.cartographer_bench <recording> [--calibrate-s 62] [--lua "..."]`. Cartographer
(ROS 2 Humble package, Docker image carto-humble; the ROS 1 Noetic package is no longer in the ROS archive) runs in 2D
with odometry and no IMU on a ROS 2 bag made from our recording: /scan = the same 2D scan our SLAM gets (290 bins over
the 58 deg camera field of view), /odom = leg odometry + gyro with the stand-still gyro bias removed (identical to the
odometry-only baseline). Its online map->base_link pose is recorded, moved into the world frame by the start pose and
scored with the same ATE as ours. Frame check: at t = 90 s Cartographer is 1 cm from the truth.

| Setting | Scene 1 (run 34) | Scene 2 (run 19) | Average |
|---|---|---|---|
| Cartographer defaults (backpack_2d style) | 585.8 cm | 216.5 cm | 401.2 cm |
| + scan matcher trusts odometry 10x more | 343.8 cm | 95.1 cm | 219.5 cm |
| + online correlative scan matching | not run | 311.8 cm | |
| Odometry only | 291.4 cm | 15.7 cm | 153.6 cm |
| Ours | 11.7 cm | 12.3 cm | 12.0 cm |

Why it fails here: Cartographer is built for 360 deg laser scanners with dense returns. Our scan is a 58 deg depth
camera slice with about 210 points; its scan matcher pulls the pose towards wrong alignments and its loop closures
(logged "differs by translation 5.4 m") accept wrong matches. All settings ended worse than odometry alone.
Limit: 3 settings tried; a dedicated Cartographer tuning for narrow scans could do better.
