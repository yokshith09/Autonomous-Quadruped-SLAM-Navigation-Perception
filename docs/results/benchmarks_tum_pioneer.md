# TUM RGB-D Pioneer sequences (benchmark 3g, 2026-11-15)

Runner: `python -m evaluation.tum_pioneer E:/project_phase1/datasets/tum [--set ...]`. Each ROS bag is converted to our
recording format (depth -> 2D scan of the 0.15-0.80 m band, Kinect tilt -7.5 deg corrected; wheel odometry /pose ->
forward, lateral, turn; ground truth = Kinect pose on the floor plane) and replayed through SLAMAgent. ATE RMSE after a
rigid 2D alignment, as the TUM tool does in 3D. Conversion checked by building a map from the true poses (crisp walls).
Camera offset from the turning centre: 3.5-4.6 cm (negligible).

| System | Input | 360 | slam | slam2 | slam3 | Average |
|---|---|---|---|---|---|---|
| ORB-SLAM3 (RGB-D) [MM] | images | 105.9 | 211.8 | 170.9 | 199.2 | 172.0 |
| OpenVSLAM (RGB-D) [MM] | images | 4.8 | 18.8 | 122.4 | 10.6 | 39.2 |
| RTABMap (RGB-D) [MM] | images | 3.8 | 35.1 | 4.4 | 71.6 | 28.7 |
| Wheel odometry only | wheel odometry | 16.3 | 19.0 | 58.5 | 21.0 | 28.7 |
| Ours, scene-1 settings | depth scan + wheel odometry | 14.8 | 69.5 | 33.8 | 44.1 | 40.5 |
| Ours, scene-2 settings | depth scan + wheel odometry | 15.2 | 22.9 | 38.3 | 27.3 | 25.9 |

All in cm. [MM] Merzlyakov & Macenski, arXiv 2107.07589, Table IV.

Findings
- Scene-2 settings (no right-angle heading prior, 16-keyframe submap) give the best average of the table and are the
  only row without a failure (all sequences 15-38 cm), but they gain little over the Pioneer's own wheel odometry
  (25.9 vs 28.7 cm) and lose to it on slam and slam3.
- Scene-1 settings: the heading prior assumes walls at right angles to the start heading; it hurts in this hall.
- Not TUM-tuned; both existing profiles reported. Ours uses wheel odometry, the visual systems do not.
- Next: make scan matching gain clearly over good wheel odometry on real Kinect data (noise model, acceptance gate).
