# Autonomous Quadruped SLAM — Research Benchmark Comparison


## Research Benchmark Comparison Table

| Metric | Our Quadruped (Measured) |
|--------|--------------------------|
| **2D Occupancy Reconstruction** | |
| Accuracy (SLAM→GT) | N/A |
| Completeness (GT→SLAM) | N/A |
| Completion Ratio @ 5 cm | N/A |
| Completion Ratio @ 10 cm | N/A |
| Completion Ratio @ 15 cm | N/A |
| Chamfer Distance | 5.12 cm |
| 95% Hausdorff Distance | 19.21 cm |
| **Localization** | |
| ATE RMSE | 5.70 cm |
| Mean ATE | 4.76 cm |
| Median ATE | N/A |
| Maximum ATE | 11.17 cm |
| RPE Translation RMSE | N/A |
| RPE Rotation RMSE | N/A |
| Drift Rate | 0.30 % |
| **Occupancy Map** | |
| Precision | 91.87 % |
| Recall | 94.76 % |
| F1 Score | 93.29 % |
| Obstacle IoU (Exact) | 27.39 % |
| Tolerant IoU (±15 cm) | 59.37 % |
| Free-space IoU | 85.23 % |
| Overall Grid Accuracy | 91.60 % |
| **Exploration** | |
| Explored Area | 48.00 % |
| Rooms Detected | 5 |
| Doorways Detected | 6 |
| Path Efficiency | 24.58 % |

> **Note**: All values in the 'Our Quadruped (Measured)' column are
> computed exclusively from real simulation data recorded by this system.
> No values have been hardcoded, interpolated from, or compared to
> published MNE-SLAM or any other paper results in this column.
> N/A entries indicate metrics not persisted during the simulation run.
