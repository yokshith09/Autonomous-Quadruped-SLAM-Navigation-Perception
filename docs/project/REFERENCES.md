# Main references

The 10 papers this project rests on. The full list with design notes is in `docs/project/RESEARCH.md`.

1. A. Censi, "An ICP Variant Using a Point-to-Line Metric," *Proc. IEEE ICRA*, pp. 19-25, 2008.
   Our scan matcher.
2. G. Grisetti, R. Kümmerle, C. Stachniss, W. Burgard, "A Tutorial on Graph-Based SLAM," *IEEE Intelligent
   Transportation Systems Magazine*, vol. 2, no. 4, pp. 31-43, 2010.
   Pose graph and loop closure.
3. P. Agarwal, G. D. Tipaldi, L. Spinello, C. Stachniss, W. Burgard, "Robust Map Optimization Using Dynamic
   Covariance Scaling," *Proc. IEEE ICRA*, 2013.
   Robust pose graph against wrong loop closures.
4. A. Elfes, "Using Occupancy Grids for Mobile Robot Perception and Navigation," *Computer*, vol. 22, no. 6,
   pp. 46-57, 1989.
   The map.
5. B. Yamauchi, "A Frontier-Based Approach for Autonomous Exploration," *Proc. IEEE CIRA*, pp. 146-151, 1997.
   Frontier exploration; our nearest-frontier baseline.
6. W. Hess, D. Kohler, H. Rapp, D. Andor, "Real-Time Loop Closure in 2D LIDAR SLAM," *Proc. IEEE ICRA*,
   pp. 1271-1278, 2016.
   Cartographer, run on our data.
7. B. Steux, O. El Hamzaoui, "tinySLAM: A SLAM Algorithm in Less Than 200 Lines C-Language Program,"
   *Proc. ICARCV*, 2010.
   BreezySLAM baseline, run on our data.
8. J. Sturm, N. Engelhard, F. Endres, W. Burgard, D. Cremers, "A Benchmark for the Evaluation of RGB-D SLAM Systems,"
   *Proc. IEEE/RSJ IROS*, 2012.
   TUM RGB-D dataset and ATE metric.
9. A. Merzlyakov, S. Macenski, "A Comparison of Modern General-Purpose Visual SLAM Approaches,"
   arXiv:2107.07589, 2021.
   Published TUM Pioneer results of RTAB-Map, OpenVSLAM and ORB-SLAM3.
10. R. Kümmerle, B. Steder, C. Dornhege, M. Ruhnke, G. Grisetti, C. Stachniss, A. Kleiner, "On Measuring the
    Accuracy of SLAM Algorithms," *Autonomous Robots*, vol. 27, no. 4, pp. 387-407, 2009.
    How SLAM accuracy is measured (ATE / RPE).
