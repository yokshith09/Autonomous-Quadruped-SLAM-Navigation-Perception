# Main references

The papers this project builds on and compares against. Each entry says where it is used.
Before citing RTAB-Map (12) and OpenVSLAM (13), check the details against the original papers: they are not stored
in this repository.

## Methods our system is built on

1. P. J. Besl, N. D. McKay, "A Method for Registration of 3-D Shapes," *IEEE Trans. Pattern Analysis and Machine
   Intelligence*, vol. 14, no. 2, pp. 239-256, 1992.
   Used for: ICP scan matching (basis).
2. A. Censi, "An ICP Variant Using a Point-to-Line Metric," *Proc. IEEE ICRA*, pp. 19-25, 2008.
   Used for: our point-to-line scan matcher.
3. G. Grisetti, R. Kümmerle, C. Stachniss, W. Burgard, "A Tutorial on Graph-Based SLAM," *IEEE Intelligent
   Transportation Systems Magazine*, vol. 2, no. 4, pp. 31-43, 2010.
   Used for: pose graph and loop closure.
4. P. Agarwal, G. D. Tipaldi, L. Spinello, C. Stachniss, W. Burgard, "Robust Map Optimization Using Dynamic
   Covariance Scaling," *Proc. IEEE ICRA*, 2013.
   Used for: robust pose graph (DCS) against wrong loop closures.
5. A. Elfes, "Using Occupancy Grids for Mobile Robot Perception and Navigation," *Computer*, vol. 22, no. 6,
   pp. 46-57, 1989.
   Used for: Bayesian occupancy-grid map.
6. B. Yamauchi, "A Frontier-Based Approach for Autonomous Exploration," *Proc. IEEE CIRA*, pp. 146-151, 1997.
   Used for: frontier exploration, and the nearest-frontier baseline we compare with.
7. P. E. Hart, N. J. Nilsson, B. Raphael, "A Formal Basis for the Heuristic Determination of Minimum Cost Paths,"
   *IEEE Trans. Systems Science and Cybernetics*, vol. 4, no. 2, pp. 100-107, 1968.
   Used for: A* path planning.
8. C. Cadena, L. Carlone, H. Carrillo, Y. Latif, D. Scaramuzza, J. Neira, I. Reid, J. J. Leonard, "Past, Present, and
   Future of Simultaneous Localization and Mapping: Toward the Robust-Perception Age," *IEEE Trans. Robotics*,
   vol. 32, no. 6, pp. 1309-1332, 2016.
   Used for: SLAM background.

## Systems we compared against

9. W. Hess, D. Kohler, H. Rapp, D. Andor, "Real-Time Loop Closure in 2D LIDAR SLAM," *Proc. IEEE ICRA*,
   pp. 1271-1278, 2016.
   Used for: Cartographer, run on our recordings.
10. B. Steux, O. El Hamzaoui, "tinySLAM: A SLAM Algorithm in Less Than 200 Lines C-Language Program,"
    *Proc. ICARCV*, 2010. Software: BreezySLAM, github.com/simondlevy/BreezySLAM.
    Used for: BreezySLAM baseline, run on our recordings.
11. C. Campos, R. Elvira, J. J. G. Rodríguez, J. M. M. Montiel, J. D. Tardós, "ORB-SLAM3: An Accurate Open-Source
    Library for Visual, Visual-Inertial and Multi-Map SLAM," *IEEE Trans. Robotics*, vol. 37, no. 6, pp. 1874-1890,
    2021.
    Used for: published system on TUM Pioneer.
12. M. Labbé, F. Michaud, RTAB-Map (original paper; check details before citing).
    Used for: published system on TUM Pioneer.
13. S. Sumikura et al., OpenVSLAM (original paper; check details before citing).
    Used for: published system on TUM Pioneer.

## Benchmark, metrics and simulator

14. J. Sturm, N. Engelhard, F. Endres, W. Burgard, D. Cremers, "A Benchmark for the Evaluation of RGB-D SLAM Systems,"
    *Proc. IEEE/RSJ IROS*, 2012.
    Used for: TUM RGB-D dataset (Pioneer sequences), ATE metric.
15. A. Merzlyakov, S. Macenski, "A Comparison of Modern General-Purpose Visual SLAM Approaches,"
    arXiv:2107.07589, 2021.
    Used for: published TUM Pioneer results of RTAB-Map, OpenVSLAM and ORB-SLAM3.
16. R. Kümmerle, B. Steder, C. Dornhege, M. Ruhnke, G. Grisetti, C. Stachniss, A. Kleiner, "On Measuring the Accuracy
    of SLAM Algorithms," *Autonomous Robots*, vol. 27, no. 4, pp. 387-407, 2009.
    Used for: ATE / RPE evaluation.
17. E. Rohmer, S. P. N. Singh, M. Freese, "V-REP: A Versatile and Scalable Robot Simulation Framework,"
    *Proc. IEEE/RSJ IROS*, pp. 1321-1326, 2013.
    Used for: simulator (now CoppeliaSim).

## Related work for context (optional)

18. R. Corlito et al., "Sensor Configuration Matters: A Systematic Evaluation of Multimodal SLAM on Quadruped Robots,"
    arXiv:2606.19067, 2026.
    Used for: quadruped SLAM state of the art (lidar-based).
19. S. Liao et al., "FARE: Fast-Slow Agentic Robotic Exploration," arXiv:2601.14681, 2026.
    Used for: modern exploration planners (TARE, DSVP, ARiADNE).
20. Z. Zhang et al., "Autonomous Exploration-Based Precise Mapping through Stepwise and Consistent Motions,"
    arXiv:2503.17005.
    Used for: turn-in-place and patrol design.

The full list with design notes (about 40 related papers) is in `docs/project/RESEARCH.md`.
