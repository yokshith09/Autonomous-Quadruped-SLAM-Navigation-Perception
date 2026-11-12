# Research notes: recent work relevant to this robot

Searched 2026-11-07 with the Firecrawl paper index and web search, Consensus (Semantic Scholar, OpenAlex, PubMed) and fastwrite (Semantic Scholar abstracts and Exa scholarly web). Scholar Sidekick citation verification was not available (API not subscribed). The attached Perplexity connector exposes dashboard import tools only, no search.
Each entry says what the paper found and what we do with it here. Status refers to `FIXES.md` IDs.

## Localization and mapping

| Paper | Finding | Use here | Status |
|---|---|---|---|
| Vizzo et al., *KISS-ICP: In Defense of Point-to-Point ICP*, RA-L 2023, arXiv 2209.15397 | Correspondence distance threshold adapted from the recent deviation of the motion model, plus a robust kernel, gives accurate odometry with almost no tuning | Our dominant scan-match rejection (67 % of updates, F16) is a fixed 1 cm + 100 % of step gate against odometry. Tried as an adaptive gate (3 sigma of recent deviations): **worse on all 5 recordings** (e.g. 22.0 -> 32.8 cm, 17.8 -> 32.6 cm); a looser gate admits the submap's own errors. Rejected | Rejected |
| Cao et al., *DPCR-SLAM*, Sensors 2025 | Point-to-line ICP scan -> submap -> regional map; -10.9 % localization error, -55.8 % processing vs Cartographer | Confirms scan-to-submap (F36); a regional-map tier is the next step up | F36 done |
| Tran et al., *HP2-SLAM: Adaptive Hybrid ICP*, arXiv 2608.14996 (2026) | Per-correspondence choice between point-to-plane and point-to-point by local planarity; stable in planar and repetitive scenes | Our corridors and flat walls: try hybrid residuals in `icp_point_to_line` | Candidate |
| Gupta et al., *MapClosures*, IJRR 2026, arXiv 2501.07399 | Loop closure from density-map descriptors with self-similarity pruning against perceptual aliasing | Matches our measured false-closure problem (about half of accepted closures hurt); add a self-similarity check before accepting a closure | Candidate (F36/F37 reduced the damage) |
| Zhang et al., *Autonomous Exploration-Based Precise Mapping through Stepwise and Consistent Motions*, arXiv 2503.17005 (2026) | Drive straight, rotate in place, one goal at a time; waypoint retracing triggers loop closures | Turn in place (F39), single committed goal (F34). Retracing for closures: candidate | F34, F39 done |

| Tang et al., *Optimization of 2D laser SLAM with visual feedback loop integration*, Industrial Robot 2026 (via Consensus) | Visual bag-of-words loop detection with SURF geometric checks removes false laser loop closures in look-alike corridors | Our RGB camera is unused by SLAM: verify a laser closure visually before accepting it (about half of accepted closures increased the error) | Candidate, high value |
| Zhu et al., *Loop closure detection in hybrid scene LiDAR SLAM*, Eng. Res. Express 2026 (via Consensus) | Spatial-consistency constraints reject geometrically similar but spatially impossible loop candidates | Independent support for the drift-bounded closure gate | F36 done |
| Zhang et al., *2DLIW-SLAM*, Meas. Sci. Technol. 2024, 21 citations (via Consensus) | Line-to-line constraints plus tightly coupled LiDAR, IMU and wheel odometry handle 2D degeneracy; lower error than Cartographer | Line features from our wall scans; tighter gyro-odometry fusion | Candidate |

| Pei et al., *Loop Closure in 2D LIDAR and RGB-D SLAM*, 2022, doi 10.1145/3584376.3584531 (via fastwrite) | 2D laser grid map plus RGB-D loop closure: global image descriptors screen candidates, local descriptors pick the match, depth point-cloud matching sets the pose (coarse to fine) | Template for checking our laser closures with the Kinect RGB | Candidate, high value |
| Tong et al., *SLAM closed-loop detection method of 2D laser fusion vision in a larger space*, 2021, doi 10.1088/1742-6596/1948/1/012036 (via fastwrite) | Visual loop candidates accepted only after geometric verification with enough common points; fixes the lack of robustness of laser-only closure detection | Same | Candidate |
| Jiang et al., *TS-LCD: Two-Stage Loop-Closure Detection*, Sensors 2024, doi 10.3390/s24123702 (via fastwrite) | Laser closures fail in structurally similar places (corridors, repeated layouts); validating visual and laser loops together fixes it | Matches our measured false closures | Candidate |
| Liu et al., *Lidar-Vision Depth Fusion for Robust Loop Closure Detection*, 2025 (via fastwrite) | LiDAR plus vision closure detection cuts ATE 11.2-16.4 % against LiDAR-only | Expected size of the gain | Candidate |

## Exploration efficiency (no revisits, finish sooner)

| Paper | Finding | Use here | Status |
|---|---|---|---|
| Zhou et al., *OTO Planner: Only Travelling Once*, arXiv 2406.07294 | Enclosed sub-region detection avoids repeated paths; -10-20 % time and distance | Our room memory (F38) plus doorway skip (F41) are the same idea at room level | F38, F41 done |
| Song et al., *P2 Explore*, arXiv 2409.10878; Gao et al., *Task-oriented environment partitioning*, arXiv 2309.11107 | Room segmentation plus an optimized room visiting order (TSP) shortens paths 2-35 % | Order rooms through the doorway graph instead of nearest-first | Candidate |
| Yin et al., *PUL-SLAM*, arXiv 2511.04180 | Map-update stagnation detection cuts redundant exploration | We measure the coverage plateau; use it to rank remaining targets | Candidate |
| Wang et al., *DART*, arXiv 2503.12782 | Prioritize incompletely explored convex regions to avoid backtracking | Same principle as finishing a room before leaving (F3) | F3 done |

## Agent memory (knowledge bases)

| Paper | Finding | Use here | Status |
|---|---|---|---|
| Rasheed and Kabtoul, *eMEM: hybrid spatio-temporal memory for embodied agents*, arXiv 2606.03374 (2026) | Embodied memory must be searchable by space, time and meaning; multi-index, in-process | `agents/spatial_memory.py`: hashed by position, timestamped, typed; constant-time lookups (16x faster than the old scan at 1500 items) | F38 done |
| Lei et al., *RoboMemory*, arXiv 2508.01415; Tan et al., *RoboOS-NeXT*, arXiv 2510.26536 | Unified spatial, temporal, episodic memory with a dynamic spatial knowledge graph | Supervisor keeps the knowledge graph and timeline; doorways now by place with real rooms | F38 done |

## Perception (object detection, F28)

| Paper | Finding | Use here | Status |
|---|---|---|---|
| Liu et al., *YOLO-UniOW*, arXiv 2412.20645 | Open-world detection, 34.6 AP on LVIS at 69.6 FPS (GPU), "unknown" objects handled | Candidate detector for the RGB camera; check CPU speed and licence first | Candidate |
| He et al., *DOSOD*, arXiv 2412.14680 | Lighter YOLO-World variant, +57 % FPS at similar accuracy, edge deployable | Same | Candidate |
| Basit and Shafique, *Energy-aware open-vocabulary detection scheduling*, arXiv 2611.07663 (2026) | Run the detector only when odometry or map change makes a frame informative; -34 % detector calls at equal success | Run detection on new viewpoints only, keeping the loop real time on CPU | Candidate |

## Locomotion (smooth, faster gait)

| Paper | Finding | Use here | Status |
|---|---|---|---|
| Bellegarda et al., *AllGaits*, arXiv 2411.04787 | A CPG of coupled oscillators whose couplings select the gait; smooth transitions at any speed | Our gait is fixed sinusoids; measured body surge (18 % of samples moving backward within a stride) is a stance-phase problem a coupled-oscillator trot addresses | Candidate (gait redesign, then odometry recalibration) |
| Liu et al., *An Improved Method Based on CPG to Achieve Trotting Gait of Quadrupedal Robot*, 2019 (via fastwrite) | Dropping the assumption that only the upper limb moves during stance (lower limb fixed) greatly reduces torso pitch and roll | First thing to check in our `apply_gait` for the measured forward-back surge | Candidate, high value |
| Liu et al., *A Trainable CPG Framework for Quadruped Locomotion*, arXiv 2610.36537 (2026) | Symmetric 8-neuron CPG plus a learned joint mapping tracks velocity and yaw rate across gaits | Longer-term option; needs training | Later |

## Tried and rejected after replay

| Idea | Result |
|---|---|
| Temporal-consistency check for loop closures (second agreeing closure within 5 s, 8 cm), motivated by run 8's 69 cm spike: one unconfirmed 18 cm closure against a keyframe 57 m old | Inconsistent across routes: from another keyframe run 2 24.8 -> 9.8 cm but run 3 17.8 -> 44.7 cm (closures 65 -> 10, good ones rejected too); any second closure 24.8 -> 20.0 and 17.8 -> 29.5 cm. Not adopted; camera verification of closures remains the candidate |
| KISS-ICP-style adaptive odometry gate | Worse on all 5 recordings (see Localization) |
| Soft pull of submap corrections | Worse on 3 of 4 recordings |
