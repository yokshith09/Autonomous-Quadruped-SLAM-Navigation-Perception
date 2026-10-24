"""Per-run measurement: agent latency/failures, tracking availability, resource use, standard SLAM metrics,
and a self-contained run directory (manifest.json, metrics.json, agents.csv, trajectory.csv).

Everything printed or saved is computed from the current run. A metric that cannot be measured is reported
as {"value": None, "reason": "..."} and printed as "n/a (reason)", never as a plausible default.

Overhead: per-step work is two perf_counter() calls and a list append per agent; the resource sampler is a
daemon thread reading psutil once per second; all file I/O happens after the control loop has exited.
"""
import csv
import json
import math
import os
import platform
import subprocess
import sys
import threading
import time
from collections import Counter

import numpy as np

SAMPLE_CAP = 2_000_000          # per-series cap: bounds memory on very long runs; overflow is counted, not hidden
TRACK_LOSS_MIN_UPDATES = 10     # >= 10 consecutive rejected ICP updates (~1 s at 10 Hz) = one tracking-loss episode
RPE_DELTA_M = 1.0               # RPE evaluated over GT path segments of this length (KITTI-style, distance-based)


def na(reason):
    return {"value": None, "reason": reason}


def wrap(a):
    return (np.asarray(a) + np.pi) % (2.0 * np.pi) - np.pi


def pctl(samples, q):
    return float(np.percentile(samples, q)) if len(samples) else None


# ----------------------------------------------------------------------------------------------- formulas
def path_length(xy):
    xy = np.asarray(xy, dtype=np.float64)
    return float(np.sum(np.hypot(*np.diff(xy, axis=0).T))) if len(xy) > 1 else 0.0


def rpe_distance(gt, est, delta_m=RPE_DELTA_M):
    """Relative pose error over GT path segments of length delta_m.

    For each start index i, j is the first index whose GT path length from i reaches delta_m. The relative
    motion i->j is expressed in the body frame at i for both trajectories (so a constant global offset does
    not count as error), and the translation / rotation differences are collected.
    Returns (trans_rmse_m, trans_pct_of_delta, rot_rmse_deg_per_m, n_pairs); None values if no pair exists.
    """
    gt, est = np.asarray(gt, float), np.asarray(est, float)
    cum = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(gt[:, :2], axis=0).T))]) if len(gt) > 1 else np.zeros(1)
    j = np.searchsorted(cum, cum + delta_m)
    i = np.nonzero(j < len(gt))[0]
    if len(i) == 0:
        return None, None, None, 0
    j = j[i]

    def rel(p):
        d = p[j, :2] - p[i, :2]
        c, s = np.cos(p[i, 2]), np.sin(p[i, 2])
        return np.column_stack([c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1]]), wrap(p[j, 2] - p[i, 2])

    (tg, rg), (te, re) = rel(gt), rel(est)
    seg = cum[j] - cum[i]
    et = np.hypot(*(te - tg).T)
    er = np.degrees(np.abs(wrap(re - rg))) / seg
    t_rmse = float(np.sqrt(np.mean(et ** 2)))
    return t_rmse, float(np.mean(et / seg) * 100.0), float(np.sqrt(np.mean(er ** 2))), int(len(i))


def loss_episodes(accepted, times, min_len=TRACK_LOSS_MIN_UPDATES):
    """Runs of >= min_len consecutive rejected tracking updates -> [(t_start, t_end, n_updates)]."""
    eps, start = [], None
    for k, ok in enumerate(list(accepted) + [True]):
        if not ok and start is None:
            start = k
        elif ok and start is not None:
            if k - start >= min_len:
                eps.append((float(times[start]), float(times[k - 1]), k - start))
            start = None
    return eps


OVERHEAD_BASIS = ("calibrated estimate: micro-benchmarked cost of the per-agent timing wrapper x calls per step, plus measured "
                  "tracking bookkeeping; recording the scan stream is not included and no uninstrumented A/B run was made")


def overhead_per_step_ms(total_calls, steps, n=50_000):
    """Estimated telemetry cost per control step: wrapper cost (timed here against a no-op agent) x calls / steps."""
    class _A:
        name = "x"

        def step(self, t):
            pass

    a, lat, calls = _A(), [], Counter()
    t0 = time.perf_counter()
    for _ in range(n):
        a.step(0.0)
    base = time.perf_counter() - t0
    t0 = time.perf_counter()
    for _ in range(n):
        s = time.perf_counter()
        a.step(0.0)
        lat.append((time.perf_counter() - s) * 1000.0)
        calls["x"] += 1
    per_call_ms = max(0.0, (time.perf_counter() - t0) - base) * 1000.0 / n
    return per_call_ms * total_calls / max(1, steps)


def extra_warnings(overall, loc):
    """Data-quality warnings derived from the computed metrics (shared by the live summary and the report)."""
    w, end = [], str(overall.get("end_reason", ""))
    if end.startswith(("timeout", "max steps")):
        w.append(f"run ended by a limit ({end}), not by mission completion: coverage, rooms completed and task success are lower bounds")
    elif overall.get("task_success") is False:
        w.append(f"mission did not report completion ({end})")
    acc = loc.get("icp_acceptance_pct")
    if isinstance(acc, float) and acc < 50.0:
        w.append(f"scan matching passed its acceptance gates on only {acc:.0f} % of updates; rejection reasons are not recorded "
                 f"(and the flag is not refreshed for scans with <20 points), so this is a diagnostic indicator, not a failure rate")
    return w


def first_crossing(times, err, thr):
    idx = np.nonzero(np.asarray(err) > thr)[0]
    return float(times[idx[0]]) if len(idx) else None


# ----------------------------------------------------------------------------------------------- recorder
class RunTelemetry:
    def __init__(self, out_root="runs", argv=None):
        self.run_id = time.strftime("%Y%m%d_%H%M%S")
        self.dir = os.path.join(out_root, self.run_id)
        os.makedirs(self.dir, exist_ok=True)
        self.argv = list(argv if argv is not None else sys.argv)
        self.wall_start = time.time()
        self.lat = {}                 # agent name -> [ms]
        self.calls = Counter()
        self.err0 = {}
        self.dropped = Counter()
        self.events = Counter()
        self.event_log = []
        self.track_ok, self.track_t, self.scan_pts = [], [], []
        self.loop_ms = []
        self.res = []                 # (wall_s, cpu_pct, rss_mb)
        self._stop = threading.Event()
        self._sampler = None
        self.overhead_s = 0.0

    # --- live hooks (cheap, never raise into the control loop)
    def attach(self, bus, agents):
        for a in agents:
            self.err0[a.name] = a.error_count
            self.lat[a.name] = []
        bus.subscribe("mission/event", self._on_event)
        try:
            import psutil
            self._sampler = threading.Thread(target=self._sample, args=(psutil.Process(),), daemon=True)
            self._sampler.start()
        except Exception as e:  # psutil missing -> resource metrics reported unavailable
            self._sampler_err = str(e)

    def _on_event(self, msg):
        self.events[msg.event_type] += 1
        if len(self.event_log) < 10_000:
            self.event_log.append((round(float(msg.timestamp), 2), msg.event_type))

    def _sample(self, proc):
        proc.cpu_percent(None)
        while not self._stop.wait(1.0):
            try:
                self.res.append((time.time() - self.wall_start, proc.cpu_percent(None), proc.memory_info().rss / 2 ** 20))
            except Exception:
                pass

    def _push(self, series, key, v):
        if len(series) < SAMPLE_CAP:
            series.append(v)
        else:
            self.dropped[key] += 1

    def step(self, agent, t):
        """Run agent.step(t) and time it. Exceptions propagate unchanged (same behaviour as an untimed call)."""
        s = time.perf_counter()
        agent.step(t)
        self._push(self.lat[agent.name], agent.name, (time.perf_counter() - s) * 1000.0)
        self.calls[agent.name] += 1

    def slam_tracked(self, slam_agent, n_before, t):
        """Record one tracking outcome if SLAM actually ran an update this step."""
        s = time.perf_counter()
        if len(slam_agent.track_times_ms) > n_before:
            self._push(self.track_ok, "track", bool(getattr(slam_agent.pose_estimator.pose, "accepted", False)))
            self.track_t.append(t)
            scan = slam_agent.latest_scan
            self.scan_pts.append(len(scan.points_2d) if scan is not None else 0)
        self.overhead_s += time.perf_counter() - s

    def loop_time(self, ms):
        self._push(self.loop_ms, "loop", ms)

    # --- end of run
    def finalize(self, *, agents, gt, est, times, grid_metrics, ctx):
        """Compute all metrics, write the run directory, print the summary. Returns the metrics dict."""
        self._stop.set()
        try:
            m = self._compute(agents, gt, est, times, grid_metrics, ctx)
        except Exception as e:
            print(f"  [TELEMETRY] metric computation failed: {e}", flush=True)
            m = {"error": str(e)}
        try:
            self._write(m, agents, gt, est, times, ctx)
        except Exception as e:
            print(f"  [TELEMETRY] writing run files failed: {e}", flush=True)
        try:
            print_summary(m)
        except Exception as e:
            print(f"  [TELEMETRY] summary print failed: {e}", flush=True)
        return m

    def _compute(self, agents, gt, est, times, gm, ctx):
        gt, est, times = np.asarray(gt, float), np.asarray(est, float), np.asarray(times, float)
        n = min(len(gt), len(est), len(times))
        gt, est, times = gt[:n], est[:n], times[:n]
        wall = time.time() - self.wall_start
        sim_t = float(times[-1]) if n else 0.0
        dist = path_length(gt[:, :2]) if n else 0.0
        warnings = []

        # ---- overall
        ev = dict(self.events)
        overall = {
            "end_reason": ctx["end_reason"],
            "task_success": ctx["end_reason"] == "all reachable frontiers explored",
            "sim_time_s": sim_t,
            "wall_time_s": wall,
            "real_time_factor": sim_t / wall if wall > 0 else None,
            "distance_travelled_gt_m": dist,
            "rooms_discovered": ctx.get("rooms_discovered"),
            "rooms_completed": ctx.get("rooms_completed"),
            "doorways_detected": ctx.get("doorways_detected", na("not supplied by the run")),
            "explored_free_area_m2": ctx.get("free_area_m2"),
            "explored_pct_of_bounding_box": ctx.get("explored_pct"),
            "stuck_events": ev.get("ROBOT_STUCK", 0),
            "unreachable_targets": ev.get("PATH_UNREACHABLE", 0),
            "human_interventions": na("no intervention channel exists in the stack; only the 'q' key / external stop, recorded in end_reason"),
            "collisions": na("CoppeliaSim contact sensing is not wired into the stack"),
        }

        # ---- agents
        agent_rows = []
        for a in agents:
            lat = self.lat.get(a.name, [])
            errs = a.error_count - self.err0.get(a.name, 0)
            calls = self.calls.get(a.name, 0)
            agent_rows.append({
                "agent": a.name, "rate_hz": a.update_rate_hz, "calls": calls, "errors": errs,
                "ok": calls - errs,
                "p50_ms": pctl(lat, 50), "p95_ms": pctl(lat, 95), "max_ms": float(max(lat)) if lat else None,
                "total_s": float(np.sum(lat)) / 1000.0 if lat else 0.0,
                "task_metric": ctx.get("agent_task", {}).get(a.name, "-"),
            })
        loop = self.loop_ms
        tracked = np.array(self.track_ok, bool)
        n_track = len(tracked)
        agent_rows_extra = {
            "control_loop_p50_ms": pctl(loop, 50), "control_loop_p95_ms": pctl(loop, 95),
            "control_loop_max_ms": float(max(loop)) if loop else None,
            "component_contribution": na("not measurable from a single live run; run ablate.py on this run's recording"),
        }

        # ---- localization
        loc = {}
        if n > 1:
            err = np.hypot(*(est[:, :2] - gt[:, :2]).T)
            yaw_err = np.degrees(np.abs(wrap(est[:, 2] - gt[:, 2])))
            t_rmse, t_pct, r_rmse, n_pairs = rpe_distance(gt, est)
            eps = loss_episodes(tracked, self.track_t)
            reloc = [e for e in eps if any(t > e[1] and t - e[1] <= 30.0 for t, k in self.event_log
                                           if k == "LOOP_CLOSURE_DETECTED")]
            final_err = float(err[-1])
            loc = {
                "ate_rmse_m": float(np.sqrt(np.mean(err ** 2))), "ate_mean_m": float(err.mean()),
                "ate_median_m": float(np.median(err)), "ate_max_m": float(err.max()),
                "final_position_error_m": final_err,
                "final_drift_pct_of_distance": final_err / dist * 100.0 if dist > 0 else None,
                "rpe_trans_rmse_m_per_1m": t_rmse, "rpe_trans_pct": t_pct,
                "rpe_rot_rmse_deg_per_m": r_rmse, "rpe_pairs": n_pairs,
                "heading_err_mean_deg": float(yaw_err.mean()), "heading_err_max_deg": float(yaw_err.max()),
                "time_error_first_exceeds_0p5m_s": first_crossing(times, err, 0.5),
                "time_error_first_exceeds_1m_s": first_crossing(times, err, 1.0),
                "tracking_updates": n_track,
                "icp_acceptance_pct": float(tracked.mean() * 100.0) if n_track else None,
                "tracking_loss_episodes": len(eps),
                "tracking_loss_total_s": float(sum(e[1] - e[0] for e in eps)),
                "relocalized_by_loop_closure_within_30s": len(reloc),
                "loop_closures": ev.get("LOOP_CLOSURE_DETECTED", 0),
                "odometry_distance_m": ctx.get("odom_distance_m"),
                "odometry_scale_error_pct": ((ctx["odom_distance_m"] - dist) / dist * 100.0)
                if dist > 0 and ctx.get("odom_distance_m") is not None else None,
            }
            if n_track == 0:
                warnings.append("no SLAM tracking updates were observed")
        else:
            loc = {"ate_rmse_m": na("fewer than 2 pose samples")}
            warnings.append("trajectory too short for localization metrics")

        # ---- mapping
        if gm:
            mapping = {
                "explored_free_area_m2": ctx.get("free_area_m2"),
                "occupied_cells": gm.get("total_slam_obstacle_cells"),
                "reference_occupied_cells": gm.get("total_true_obstacle_cells"),
                "wall_accuracy_m": gm.get("accuracy_meters"), "wall_completion_m": gm.get("completion_meters"),
                "completion_ratio_5cm_pct": gm.get("completion_ratio_5cm", 0) * 100.0,
                "chamfer_m": gm.get("chamfer_distance_meters"), "hausdorff95_m": gm.get("hausdorff_95_meters"),
                "precision_pct": gm.get("precision", 0) * 100.0, "recall_pct": gm.get("recall", 0) * 100.0,
                "f1_pct": gm.get("f1_score", 0) * 100.0, "tolerance_m": gm.get("tolerance_margin_meters"),
                "free_space_iou_pct": gm.get("free_space_iou", 0) * 100.0,
                "map_update_p50_ms": pctl(self.lat.get("MappingAgent", []), 50),
                "map_update_p95_ms": pctl(self.lat.get("MappingAgent", []), 95),
                "map_memory_mb": ctx.get("map_memory_mb"),
                "frontiers_remaining": ctx.get("frontiers_remaining"),
            }
        else:
            mapping = {"wall_accuracy_m": na("grid evaluation did not run")}
            warnings.append("map evaluation unavailable")

        # ---- compute
        res = np.array(self.res) if self.res else np.empty((0, 3))
        gpu = gpu_info()
        compute = {
            "cpu": ctx.get("cpu_name"), "logical_cores": os.cpu_count(),
            "process_cpu_pct_mean": float(res[:, 1].mean()) if len(res) else na("psutil sampler did not run"),
            "process_cpu_pct_p95": float(np.percentile(res[:, 1], 95)) if len(res) else None,
            "rss_mb_mean": float(res[:, 2].mean()) if len(res) else None,
            "rss_mb_peak": float(res[:, 2].max()) if len(res) else None,
            "resource_samples": int(len(res)),
            "gpu": gpu or "none detected",
            "gpu_utilization": na("no code path uses the GPU (NumPy/SciPy/OpenCV on CPU); nothing to attribute"),
            "slam_tracking_p50_ms": pctl(self.lat.get("SLAMAgent", []), 50),
            "slam_tracking_p95_ms": pctl(self.lat.get("SLAMAgent", []), 95),
            "bottleneck_agent": max(agent_rows, key=lambda r: r["total_s"])["agent"] if agent_rows else None,
            "telemetry_overhead_ms_per_step": overhead_per_step_ms(sum(self.calls.values()), n) + self.overhead_s * 1000.0 / max(1, n),
            "telemetry_overhead_basis": OVERHEAD_BASIS,
            "note": "process CPU % is per-process and may exceed 100 on multi-core; CoppeliaSim runs in its own process and is not included",
        }

        # ---- data quality
        if any(self.dropped.values()):
            warnings.append(f"samples dropped at cap {SAMPLE_CAP}: {dict(self.dropped)} (percentiles cover the kept part only)")
        if n and np.any(np.diff(times) <= 0):
            warnings.append(f"{int(np.sum(np.diff(times) <= 0))} non-increasing timestamps in trajectory")
        if not ctx.get("closed_loop_slam", True):
            warnings.append("run used --ground-truth-nav: navigation did not depend on SLAM")
        if sim_t < 120:
            warnings.append("run shorter than 2 min: metrics are not representative")
        warnings += extra_warnings(overall, loc)
        limitations = [
            "Single run: no repeatability or variance claims.",
            "Ground truth = CoppeliaSim body pose (exact, same timestamps; no alignment applied because SLAM starts at the GT start pose).",
            "Map reference = occupancy grid built from the same depth stream using the GT pose: it isolates SLAM pose error, "
            "but is limited by what the sensor saw; it is not an independent CAD floor plan.",
            "Simulation only: sensor noise, gait slip and lighting are simulated, not measured on hardware.",
        ]
        return {"run_id": self.run_id, "overall": overall, "agents": agent_rows, "agents_extra": agent_rows_extra,
                "localization": loc, "mapping": mapping, "compute": compute,
                "data_quality": {"warnings": warnings, "limitations": limitations, "events": ev,
                                 "dropped_samples": dict(self.dropped)}}

    def _write(self, m, agents, gt, est, times, ctx):
        with open(os.path.join(self.dir, "metrics.json"), "w", encoding="utf-8") as f:
            json.dump(m, f, indent=2, default=str)
        if m.get("agents"):
            with open(os.path.join(self.dir, "agents.csv"), "w", newline="", encoding="utf-8") as f:
                w = csv.DictWriter(f, fieldnames=list(m["agents"][0].keys()))
                w.writeheader()
                w.writerows(m["agents"])
        n = min(len(gt), len(est), len(times))
        with open(os.path.join(self.dir, "trajectory.csv"), "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["t_s", "gt_x_m", "gt_y_m", "gt_yaw_rad", "est_x_m", "est_y_m", "est_yaw_rad"])
            for k in range(n):
                w.writerow([f"{times[k]:.3f}", *(f"{v:.4f}" for v in gt[k]), *(f"{v:.4f}" for v in est[k])])
        with open(os.path.join(self.dir, "manifest.json"), "w", encoding="utf-8") as f:
            json.dump({
                "run_id": self.run_id, "started": time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(self.wall_start)),
                "command": " ".join(self.argv), "git": git_info(), "python": sys.version.split()[0],
                "platform": platform.platform(), "cpu": ctx.get("cpu_name"), "gpu": gpu_info(),
                "config": ctx.get("config", {}),
                "ground_truth": "CoppeliaSim /Base pose every control step (used for scoring only)",
                "sources": {"recording": ctx.get("recording"), "trajectory": "trajectory.csv",
                            "grids": ["captured_matrices/true_grid.npy", "captured_matrices/slam_grid.npy"]},
                "metric_definitions": METRIC_DEFS,
            }, f, indent=2, default=str)
        print(f"  [SAVED] Run report -> {self.dir}/ (manifest.json, metrics.json, agents.csv, trajectory.csv)", flush=True)


METRIC_DEFS = {
    "frame": "2D world frame of the CoppeliaSim scene (x, y in m, yaw in rad, CCW); SLAM initialised at the GT start pose, no trajectory alignment",
    "ate": "per-step Euclidean position error |p_est - p_gt| over every control step; RMSE/mean/median/max",
    "final_drift_pct": "final position error / GT path length * 100",
    "rpe": f"distance-based relative pose error over {RPE_DELTA_M} m GT segments, relative motion in the start body frame; trans RMSE (m), mean trans error as % of segment, rot RMSE (deg/m)",
    "icp_acceptance": "share of SLAM updates whose scan match passed the acceptance gates (else odometry-only)",
    "tracking_loss_episode": f">= {TRACK_LOSS_MIN_UPDATES} consecutive rejected SLAM updates",
    "relocalized": "tracking-loss episode followed by an accepted loop closure within 30 s",
    "wall_accuracy/completion": "mean distance SLAM->reference occupied cells / reference->SLAM (m)",
    "precision/recall/f1": "occupied-cell agreement with reference grid within tolerance_m",
    "free_space_iou": "IoU of free cells between SLAM grid and reference grid",
    "latency": "wall-clock time of each agent.step() call (ms), p50/p95/max over all calls",
    "real_time_factor": "simulated seconds per wall-clock second",
}


def git_info():
    try:
        sha = subprocess.run(["git", "rev-parse", "--short", "HEAD"], capture_output=True, text=True, timeout=5).stdout.strip()
        dirty = bool(subprocess.run(["git", "status", "--porcelain", "--untracked-files=no"],
                                    capture_output=True, text=True, timeout=5).stdout.strip())
        return {"commit": sha, "uncommitted_changes": dirty}
    except Exception as e:
        return {"commit": None, "reason": str(e)}


def gpu_info():
    try:
        out = subprocess.run(["nvidia-smi", "--query-gpu=name,memory.total", "--format=csv,noheader"],
                             capture_output=True, text=True, timeout=5).stdout.strip()
        return out or None
    except Exception:
        return None


# ----------------------------------------------------------------------------------------------- printing
def f(v, d=1, unit=""):
    if isinstance(v, dict) and "reason" in v:
        return f"n/a ({v['reason']})"
    if v is None:
        return "n/a"
    if isinstance(v, bool):
        return "yes" if v else "no"
    if isinstance(v, (int, np.integer)):
        return f"{v}{unit}"
    if isinstance(v, float):
        return f"{v:.{d}f}{unit}" if math.isfinite(v) else "n/a"
    return f"{v}{unit}"


def print_summary(m):
    W = 78
    line = "=" * W
    o, loc, mp, c, q = m.get("overall", {}), m.get("localization", {}), m.get("mapping", {}), m.get("compute", {}), m.get("data_quality", {})
    print("\n" + line)
    print(f"  RUN SUMMARY  {m.get('run_id')}   (all values measured in this run)")
    print(line)
    print("  1. Overall robot performance")
    print(f"     Outcome: {o.get('end_reason')}  | task success: {f(o.get('task_success'))}")
    print(f"     Sim time {f(o.get('sim_time_s'), 0, ' s')} | wall {f(o.get('wall_time_s'), 0, ' s')} | real-time factor {f(o.get('real_time_factor'), 2)}")
    print(f"     Distance (GT) {f(o.get('distance_travelled_gt_m'), 1, ' m')} | rooms completed {f(o.get('rooms_completed'))}/{f(o.get('rooms_discovered'))} discovered"
          f" | doorways {f(o.get('doorways_detected'))}")
    print(f"     Explored free area {f(o.get('explored_free_area_m2'), 1, ' m2')} | stuck events {f(o.get('stuck_events'))}"
          f" | unreachable targets {f(o.get('unreachable_targets'))}")
    print(f"     Interventions: {f(o.get('human_interventions'))}")
    print("  2. Agents (latency = wall time of one step() call)")
    print(f"     {'agent':16s}{'Hz':>5s}{'calls':>8s}{'errors':>7s}{'p50 ms':>8s}{'p95 ms':>8s}{'max ms':>8s}{'total s':>9s}  task metric")
    for r in m.get("agents", []):
        print(f"     {r['agent']:16s}{r['rate_hz']:5.0f}{r['calls']:8d}{r['errors']:7d}{f(r['p50_ms'], 2):>8s}{f(r['p95_ms'], 2):>8s}"
              f"{f(r['max_ms'], 1):>8s}{f(r['total_s'], 1):>9s}  {r['task_metric']}")
    ax = m.get("agents_extra", {})
    print(f"     Control-loop step p50/p95/max: {f(ax.get('control_loop_p50_ms'), 1)}/{f(ax.get('control_loop_p95_ms'), 1)}/{f(ax.get('control_loop_max_ms'), 0)} ms")
    print(f"     Contribution to system result: {f(ax.get('component_contribution'))}")
    print("  3. Localization (vs simulator ground truth)")
    print(f"     ATE RMSE {f(_cm(loc.get('ate_rmse_m')), 1, ' cm')}"
          f" | median {f(_cm(loc.get('ate_median_m')), 1, ' cm')} | max {f(_cm(loc.get('ate_max_m')), 1, ' cm')}")
    print(f"     Final drift {f(_cm(loc.get('final_position_error_m')), 1, ' cm')} = {f(loc.get('final_drift_pct_of_distance'), 2, ' %')} of path"
          f" | heading err mean {f(loc.get('heading_err_mean_deg'), 2, ' deg')}")
    print(f"     RPE (1 m): trans {f(_cm(loc.get('rpe_trans_rmse_m_per_1m')), 1, ' cm')} RMSE ({f(loc.get('rpe_trans_pct'), 1, ' %')})"
          f" | rot {f(loc.get('rpe_rot_rmse_deg_per_m'), 2, ' deg/m')} | {f(loc.get('rpe_pairs'))} pairs")
    print(f"     Scan-match acceptance {f(loc.get('icp_acceptance_pct'), 1, ' %')} of {f(loc.get('tracking_updates'))} updates"
          f" | loss episodes {f(loc.get('tracking_loss_episodes'))} ({f(loc.get('tracking_loss_total_s'), 1, ' s')})"
          f" | relocalized {f(loc.get('relocalized_by_loop_closure_within_30s'))} | loop closures {f(loc.get('loop_closures'))}")
    print(f"     Error first > 0.5 m at {f(loc.get('time_error_first_exceeds_0p5m_s'), 0, ' s')}, > 1 m at {f(loc.get('time_error_first_exceeds_1m_s'), 0, ' s')}"
          f" | odometry scale error {f(loc.get('odometry_scale_error_pct'), 1, ' %')}")
    print("  4. Mapping (vs reference grid built with GT pose)")
    print(f"     Wall accuracy {f(_cm(mp.get('wall_accuracy_m')), 1, ' cm')} | completion {f(_cm(mp.get('wall_completion_m')), 1, ' cm')}"
          f" | chamfer {f(_cm(mp.get('chamfer_m')), 1, ' cm')} | completion ratio(5 cm) {f(mp.get('completion_ratio_5cm_pct'), 1, ' %')}")
    print(f"     Precision {f(mp.get('precision_pct'), 1, ' %')} | recall {f(mp.get('recall_pct'), 1, ' %')} | F1 {f(mp.get('f1_pct'), 1, ' %')}"
          f" (tol {f(_cm(mp.get('tolerance_m')), 0, ' cm')}) | free-space IoU {f(mp.get('free_space_iou_pct'), 1, ' %')}")
    print(f"     Coverage {f(mp.get('explored_free_area_m2'), 1, ' m2')} free | map update p50/p95 {f(mp.get('map_update_p50_ms'), 1)}/{f(mp.get('map_update_p95_ms'), 1)} ms"
          f" | map {f(mp.get('map_memory_mb'), 2, ' MB')} | frontiers left {f(mp.get('frontiers_remaining'))}")
    print("  5. Compute")
    print(f"     {c.get('cpu')} ({c.get('logical_cores')} threads) | GPU: {c.get('gpu')} - {f(c.get('gpu_utilization'))}")
    print(f"     Process CPU mean/p95 {f(c.get('process_cpu_pct_mean'), 0, ' %')}/{f(c.get('process_cpu_pct_p95'), 0, ' %')}"
          f" | RSS mean/peak {f(c.get('rss_mb_mean'), 0)}/{f(c.get('rss_mb_peak'), 0, ' MB')} | bottleneck: {c.get('bottleneck_agent')}"
          f" | telemetry overhead {f(c.get('telemetry_overhead_ms_per_step'), 3, ' ms/step')}")
    print("  6. Data quality and limitations")
    for w in q.get("warnings", []) or ["no data-quality warnings"]:
        print(f"     ! {w}")
    for lim in q.get("limitations", []):
        print(f"     - {lim}")
    print(line, flush=True)


def _cm(v):
    return v * 100.0 if isinstance(v, (int, float)) and not isinstance(v, bool) else v
