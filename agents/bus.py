"""Asynchronous message bus and strongly typed telemetry contracts."""
from dataclasses import dataclass, field
from typing import List, Tuple, Dict, Any, Optional, Callable
import time
import threading
import numpy as np


@dataclass
class PoseMsg:
    timestamp: float
    x: float
    y: float
    yaw: float
    is_ground_truth: bool = False
    covariance: float = 0.0


@dataclass
class ScanMsg:
    timestamp: float
    points_2d: np.ndarray  # (N, 2) in robot base or head frame
    world_points_3d: Optional[np.ndarray] = None


@dataclass
class RGBDMsg:
    timestamp: float
    rgb_image: Optional[np.ndarray] = None  # (H, W, 3) uint8
    depth_matrix: Optional[np.ndarray] = None  # (H, W) float32 in meters


@dataclass
class GridMapMsg:
    timestamp: float
    log_odds: np.ndarray
    free_cells: int
    occupied_cells: int
    unknown_cells: int
    explored_pct: float
    room_labels: Optional[np.ndarray] = None  # 2D array of room IDs
    doorways: List[Tuple[float, float]] = field(default_factory=list)


@dataclass
class FrontierTargetMsg:
    timestamp: float
    target_gx: int
    target_gy: int
    target_wx: float
    target_wy: float
    room_id: int
    area: int
    is_doorway: bool = False


@dataclass
class PathMsg:
    timestamp: float
    waypoints: List[Tuple[float, float]]  # List of (x, y) world coordinates
    active_idx: int = 0
    replan_count: int = 0


@dataclass
class ActuationMsg:
    timestamp: float
    joint_targets: Dict[str, float]
    heading_error: float
    speed: float
    state_str: str


@dataclass
class MissionEventMsg:
    timestamp: float
    event_type: str
    details: Dict[str, Any] = field(default_factory=dict)


class MessageBus:
    """Thread-safe, non-blocking asynchronous event bus for multi-agent coordination."""

    def __init__(self):
        self._lock = threading.Lock()
        self._subscribers: Dict[str, List[Callable[[Any], None]]] = {}
        self._latest_state: Dict[str, Any] = {}

    def subscribe(self, topic: str, callback: Callable[[Any], None]):
        with self._lock:
            if topic not in self._subscribers:
                self._subscribers[topic] = []
            self._subscribers[topic].append(callback)

    def publish(self, topic: str, message: Any):
        with self._lock:
            self._latest_state[topic] = message
            callbacks = list(self._subscribers.get(topic, []))
        
        for cb in callbacks:
            try:
                cb(message)
            except Exception as e:
                print(f"[MessageBus ERROR] Callback for topic '{topic}' raised: {e}")

    def get_latest(self, topic: str, default: Any = None) -> Any:
        with self._lock:
            return self._latest_state.get(topic, default)

    def clear(self):
        with self._lock:
            self._subscribers.clear()
            self._latest_state.clear()
