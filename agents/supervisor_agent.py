"""Supervisor Agent: System watchdog, Spatial Knowledge Graph custodian, and Episodic Memory manager."""
from typing import Optional, Dict, Any
from .base import BaseAgent
from .bus import MessageBus, MissionEventMsg, PoseMsg, GridMapMsg, PathMsg
from .knowledge_graph import SpatialKnowledgeGraph
from .episodic_memory import EpisodicMemory


class SupervisorAgent(BaseAgent):
    def __init__(self, bus: MessageBus, update_rate_hz: float = 2.0):
        super().__init__("SupervisorAgent", bus, update_rate_hz)
        self.knowledge_graph = SpatialKnowledgeGraph()
        self.episodic_memory = EpisodicMemory()
        self.latest_pose: Optional[PoseMsg] = None
        self.latest_grid: Optional[GridMapMsg] = None
        self.mission_status: str = "ACTIVE"

    def initialize(self) -> bool:
        self.bus.subscribe("mission/event", self.on_mission_event)
        self.bus.subscribe("pose/state", self.on_pose)
        self.bus.subscribe("map/grid", self.on_grid)
        self.is_running = True
        return True

    def on_pose(self, pose_msg: PoseMsg):
        self.latest_pose = pose_msg

    def on_grid(self, grid_msg: GridMapMsg):
        self.latest_grid = grid_msg

    def on_mission_event(self, event_msg: MissionEventMsg):
        rx = self.latest_pose.x if self.latest_pose else 0.0
        ry = self.latest_pose.y if self.latest_pose else 0.0
        
        # Log to episodic memory
        self.episodic_memory.record_event(
            timestamp=event_msg.timestamp,
            event_type=event_msg.event_type,
            description=f"{event_msg.event_type}: {event_msg.details}",
            coordinates={"x": rx, "y": ry},
            **event_msg.details
        )

        # Update knowledge graph
        if event_msg.event_type == "ROOM_ENTERED":
            room_id = event_msg.details.get("room_id", 1)
            room = self.knowledge_graph.get_or_create_room(room_id)
            room.status = "EXPLORING"
        elif event_msg.event_type == "ROOM_COMPLETED":
            room_id = event_msg.details.get("room_id", 1)
            room = self.knowledge_graph.get_or_create_room(room_id)
            room.status = "EXPLORED"
        elif event_msg.event_type == "DOORWAY_DISCOVERED":
            d_id = event_msg.details.get("door_id", "door_1")
            pos = event_msg.details.get("pos", (rx, ry))
            r_a = event_msg.details.get("room_a", 1)
            r_b = event_msg.details.get("room_b", 2)
            self.knowledge_graph.add_doorway(d_id, pos, r_a, r_b)

    def step(self, current_time: float) -> None:
        if self.latest_grid:
            # Register newly discovered doorways in Knowledge Graph
            for idx, dw in enumerate(self.latest_grid.doorways):
                d_id = f"doorway_{idx+1}"
                if d_id not in self.knowledge_graph.doorways:
                    self.knowledge_graph.add_doorway(d_id, dw, 1, 2)
                    self.episodic_memory.record_event(
                        timestamp=current_time,
                        event_type="DOORWAY_DETECTED",
                        description=f"Topological doorway bottleneck identified at ({dw[0]:.2f}, {dw[1]:.2f})",
                        coordinates={"x": dw[0], "y": dw[1]}
                    )

    def export_mission_report(self) -> Dict[str, Any]:
        return {
            "knowledge_graph": self.knowledge_graph.export_json(),
            "timeline_markdown": self.episodic_memory.generate_timeline_markdown()
        }
