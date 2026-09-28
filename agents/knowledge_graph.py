"""Spatial-Temporal Semantic Knowledge Graph (3D Scene Graph)."""
from dataclasses import dataclass, field
from typing import Dict, List, Tuple, Any, Optional
import time
import json


@dataclass
class EntityNode:
    entity_id: str
    class_name: str
    world_pos: Tuple[float, float, float]
    room_id: int
    confidence: float = 1.0
    first_seen: float = 0.0
    last_seen: float = 0.0
    attributes: Dict[str, Any] = field(default_factory=dict)


@dataclass
class DoorwayNode:
    door_id: str
    world_pos: Tuple[float, float]
    connects_rooms: Tuple[int, int]
    width_m: float = 0.8
    is_traversed: bool = False


@dataclass
class RoomNode:
    room_id: int
    name: str
    status: str = "UNEXPLORED"  # UNEXPLORED | EXPLORING | EXPLORED
    centroid: Tuple[float, float] = (0.0, 0.0)
    entities: List[str] = field(default_factory=list)
    doorways: List[str] = field(default_factory=list)


class SpatialKnowledgeGraph:
    """4-Layer Hierarchical Spatial-Semantic Scene Graph."""

    def __init__(self):
        self.rooms: Dict[int, RoomNode] = {}
        self.doorways: Dict[str, DoorwayNode] = {}
        self.entities: Dict[str, EntityNode] = {}
        self.edges: List[Dict[str, Any]] = []

    def get_or_create_room(self, room_id: int, name: Optional[str] = None) -> RoomNode:
        if room_id not in self.rooms:
            self.rooms[room_id] = RoomNode(
                room_id=room_id,
                name=name or f"Room_{room_id}",
                status="EXPLORING"
            )
        return self.rooms[room_id]

    def add_doorway(self, door_id: str, world_pos: Tuple[float, float], room_a: int, room_b: int) -> DoorwayNode:
        if door_id not in self.doorways:
            node = DoorwayNode(
                door_id=door_id,
                world_pos=world_pos,
                connects_rooms=(room_a, room_b)
            )
            self.doorways[door_id] = node
            
            # Register in rooms
            r_a = self.get_or_create_room(room_a)
            r_b = self.get_or_create_room(room_b)
            if door_id not in r_a.doorways: r_a.doorways.append(door_id)
            if door_id not in r_b.doorways: r_b.doorways.append(door_id)

            self.edges.append({"source": f"Room_{room_a}", "relation": "CONNECTS_VIA", "target": door_id})
            self.edges.append({"source": door_id, "relation": "LEADS_TO", "target": f"Room_{room_b}"})
        return self.doorways[door_id]

    def add_entity(self, entity_id: str, class_name: str, pos: Tuple[float, float, float], room_id: int, confidence: float = 1.0) -> EntityNode:
        now = time.time()
        if entity_id in self.entities:
            self.entities[entity_id].last_seen = now
            self.entities[entity_id].confidence = max(self.entities[entity_id].confidence, confidence)
            return self.entities[entity_id]

        node = EntityNode(
            entity_id=entity_id,
            class_name=class_name,
            world_pos=pos,
            room_id=room_id,
            confidence=confidence,
            first_seen=now,
            last_seen=now
        )
        self.entities[entity_id] = node

        room = self.get_or_create_room(room_id)
        if entity_id not in room.entities:
            room.entities.append(entity_id)

        self.edges.append({"source": entity_id, "relation": "LOCATED_IN", "target": f"Room_{room_id}"})
        return node

    def query(self, query_type: str, **kwargs) -> Any:
        """Query semantic graph."""
        if query_type == "get_entities_in_room":
            room_id = kwargs.get("room_id")
            return [self.entities[eid] for eid in self.rooms.get(room_id, RoomNode(room_id, "")).entities]
        elif query_type == "find_by_class":
            cls_name = kwargs.get("class_name")
            return [e for e in self.entities.values() if e.class_name.lower() == cls_name.lower()]
        elif query_type == "unexplored_rooms":
            return [r for r in self.rooms.values() if r.status != "EXPLORED"]
        return []

    def query_memory(self, question: str) -> List[EntityNode]:
        """Natural-language keyword lookup over discovered entities (e.g. 'where is the chair?')."""
        q = question.lower()
        for cls_name in {e.class_name.lower() for e in self.entities.values()}:
            if cls_name in q:
                return self.query("find_by_class", class_name=cls_name)
        return list(self.entities.values())

    def export_json(self) -> str:
        return json.dumps({
            "rooms": {k: vars(v) for k, v in self.rooms.items()},
            "doorways": {k: vars(v) for k, v in self.doorways.items()},
            "entities": {k: vars(v) for k, v in self.entities.items()},
            "edges": self.edges
        }, indent=2)
