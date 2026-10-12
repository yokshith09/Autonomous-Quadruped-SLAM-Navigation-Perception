"""Episodic Memory Engine: Temporal timeline of mission milestones and discoveries."""
from dataclasses import dataclass, field
from typing import List, Dict, Any, Optional


@dataclass
class EpisodicEvent:
    timestamp: float
    event_type: str
    description: str
    coordinates: Optional[Dict[str, float]] = None
    metadata: Dict[str, Any] = field(default_factory=dict)


class EpisodicMemory:
    """Chronological mission memory store."""

    def __init__(self):
        self.events: List[EpisodicEvent] = []

    def record_event(self, timestamp: float, event_type: str, description: str, coordinates: Optional[Dict[str, float]] = None, **kwargs) -> EpisodicEvent:
        event = EpisodicEvent(
            timestamp=round(timestamp, 2),
            event_type=event_type,
            description=description,
            coordinates=coordinates,
            metadata=kwargs
        )
        self.events.append(event)
        return event

    def get_events(self, event_type: Optional[str] = None) -> List[EpisodicEvent]:
        if event_type:
            return [e for e in self.events if e.event_type == event_type]
        return list(self.events)

    def generate_timeline_markdown(self) -> str:
        lines = [
            "# 🕒 Episodic Mission Timeline",
            "",
            "| Sim Time | Event Type | Description | Location |",
            "| :--- | :--- | :--- | :--- |"
        ]
        for e in self.events:
            loc_str = f"({e.coordinates['x']:.2f}, {e.coordinates['y']:.2f})" if e.coordinates and 'x' in e.coordinates else "-"
            lines.append(f"| **{e.timestamp}s** | {e.event_type} | {e.description} | {loc_str} |")
        return "\n".join(lines)
