"""Base Agent class for robotic multi-agent subsystems."""
from abc import ABC, abstractmethod
import time
import threading
from .bus import MessageBus


class BaseAgent(ABC):
    """Abstract base class for all autonomous subsystem agents."""

    def __init__(self, name: str, bus: MessageBus, update_rate_hz: float = 10.0):
        self.name = name
        self.bus = bus
        self.update_rate_hz = update_rate_hz
        self.period = 1.0 / max(0.001, update_rate_hz)
        self.is_running = False
        self.last_update_time = 0.0
        self.error_count = 0

    @abstractmethod
    def initialize(self) -> bool:
        """Initialize agent resources, subscribe to bus topics."""
        pass

    @abstractmethod
    def step(self, current_time: float) -> None:
        """Periodic tick for the agent."""
        pass

    def shutdown(self) -> None:
        """Clean teardown of resources."""
        self.is_running = False

    def handle_error(self, err: Exception) -> None:
        """Fault isolation and error reporting."""
        self.error_count += 1
        print(f"[Agent {self.name} ERROR #{self.error_count}] {err}")
