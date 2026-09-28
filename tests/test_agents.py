"""Runnable verification tests for the Multi-Agent Robotics Stack."""
import numpy as np
import math
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from agents import (
    MessageBus, PoseMsg, ScanMsg, GridMapMsg, FrontierTargetMsg, PathMsg,
    SpatialKnowledgeGraph, EpisodicMemory, MappingAgent, FrontierAgent, SupervisorAgent
)
from grid import BayesianOccupancyGrid


def test_message_bus():
    print("Running test_message_bus...", end=" ")
    bus = MessageBus()
    received = []

    def on_pose(msg: PoseMsg):
        received.append(msg)

    bus.subscribe("pose/state", on_pose)
    bus.publish("pose/state", PoseMsg(timestamp=1.0, x=2.5, y=-1.0, yaw=0.5))

    assert len(received) == 1
    assert received[0].x == 2.5
    assert bus.get_latest("pose/state").yaw == 0.5
    print("PASSED")


def test_spatial_knowledge_graph():
    print("Running test_spatial_knowledge_graph...", end=" ")
    kg = SpatialKnowledgeGraph()
    kg.get_or_create_room(1, "Office_West")
    kg.get_or_create_room(2, "Corridor_Main")
    kg.add_doorway("door_1", (0.0, 1.5), 1, 2)
    kg.add_entity("victim_1", "person", (2.0, 1.0, 0.2), room_id=1)

    assert len(kg.rooms) == 2
    assert "door_1" in kg.doorways
    assert len(kg.entities) == 1

    # Query
    entities_in_r1 = kg.query("get_entities_in_room", room_id=1)
    assert len(entities_in_r1) == 1
    assert entities_in_r1[0].class_name == "person"
    print("PASSED")


def test_episodic_memory():
    print("Running test_episodic_memory...", end=" ")
    mem = EpisodicMemory()
    mem.record_event(10.0, "ROOM_ENTERED", "Entered Room 1", {"x": 1.0, "y": 2.0})
    mem.record_event(25.5, "VICTIM_FOUND", "Located survivor", {"x": 3.5, "y": 1.2})

    events = mem.get_events()
    assert len(events) == 2
    assert events[0].event_type == "ROOM_ENTERED"

    md = mem.generate_timeline_markdown()
    assert "ROOM_ENTERED" in md
    assert "VICTIM_FOUND" in md
    print("PASSED")


def test_room_segmentation():
    print("Running test_room_segmentation...", end=" ")
    bus = MessageBus()
    mapping_agent = MappingAgent(bus, x_min=-5.0, x_max=5.0, y_min=-5.0, y_max=5.0, resolution=0.1)

    # Construct a synthetic 2-room map separated by a wall with a 1-meter doorway
    # Room 1: x in [-4, -0.5], y in [-4, 4]
    # Room 2: x in [0.5, 4], y in [-4, 4]
    # Wall at x in [-0.5, 0.5], with doorway at y in [-0.5, 0.5]
    for wy in np.arange(-4.0, 4.0, 0.1):
        for wx in np.arange(-4.0, 4.0, 0.1):
            gx, gy = mapping_agent.grid.world_to_grid(wx, wy)
            # Dividing wall
            if -0.5 <= wx <= 0.5:
                if -0.5 <= wy <= 0.5:
                    # Doorway (FREE)
                    mapping_agent.grid.grid_log_odds[gy, gx] = mapping_agent.grid.l_min
                else:
                    # Wall (OCCUPIED)
                    mapping_agent.grid.grid_log_odds[gy, gx] = mapping_agent.grid.l_max
            else:
                # Inside rooms (FREE)
                mapping_agent.grid.grid_log_odds[gy, gx] = mapping_agent.grid.l_min

    # Run Watershed Room Segmentation
    markers, doorways = mapping_agent.segment_rooms()
    num_unique_rooms = len(set(np.unique(markers)) - {0, -1})

    # Should detect 2 distinct room basins
    assert num_unique_rooms >= 2, f"Expected at least 2 rooms, got {num_unique_rooms}"
    print("PASSED")


def test_systematic_room_exploration():
    print("Running test_systematic_room_exploration...", end=" ")
    bus = MessageBus()
    grid = BayesianOccupancyGrid(-5.0, 5.0, -5.0, 5.0, 0.1)
    frontier_agent = FrontierAgent(bus, grid_ref=grid)
    frontier_agent.initialize()

    # Robot in Room 1
    frontier_agent.active_room_id = 2
    frontier_agent.on_pose(PoseMsg(timestamp=1.0, x=-2.0, y=0.0, yaw=0.0))

    # Mock GridMapMsg with mock room labels
    mock_labels = np.zeros((grid.height, grid.width), dtype=np.int32)
    # Left half room 2, right half room 3
    mock_labels[:, :grid.width//2] = 2
    mock_labels[:, grid.width//2:] = 3

    frontier_agent.on_grid(GridMapMsg(
        timestamp=1.0,
        log_odds=grid.grid_log_odds,
        free_cells=1000,
        occupied_cells=200,
        unknown_cells=800,
        explored_pct=60.0,
        room_labels=mock_labels,
        doorways=[(0.0, 0.0)]
    ))

    # Mock detect_frontiers to return one in room 2 (x=-1) and one in room 3 (x=3)
    gx_r2, gy_r2 = grid.world_to_grid(-1.0, 0.0)
    gx_r3, gy_r3 = grid.world_to_grid(3.0, 0.0)
    grid.detect_frontiers = lambda: [(gx_r2, gy_r2, 10), (gx_r3, gy_r3, 50)]

    # Frontier agent MUST pick room 2 frontier even though room 3 has larger area (50 vs 10)
    selected = frontier_agent.select_frontier(current_time=2.0)
    assert selected is not None
    assert selected.room_id == 2
    assert selected.target_gx == gx_r2
    print("PASSED")


def test_planning_agent():
    print("Running test_planning_agent...", end=" ")
    from agents import PlanningAgent
    bus = MessageBus()
    grid = BayesianOccupancyGrid(-5.0, 5.0, -5.0, 5.0, 0.1)
    
    # Mark a 2x2m area as FREE
    for wy in np.arange(-1.0, 1.0, 0.1):
        for wx in np.arange(-1.0, 1.0, 0.1):
            gx, gy = grid.world_to_grid(wx, wy)
            grid.grid_log_odds[gy, gx] = grid.l_min

    planner = PlanningAgent(bus, grid_ref=grid, replan_cooldown=0.0)
    planner.initialize()

    planner.on_pose(PoseMsg(timestamp=1.0, x=0.0, y=0.0, yaw=0.0))
    tgt_gx, tgt_gy = grid.world_to_grid(0.5, 0.5)
    planner.on_target(FrontierTargetMsg(
        timestamp=1.0, target_gx=tgt_gx, target_gy=tgt_gy,
        target_wx=0.5, target_wy=0.5, room_id=1, area=10
    ))

    path = planner.plan_path(current_time=1.0)
    assert path is not None
    assert len(path) >= 2
    assert bus.get_latest("plan/path") is not None
    print("PASSED")


def test_locomotion_agent():
    print("Running test_locomotion_agent...", end=" ")
    from agents import LocomotionAgent
    bus = MessageBus()
    loco = LocomotionAgent(bus, robot_driver=None)
    loco.initialize()

    # 1. Test idle state when no waypoints (Must NOT spin)
    loco.on_pose(PoseMsg(timestamp=1.0, x=0.0, y=0.0, yaw=0.0))
    loco.step(current_time=1.0)
    assert loco.speed == 0.0
    assert loco.state_str == "IDLE"

    # 2. Test walk when waypoints provided
    loco.on_path(PathMsg(timestamp=1.1, waypoints=[(1.0, 0.0), (2.0, 0.0)]))
    loco.step(current_time=1.1)
    assert loco.speed > 0.0
    assert loco.state_str == "WALK"

    # 3. Test reactive obstacle repulsion at non-zero world pose (body-frame scan)
    loco.on_pose(PoseMsg(timestamp=1.2, x=5.0, y=-3.0, yaw=1.2))
    loco.on_path(PathMsg(timestamp=1.2, waypoints=[(6.0, -2.0), (7.0, -1.0)]))
    # Add obstacle point 20cm ahead slightly to the left in robot body frame
    scan = ScanMsg(timestamp=1.2, points_2d=np.array([[0.20, 0.08]]))
    loco.on_scan(scan)
    loco.step(current_time=1.2)
    assert loco.state_str == "AVOID", f"Expected AVOID at non-zero pose, got {loco.state_str}"

    # 4. Test anti-stuck detection
    loco.last_stuck_check_time = 0.0
    loco.last_stuck_pos = (5.0, -3.0)
    # Simulate staying stuck at same position over consecutive intervals
    loco.step(current_time=2.5)
    loco.step(current_time=3.6)
    assert loco.recovery_countdown > 0 or loco.state_str in ["RECOVER_BACKUP", "IDLE"]
    print("PASSED")


def test_frontier_boundary_room_association():
    print("Running test_frontier_boundary_room_association...", end=" ")
    bus = MessageBus()
    grid = BayesianOccupancyGrid(-5.0, 5.0, -5.0, 5.0, 0.1)
    fa = FrontierAgent(bus, grid_ref=grid)
    fa.initialize()

    # Create room_labels where Room 1 is x < 0, Room 2 is x > 0
    # BUT on the boundary (x = 0), cells are 0 (unknown/unsegmented)
    labels = np.zeros((grid.height, grid.width), dtype=np.int32)
    labels[:, :grid.width // 2 - 1] = 1
    labels[:, grid.width // 2 + 1:] = 2

    gx_boundary = grid.width // 2 - 1
    gy_boundary = grid.height // 2

    # Verify cell at boundary has label 0 in raw map
    assert labels[gy_boundary, gx_boundary] == 0

    # _get_room_at should correctly inspect neighborhood and resolve to Room 1
    assigned_room = fa._get_room_at(gx_boundary, gy_boundary, labels, radius=3)
    assert assigned_room == 1, f"Expected Room 1, got {assigned_room}"
    print("PASSED")


def test_semantic_entity_extraction():
    print("Running test_semantic_entity_extraction...", end=" ")
    bus = MessageBus()
    ma = MappingAgent(bus, -5.0, 5.0, -5.0, 5.0, 0.1)
    sup = SupervisorAgent(bus)
    ma.initialize()
    sup.initialize()

    # Clear free space in center
    ma.grid.grid_log_odds[20:80, 20:80] = ma.grid.l_min
    # Add a long wall (>= 1.2m -> 15 cells at 0.1m res)
    ma.grid.grid_log_odds[20, 20:40] = ma.grid.l_max
    # Add a compact chair cluster (3x3 cells = 0.3m x 0.3m inside room)
    ma.grid.grid_log_odds[50:53, 50:53] = ma.grid.l_max

    entities = ma.extract_semantic_entities(current_time=1.0)
    classes = {e["class_name"] for e in entities}
    assert "wall" in classes, f"Expected 'wall' in {classes}"
    assert "chair" in classes, f"Expected 'chair' in {classes}"
    assert len(sup.knowledge_graph.entities) >= 2
    print("PASSED")


def load_tests(loader, standard_tests, pattern):
    import unittest
    suite = unittest.TestSuite()
    for name, obj in list(globals().items()):
        if name.startswith("test_") and callable(obj):
            suite.addTest(unittest.FunctionTestCase(obj))
    return suite


if __name__ == "__main__":
    print("=== RUNNING MULTI-AGENT STACK UNIT TESTS ===")
    test_message_bus()
    test_spatial_knowledge_graph()
    test_episodic_memory()
    test_room_segmentation()
    test_systematic_room_exploration()
    test_planning_agent()
    test_locomotion_agent()
    test_frontier_boundary_room_association()
    test_semantic_entity_extraction()
    print("=== ALL TESTS PASSED SUCCESSFULLY! ===")


