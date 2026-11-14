import cv2
"""Runnable verification tests for the Multi-Agent Robotics Stack."""
import math
import numpy as np
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from agents import (
    MessageBus, PoseMsg, ScanMsg, GridMapMsg, FrontierTargetMsg, PathMsg,
    SpatialKnowledgeGraph, EpisodicMemory, MappingAgent, FrontierAgent, SupervisorAgent
)
from core.grid import BayesianOccupancyGrid


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


def test_planner_avoids_unknown_space():
    print("Running test_planner_avoids_unknown_space...", end=" ")
    from core.planner import a_star_search
    grid = BayesianOccupancyGrid(-5.0, 5.0, -5.0, 5.0, 0.1)
    # A thin never-observed strip (like the space beyond an unseen wall) almost splits the free area. Crossing
    # it is short; the only known-free route goes around its end, far away.
    grid.grid_log_odds[20:80, 20:80] = grid.l_min
    grid.grid_log_odds[23:80, 47:54] = 0.0
    start, goal = (35, 60), (65, 60)
    path = a_star_search(grid, start, goal)
    assert path, "A detour through known free space exists"
    for wx, wy in path:
        gx, gy = grid.world_to_grid(wx, wy)
        assert not (23 <= gy < 80 and 47 <= gx < 54), f"Path entered unknown space at {(gx, gy)}"
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
    assert loco.recovery_countdown > 0 or loco.state_str in ["RECOVER_TURN", "IDLE"]
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
    chairs = sup.knowledge_graph.query_memory("Where is the chair?")
    assert len(chairs) >= 1 and chairs[0].class_name == "chair"
    print("PASSED")


def _walled_room():
    """3 x 3 m free room inside a one-cell wall; everything else unknown."""
    grid = BayesianOccupancyGrid(-2.0, 2.0, -2.0, 2.0, 0.05)
    lo = grid.grid_log_odds
    lo[:] = 0.0
    lo[10:71, 10:71] = 2.0
    lo[11:70, 11:70] = -2.0
    return grid


def _explore_static(fa, x, y, max_decisions=50):
    """Walk the robot to each target on a frozen map; return decisions until the agent returns no target."""
    fa.patrol = False  # these tests measure exploration finishing; patrolling afterwards is tested separately
    fa.on_pose(PoseMsg(timestamp=0.0, x=x, y=y, yaw=0.0))
    for n in range(1, max_decisions + 1):
        tgt = fa.select_frontier(current_time=n * 10.0)
        if tgt is None:
            return n
        fa.on_pose(PoseMsg(timestamp=n, x=tgt.target_wx, y=tgt.target_wy, yaw=0.0))
    return None


def test_frontier_behind_wall_is_ignored():
    """F1: depth rays leaking past a wall leave free cells outside it; their frontier cannot be observed from inside."""
    print("Running test_frontier_behind_wall_is_ignored...", end=" ")
    grid = _walled_room()
    grid.grid_log_odds[35:46, 71] = -2.0  # leaked free strip just outside the east wall, unknown beyond
    assert grid.detect_frontiers()
    fa = FrontierAgent(MessageBus(), grid_ref=grid)
    assert _explore_static(fa, 0.0, 0.0) == 1 and fa.state == "MISSION_COMPLETE"
    print("PASSED")


def test_unobservable_frontier_is_retired_not_revisited():
    """F2: a frontier still unknown after the robot stood at its viewpoint is retired for good (baseline re-picked one
    such spot 141 times in its last 90 s)."""
    print("Running test_unobservable_frontier_is_retired_not_revisited...", end=" ")
    grid = _walled_room()
    grid.grid_log_odds[38:41, 50:53] = 0.0  # unknown pocket the frozen map never resolves
    fa = FrontierAgent(MessageBus(), grid_ref=grid)
    n = _explore_static(fa, -1.0, -1.0)
    assert n is not None and n <= 4 and fa.state == "MISSION_COMPLETE"
    print("PASSED")


def test_unfinished_room_is_not_abandoned_on_label_change():
    """F3: crossing into cells labelled as another room must not abandon a room that still has frontiers (live run
    20261107: the active room flipped 4 <-> 6 twenty times and one room-4 target was re-picked 19 times)."""
    print("Running test_unfinished_room_is_not_abandoned_on_label_change...", end=" ")
    grid = _walled_room()
    grid.grid_log_odds[38:41, 20:23] = 0.0  # unknown pocket in the west room (4)
    grid.grid_log_odds[38:41, 58:61] = 0.0  # unknown pocket in the east room (6)
    labels = np.zeros((grid.height, grid.width), dtype=np.int32)
    labels[:, :40] = 4
    labels[:, 40:] = 6
    fa = FrontierAgent(MessageBus(), grid_ref=grid)
    fa.on_grid(GridMapMsg(timestamp=0.0, log_odds=grid.grid_log_odds, free_cells=0, occupied_cells=0, unknown_cells=0,
                          explored_pct=0.0, room_labels=labels, doorways=[]))
    fa.on_pose(PoseMsg(timestamp=0.0, x=-0.3, y=0.0, yaw=0.0))
    assert fa.select_frontier(current_time=1.0).room_id == 4
    for k in range(6):  # the robot wobbles across the 4 / 6 label boundary on its way
        fa.on_pose(PoseMsg(timestamp=k, x=0.05 if k % 2 == 0 else -0.05, y=0.0, yaw=0.0))
        tgt = fa.select_frontier(current_time=2.0 + k)
        assert fa.active_room_id == 4 and tgt.room_id == 4, f"abandoned room 4 for {fa.active_room_id}"
    print("PASSED")


class _FakeSim:
    handle_world = 0

    def getObjectOrientation(self, *a):
        return [0.0, 0.0, 0.0]


class _FakeRobot:
    """Stands in for LizardRobot. leg_load returns what the real robot shows at gait command 0.75 (stall_probe):
    0.0062 walking freely, 0.020 pushing on a wall."""
    sim, base, last_odom_lateral = _FakeSim(), 1, 0.0

    def __init__(self):
        self.blocked = False

    def steer_heading(self, *a, **k):
        return 0.0

    def apply_gait(self, *a):
        pass

    def stop_motors(self):
        pass

    def get_odometry_imu_delta(self, dt, speed):
        from core.robot import gait_body_speed
        return gait_body_speed(speed) * dt, 0.0

    def leg_load(self):
        return 0.020 if self.blocked else 0.0062

    up = 1.0

    def body_up(self):
        return self.up


def test_blocked_legs_are_felt_and_phantom_odometry_stops():
    """Live run 10: 16 s pushing a wall while odometry reported 7.9 cm/s took the position error from 11 to 122 cm; the
    stall watchdog could not see it because it measures motion with that same dead-reckoned pose. The legs' load can."""
    print("Running test_blocked_legs_are_felt_and_phantom_odometry_stops...", end=" ")
    from agents import LocomotionAgent
    bus = MessageBus()
    events, acts = [], []
    bus.subscribe("mission/event", lambda e: events.append((e.timestamp, e.event_type)))
    bus.subscribe("control/actuation", lambda a: acts.append(a))
    robot = _FakeRobot()
    loco = LocomotionAgent(bus, robot_driver=robot)
    loco.initialize()
    loco.on_path(PathMsg(timestamp=0.0, waypoints=[(0.1 * i, 0.0) for i in range(1, 200)]))
    for k in range(120):  # 6 s; the estimated pose keeps advancing 8 cm/s, as dead reckoning does
        t = k * 0.05
        robot.blocked = t >= 2.0
        loco.on_pose(PoseMsg(timestamp=t, x=0.08 * t, y=0.0, yaw=0.0))
        loco.step(t)
        if k == 38:
            assert not events and acts[-1].odom_forward > 0.0, "free walking reported stuck or no odometry"
    stuck = [t for t, e in events if e == "ROBOT_STUCK"]
    assert stuck and 2.0 < stuck[0] <= 2.8, f"blocked legs not felt within 0.8 s (stuck events at {stuck})"
    net = sum(a.odom_forward for a in acts if 2.0 - 1e-9 <= a.timestamp <= stuck[0] + 1e-9)
    assert abs(net) < 1e-9, f"odometry reported while blocked was not taken back (net {net * 100:.2f} cm)"
    print("PASSED")


def test_recovery_turns_away_and_never_reverses():
    """Live runs 13 and 14: the reverse gait flipped the body within 0.4 s. Recovery now stops the legs and turns the body
    towards the more open side (here left: far returns on the left, a near one on the right)."""
    print("Running test_recovery_turns_away_and_never_reverses...", end=" ")
    from agents import LocomotionAgent
    bus = MessageBus()
    events = []
    bus.subscribe("mission/event", lambda e: events.append(e.event_type))
    robot = _FakeRobot()
    gaits, steers = [], []
    robot.apply_gait = lambda t, sp: gaits.append(sp)
    robot.steer_heading = lambda orient, cur, des: steers.append(des)
    loco = LocomotionAgent(bus, robot_driver=robot)
    loco.initialize()
    loco.on_scan(ScanMsg(timestamp=0.0, points_2d=np.array([[1.0, 0.8], [1.2, 0.9], [0.4, -0.1]])))
    loco.on_path(PathMsg(timestamp=0.0, waypoints=[(0.1 * i, 0.0) for i in range(1, 200)]))
    states = []
    for k in range(60):
        t = k * 0.05
        robot.blocked = t >= 0.5
        loco.on_pose(PoseMsg(timestamp=t, x=0.0, y=0.0, yaw=0.0))
        loco.step(t)
        states.append(loco.state_str)
    assert "ROBOT_STUCK" in events and "RECOVER_TURN" in states
    assert min(gaits) >= 0.0, f"a reverse gait was commanded ({min(gaits)})"
    assert any(abs(h - 1.2) < 1e-6 for h in steers), f"did not turn towards the open (left) side: {sorted(set(steers))[:5]}"
    print("PASSED")


def test_turn_walk_dithering_is_not_a_stall():
    """Live runs 13/14: turning and walking alternately for seconds without moving was sampled mid-walk and called a stall.
    Only a robot that walked most of the last second can be stalled; one that walks the whole time and does not move is."""
    print("Running test_turn_walk_dithering_is_not_a_stall...", end=" ")
    from agents import LocomotionAgent

    def run(yaw_of_step):
        bus = MessageBus()
        events = []
        bus.subscribe("mission/event", lambda e: events.append(e.event_type))
        loco = LocomotionAgent(bus, robot_driver=_FakeRobot())
        loco.initialize()
        loco.on_path(PathMsg(timestamp=0.0, waypoints=[(0.1 * i, 0.0) for i in range(1, 200)]))
        for k in range(100):  # 5 s
            loco.on_pose(PoseMsg(timestamp=k * 0.05, x=0.0, y=0.0, yaw=yaw_of_step(k)))
            loco.step(k * 0.05)
        return events

    assert "ROBOT_STUCK" not in run(lambda k: 1.0 if k % 2 else 0.0), "dithering was called a stall"
    assert "ROBOT_STUCK" in run(lambda k: 0.0), "a robot walking in place was not called a stall"
    print("PASSED")


def test_contact_obstacle_stays_in_map():
    """Live run 23: five contacts in a row with an obstacle the camera cannot see. A contact marks the cell 0.25 m ahead
    as occupied, and camera rays passing over it do not clear it."""
    print("Running test_contact_obstacle_stays_in_map...", end=" ")
    from agents import MappingAgent
    from agents.bus import MissionEventMsg
    bus = MessageBus()
    m = MappingAgent(bus)
    m.initialize()
    bus.publish("mission/event", MissionEventMsg(timestamp=1.0, event_type="ROBOT_STUCK",
                                                 details={"x": 0.0, "y": 0.0, "contact": True, "yaw": 0.0}))
    gx, gy = m.grid.world_to_grid(0.25, 0.0)
    assert m.grid.grid_log_odds[gy, gx] >= m.grid.thresh_occ
    m.on_pose(PoseMsg(timestamp=1.1, x=-1.0, y=0.0, yaw=0.0))
    m.on_scan(ScanMsg(timestamp=1.1, points_2d=np.array([[3.0, 0.0]]),
                      base_points_3d=np.array([[3.0, 0.0, 0.3]]), world_points_3d=np.array([[3.0, 0.0, 0.3]])))
    for k in range(5):
        m.step(1.1 + 0.1 * k)
    assert m.grid.grid_log_odds[gy, gx] >= m.grid.thresh_occ, "camera rays erased the contact obstacle"
    # a stall that is not a contact (no progress) marks nothing
    bus.publish("mission/event", MissionEventMsg(timestamp=2.0, event_type="ROBOT_STUCK", details={"x": 2.0, "y": 2.0}))
    assert len(m.contact_cells) == 1
    print("PASSED")


def test_planner_keeps_path_unless_clearly_shorter():
    """Live runs: the blue line flipped between two near-equal routes every 4 s. A re-plan to the same goal keeps the
    current path unless the new one is at most 85 % as long."""
    print("Running test_planner_keeps_path_unless_clearly_shorter...", end=" ")
    import agents.planning_agent as pa
    from agents.bus import FrontierTargetMsg
    from core.grid import BayesianOccupancyGrid
    bus = MessageBus()
    grid = BayesianOccupancyGrid(-5.5, 5.5, -5.5, 5.5, 0.05)
    grid.grid_log_odds[:] = grid.l_min
    agent = pa.PlanningAgent(bus, grid_ref=grid)
    agent.initialize()
    paths = [[(0.0, 1.0), (2.0, 1.0), (2.0, 0.0)], [(0.0, -1.0), (2.0, -1.0), (2.0, 0.0)], [(1.0, 0.0), (2.0, 0.0)]]
    calls = iter(paths)
    orig_a, orig_s = pa.a_star_search, pa.smooth_path
    pa.a_star_search, pa.smooth_path = (lambda g, s, e, smooth=False, **kw: next(calls)), (lambda g, p, **kw: p)
    try:
        gx, gy = grid.world_to_grid(2.0, 0.0)
        agent.on_target(FrontierTargetMsg(timestamp=0.0, target_gx=gx, target_gy=gy, target_wx=2.0, target_wy=0.0,
                                          room_id=1, area=10, is_doorway=False))
        agent.on_pose(PoseMsg(timestamp=0.0, x=0.0, y=0.0, yaw=0.0))
        first = agent.plan_path(0.0)
        assert agent.plan_path(5.0) == first, "an equally long route replaced the current path"
        assert agent.plan_path(10.0) == paths[2], "a clearly shorter route was not taken"
    finally:
        pa.a_star_search, pa.smooth_path = orig_a, orig_s
    print("PASSED")


def test_planner_escapes_a_ring_of_clearance_margins():
    """Live run 30: remembered contacts around the robot closed a ring of clearance margins; the robot stood idle with every
    target unreachable. Margins near the robot's own cell must not block it; occupied cells still do."""
    print("Running test_planner_escapes_a_ring_of_clearance_margins...", end=" ")
    from core.grid import BayesianOccupancyGrid
    from core.planner import a_star_search
    grid = BayesianOccupancyGrid(-5.5, 5.5, -5.5, 5.5, 0.05)
    grid.grid_log_odds[:] = grid.l_min
    sx, sy = grid.world_to_grid(0.0, 0.0)
    for dx, dy in ((5, 0), (-5, 0), (0, 5), (0, -5)):          # four single cells 0.25 m away: margins form a ring
        grid.grid_log_odds[sy + dy, sx + dx] = grid.l_max
    goal = grid.world_to_grid(2.0, 1.0)
    path = a_star_search(grid, (sx, sy), goal)
    assert path, "robot inside a ring of clearance margins could not plan out"
    cells = [grid.world_to_grid(x, y) for x, y in path]
    assert all(grid.grid_log_odds[cy, cx] < grid.thresh_occ for cx, cy in cells), "path crosses an occupied cell"
    # a real wall of occupied cells still blocks
    grid.grid_log_odds[sy - 6:sy + 7, sx + 3] = grid.l_max
    grid.grid_log_odds[sy - 6:sy + 7, sx - 3] = grid.l_max
    grid.grid_log_odds[sy + 3, sx - 6:sx + 7] = grid.l_max
    grid.grid_log_odds[sy - 3, sx - 6:sx + 7] = grid.l_max
    assert not a_star_search(grid, (sx, sy), goal), "planned through a closed wall"
    print("PASSED")


def test_tabletop_is_scan_structure_but_not_an_obstacle():
    """The robot (~7 cm tall) fits under a 40-50 cm tabletop. The tabletop stays in the scan-matching points (SLAM
    keeps its structure) but is not an obstacle for the map or the bumper; the table legs are both."""
    print("Running test_tabletop_is_scan_structure_but_not_an_obstacle...", end=" ")
    import agents.perception_agent as pa
    bus = MessageBus()
    got = []
    bus.subscribe("sensor/scan", lambda m: got.append(m))
    pts = np.array([[1.0, 0.0, 0.40], [1.0, 0.1, 0.42], [1.0, -0.1, 0.41], [1.0, 0.05, 0.40],   # tabletop
                    [1.0, 0.5, 0.10], [1.0, -0.5, 0.10], [1.0, 0.5, 0.15], [1.0, -0.5, 0.15],    # legs
                    [2.0, 0.0, -0.05],                                                           # floor
                    [0.30, 0.0, 0.01], [0.60, 0.0, 0.01]])    # floor ahead (no floor there reads as a drop)
    orig = pa.depth_to_base_points
    pa.depth_to_base_points = lambda *a, **k: pts
    try:
        agent = pa.PerceptionAgent(bus, camera_matrix={"T_base_cam": None})
        agent.point_stride = 1
        agent.process_frame(np.ones((2, 2)), None, 0.0)
    finally:
        pa.depth_to_base_points = orig
    m = got[-1]
    top = lambda a: np.any(np.all(np.isclose(a[:, :2], [1.0, 0.0]), axis=1))
    assert top(m.points_2d), "tabletop dropped from the scan used for matching"
    assert not top(m.obstacle_2d), "tabletop treated as an obstacle for the bumper"
    assert m.base_points_3d[:, 2].max() <= pa.OBSTACLE_MAX_Z, "tabletop sent to the map"
    assert len(m.obstacle_2d) == 4, "table legs missing from the obstacle points"
    print("PASSED")


def test_narrow_gap_opens_only_with_the_narrow_setting():
    """A 0.35 m gap is closed by the 0.20 m margin. With --narrow-gaps the planner retries at 0.15 m and goes through;
    without it (the scene-1 default) the behaviour is unchanged."""
    print("Running test_narrow_gap_opens_only_with_the_narrow_setting...", end=" ")
    import agents.planning_agent as pa
    from core import planner
    from agents.bus import FrontierTargetMsg
    from core.grid import BayesianOccupancyGrid
    grid = BayesianOccupancyGrid(-3.0, 3.0, -3.0, 3.0, 0.05)
    grid.grid_log_odds[:] = grid.l_min
    wx, _ = grid.world_to_grid(0.0, 0.0)
    gy0, gy1 = grid.world_to_grid(0.0, -0.175)[1], grid.world_to_grid(0.0, 0.175)[1]
    grid.grid_log_odds[:, wx] = grid.l_max                       # a wall across the room at x = 0 ...
    grid.grid_log_odds[gy0:gy0 + 7, wx] = grid.l_min             # ... with a 0.35 m gap (7 cells)
    def plan(narrow):
        planner.NARROW_GAPS = narrow
        bus = MessageBus()
        agent = pa.PlanningAgent(bus, grid_ref=grid)
        agent.initialize()
        gx, gy = grid.world_to_grid(2.0, 0.0)
        agent.on_target(FrontierTargetMsg(timestamp=0.0, target_gx=gx, target_gy=gy, target_wx=2.0, target_wy=0.0,
                                          room_id=1, area=10, is_doorway=False))
        agent.on_pose(PoseMsg(timestamp=0.0, x=-2.0, y=0.0, yaw=0.0))
        return agent.plan_path(0.0)
    try:
        assert not plan(False), "default margin went through a 0.35 m gap"
        path = plan(True)
        assert path and path[-1][0] > 1.5, "narrow setting did not open the 0.35 m gap"
        cells = [grid.world_to_grid(x, y) for x, y in path]
        assert all(grid.grid_log_odds[cy, cx] < grid.thresh_occ for cx, cy in cells), "path crosses the wall"
    finally:
        planner.NARROW_GAPS = False
    print("PASSED")


def test_stall_threshold_follows_the_floor_calibration():
    """Rescue run 1: on a slow floor (true speed 0.060 m/s) ordinary walking moved under the fixed 5 cm per second and
    raised false stalls. The threshold scales with the floor calibration; a robot really standing still is still caught."""
    print("Running test_stall_threshold_follows_the_floor_calibration...", end=" ")
    from agents import LocomotionAgent
    import core.robot as cr

    def run(speed_mps):
        bus = MessageBus()
        events = []
        bus.subscribe("mission/event", lambda e: events.append(e.event_type))
        loco = LocomotionAgent(bus, robot_driver=_FakeRobot())
        loco.initialize()
        loco.on_path(PathMsg(timestamp=0.0, waypoints=[(0.1 * i, 0.0) for i in range(1, 200)]))
        for k in range(100):  # 5 s
            loco.on_pose(PoseMsg(timestamp=k * 0.05, x=speed_mps * k * 0.05, y=0.0, yaw=0.0))
            loco.step(k * 0.05)
        return events
    old = cr.ODOM_SCALE
    try:
        cr.ODOM_SCALE = 0.754
        assert "ROBOT_STUCK" not in run(0.045), "normal walking on the slow floor called a stall"
        assert "ROBOT_STUCK" in run(0.0), "a robot standing still was not called a stall"
    finally:
        cr.ODOM_SCALE = old
    print("PASSED")


def test_gyro_bias_is_measured_while_standing_still():
    """Rescue run 3: without the right-angle prior a 0.05 deg/s gyro bias drove heading error to 5 deg. The robot stands
    still for calibrate_until seconds; SLAM averages the gyro there and removes the bias afterwards."""
    print("Running test_gyro_bias_is_measured_while_standing_still...", end=" ")
    from agents import SLAMAgent
    from agents.bus import ActuationMsg
    bias = math.radians(0.05)
    slam = SLAMAgent(MessageBus())
    t = 0.0
    settle = math.radians(-1.5) / 40           # the body settles -1.5 deg over the first 2 s (rescue run 4)
    for k in range(160):                       # 8 s standing still at 20 Hz
        t = k * 0.05
        rot = settle if k < 40 else 0.0
        slam.on_actuation(ActuationMsg(timestamp=t, joint_targets={}, heading_error=0.0, speed=0.0, state_str="CALIBRATE",
                                       odom_forward=0.0, odom_lateral=0.0, gyro_dyaw=bias * 0.05 + rot))
    assert abs(slam.gyro_bias - bias) < 1e-6, f"bias {math.degrees(slam.gyro_bias):.4f} deg/s (settling counted as bias)"
    # heading keeps the real -1.5 deg settling (plus the bias over those 2 s, 0.1 deg) and nothing of the settled part:
    # there the robot is still by definition and the gyro reads only bias (runs 10 and 11 carried that 30 s of bias,
    # +1.6 deg, as a heading error for the whole run)
    assert abs(math.degrees(slam.accum_odom_dyaw) - (-1.5 + 0.05 * 1.95)) < 0.02,         f"heading {math.degrees(slam.accum_odom_dyaw):.3f} deg after calibration (expected -1.40)"
    slam.accum_odom_dyaw = 0.0
    for k in range(1, 201):                    # 10 s walking straight: gyro reads only the bias
        slam.on_actuation(ActuationMsg(timestamp=t + k * 0.05, joint_targets={}, heading_error=0.0, speed=0.75,
                                       state_str="WALK", odom_forward=0.004, odom_lateral=0.0, gyro_dyaw=bias * 0.05))
    assert abs(math.degrees(slam.accum_odom_dyaw)) < 0.01, f"heading drifted {math.degrees(slam.accum_odom_dyaw):.3f} deg"
    print("PASSED")


def test_position_is_held_while_standing_still():
    """Rescue run 7: during 32 s of stand-still gyro calibration scan-matching jitter walked the estimate 15 cm. While
    the robot is known to be still, position is held (zero-velocity update)."""
    print("Running test_position_is_held_while_standing_still...", end=" ")
    from agents import SLAMAgent
    from agents.bus import ActuationMsg
    bus = MessageBus()
    slam = SLAMAgent(bus, initial_x=1.0, initial_y=2.0, initial_yaw=0.3)
    slam.initialize()
    rng = np.random.default_rng(0)
    wall = np.array([[2.0, y] for y in np.linspace(-1, 1, 60)] + [[x, 1.5] for x in np.linspace(0.5, 2, 40)])
    for k in range(200):
        t = k * 0.05
        slam.on_actuation(ActuationMsg(timestamp=t, joint_targets={}, heading_error=0.0, speed=0.0, state_str="CALIBRATE",
                                       odom_forward=0.0, odom_lateral=0.0, gyro_dyaw=0.0))
        slam.on_scan(ScanMsg(timestamp=t, points_2d=wall + rng.normal(0, 0.02, wall.shape)))
        slam.step(t)
    p = slam.pose_estimator.pose
    assert math.hypot(p.x - 1.0, p.y - 2.0) < 1e-9, f"position moved {math.hypot(p.x - 1.0, p.y - 2.0) * 100:.2f} cm while standing still"
    print("PASSED")


def test_tipped_robot_stops_and_map_ignores_camera():
    """A tipped robot (seen upside down in live run 11) stops its legs, reports it once, reports no motion, and mapping
    stops using its camera, which then faces the floor or ceiling."""
    print("Running test_tipped_robot_stops_and_map_ignores_camera...", end=" ")
    from agents import LocomotionAgent
    bus = MessageBus()
    events = []
    bus.subscribe("mission/event", lambda e: events.append(e.event_type))
    robot = _FakeRobot()
    loco = LocomotionAgent(bus, robot_driver=robot)
    mapper = MappingAgent(bus, -2.0, 2.0, -2.0, 2.0, 0.05)
    loco.initialize()
    mapper.initialize()
    loco.on_path(PathMsg(timestamp=0.0, waypoints=[(0.1 * i, 0.0) for i in range(1, 40)]))
    loco.on_pose(PoseMsg(timestamp=0.0, x=0.0, y=0.0, yaw=0.0))
    loco.step(0.0)
    assert loco.state_str == "WALK"
    robot.up = -0.9  # upside down
    for k in range(1, 6):
        loco.step(0.05 * k)
    assert loco.state_str == "TIPPED" and loco.speed == 0.0 and events.count("ROBOT_TIPPED") == 1
    assert bus.get_latest("control/actuation").odom_forward == 0.0
    before = mapper.grid.grid_log_odds.copy()
    pts = np.column_stack([np.linspace(0.5, 1.5, 200), np.zeros(200), np.full(200, 0.2)])
    mapper.on_pose(PoseMsg(timestamp=1.0, x=0.0, y=0.0, yaw=0.0))
    mapper.on_scan(ScanMsg(timestamp=1.0, points_2d=pts[:, :2], base_points_3d=pts))
    mapper.step(1.0)
    assert np.array_equal(before, mapper.grid.grid_log_odds), "map was updated from a tipped robot's camera"
    print("PASSED")


def test_doorway_given_up_after_repeated_failures():
    """Live run 11 drove into the same obstacle on the way to one doorway more than 20 times: doorway failures were never
    counted. After DOOR_MAX_FAILS stuck transits the doorway is skipped."""
    print("Running test_doorway_given_up_after_repeated_failures...", end=" ")
    from agents.bus import MissionEventMsg
    grid = _walled_room()
    grid.grid_log_odds[38:41, 20:23] = 0.0  # a frontier elsewhere, so the agent has an alternative
    labels = np.full((grid.height, grid.width), 4, dtype=np.int32)
    fa = FrontierAgent(MessageBus(), grid_ref=grid)
    fa.explored_rooms, fa.active_room_id = {4}, 4
    fa.on_grid(GridMapMsg(timestamp=0.0, log_odds=grid.grid_log_odds, free_cells=0, occupied_cells=0, unknown_cells=0,
                          explored_pct=0.0, room_labels=labels, doorways=[(0.3, 0.0)], doorway_rooms=[(4, 8)]))
    fa.on_pose(PoseMsg(timestamp=0.0, x=-0.9, y=0.0, yaw=0.0))  # transit target lands 0.55 m past the doorway, as live
    chosen = []
    for k in range(3):
        tgt = fa.select_frontier(current_time=float(k))
        chosen.append(bool(tgt and tgt.is_doorway))
        fa.on_mission_event(MissionEventMsg(timestamp=float(k), event_type="ROBOT_STUCK", details={"x": 0.0, "y": 0.0}))
    grid.grid_log_odds[38:41, 20:23] = -2.0
    assert chosen == [True, True, False], f"doorway choices after failures: {chosen}"
    print("PASSED")


def test_doorway_ahead_preferred_over_slightly_nearer_one_behind():
    """While travelling, a doorway ahead beats one behind that is only slightly nearer (the robot used to turn round
    for it); with no doorway ahead it still turns back, so it never gets stuck."""
    print("Running test_doorway_ahead_preferred_over_slightly_nearer_one_behind...", end=" ")
    grid = _walled_room()
    labels = np.full((grid.height, grid.width), 4, dtype=np.int32)
    for doors, expect_ahead in (([(1.2, 0.0), (-0.9, 0.0)], True), ([(-0.9, 0.0)], False)):
        fa = FrontierAgent(MessageBus(), grid_ref=grid)
        fa.explored_rooms = {4}
        fa.active_room_id = 4
        fa.on_grid(GridMapMsg(timestamp=0.0, log_odds=grid.grid_log_odds, free_cells=0, occupied_cells=0, unknown_cells=0,
                              explored_pct=0.0, room_labels=labels, doorways=doors,
                              doorway_rooms=[(4, 8 + i) for i in range(len(doors))]))
        fa.on_pose(PoseMsg(timestamp=0.0, x=0.0, y=0.0, yaw=0.0))  # travelling +x
        grid.grid_log_odds[38:41, 20:23] = 0.0  # keep a frontier so the agent does not report completion
        tgt = fa.select_frontier(current_time=1.0)
        grid.grid_log_odds[38:41, 20:23] = -2.0
        assert tgt is not None and tgt.is_doorway and (tgt.target_wx > 0) == expect_ahead, \
            f"doorways {doors}: chose ({tgt.target_wx:.2f}, {tgt.target_wy:.2f})"
    print("PASSED")


def test_open_floor_skips_doorway_transits():
    """Open floor (room_sweep off): no doorway transit, the frontier itself is the target (rescue run 14 crossed
    re-labelled 'rooms' and swept explored floor for 150 s)."""
    print("Running test_open_floor_skips_doorway_transits...", end=" ")
    grid = _walled_room()
    labels = np.full((grid.height, grid.width), 4, dtype=np.int32)
    for sweep in (True, False):
        fa = FrontierAgent(MessageBus(), grid_ref=grid)
        fa.room_sweep = sweep
        fa.explored_rooms, fa.active_room_id = {4}, 4
        fa.on_grid(GridMapMsg(timestamp=0.0, log_odds=grid.grid_log_odds, free_cells=0, occupied_cells=0, unknown_cells=0,
                              explored_pct=0.0, room_labels=labels, doorways=[(1.2, 0.0)], doorway_rooms=[(4, 8)]))
        fa.on_pose(PoseMsg(timestamp=0.0, x=0.0, y=0.0, yaw=0.0))
        grid.grid_log_odds[38:41, 20:23] = 0.0
        tgt = fa.select_frontier(current_time=1.0)
        grid.grid_log_odds[38:41, 20:23] = -2.0
        assert tgt is not None and tgt.is_doorway == sweep, f"room_sweep {sweep}: doorway {tgt and tgt.is_doorway}"
    print("PASSED")


def test_operator_route_is_followed_in_order_then_exploration_resumes():
    """The operator draws a search route (scene profile "route"): the robot heads for each waypoint in order (through
    the reachable cell nearest to it), moves on when within ROUTE_REACHED_M, skips one it cannot get closer to, then
    explores freely."""
    print("Running test_operator_route_is_followed_in_order_then_exploration_resumes...", end=" ")
    from agents.frontier_agent import ROUTE_STALL_S
    grid = _walled_room()
    grid.grid_log_odds[38:41, 20:23] = 0.0           # a frontier exists all along
    fa = FrontierAgent(MessageBus(), grid_ref=grid)
    fa.route = [(1.0, 1.0), (9.0, 9.0), (-1.0, 1.0)]  # the second is off the map: never reachable
    fa.on_pose(PoseMsg(timestamp=0.0, x=-1.0, y=-1.0, yaw=0.0))
    t1 = fa.select_frontier(current_time=1.0)
    assert fa.state == "ROUTE" and math.hypot(t1.target_wx - 1.0, t1.target_wy - 1.0) < 0.6, f"first target {t1}"
    fa.on_pose(PoseMsg(timestamp=2.0, x=1.0, y=1.0, yaw=0.0))
    t2 = fa.select_frontier(current_time=2.0)   # the off-map waypoint: the robot already stands as close as it can get
    assert fa.route_i == 2 and t2.target_wx < 0.0, "a waypoint it cannot approach was not skipped"
    fa.on_pose(PoseMsg(timestamp=70.0, x=-1.0, y=1.0, yaw=0.0))
    fa.select_frontier(current_time=70.0)
    assert fa.route_i == 3 and fa.state != "ROUTE", f"exploration did not resume ({fa.state})"
    grid.grid_log_odds[38:41, 20:23] = -2.0
    print("PASSED")


def test_route_waypoint_behind_an_obstacle_is_skipped_at_once():
    """The robot stands on the reachable cell nearest the waypoint and the waypoint is still behind rubble: skip it now
    instead of idling until the stall clock runs out (run 18 stood 60 s at waypoint 6)."""
    print("Running test_route_waypoint_behind_an_obstacle_is_skipped_at_once...", end=" ")
    grid = _walled_room()
    grid.grid_log_odds[11:70, 39:42] = 2.0            # full wall at x = 0: the east half cannot be reached
    fa = FrontierAgent(MessageBus(), grid_ref=grid)
    fa.route = [(1.0, -1.0), (-1.0, 1.0)]
    fa.on_pose(PoseMsg(timestamp=0.0, x=-1.0, y=-1.0, yaw=0.0))
    tgt = fa.select_frontier(current_time=1.0)
    fa.on_pose(PoseMsg(timestamp=10.0, x=tgt.target_wx, y=tgt.target_wy, yaw=0.0))
    fa.select_frontier(current_time=10.0)
    assert fa.route_i == 1, "idled at the nearest reachable spot of a blocked waypoint"
    print("PASSED")


def test_route_detour_round_a_wall_is_progress():
    """A waypoint behind a wall is reached by walking round it, first moving away from it in a straight line. Run 16
    skipped waypoints 25 and 26 that way (straight-line progress); walking distance is the measure."""
    print("Running test_route_detour_round_a_wall_is_progress...", end=" ")
    grid = _walled_room()
    grid.grid_log_odds[11:60, 39:42] = 2.0            # wall at x = 0 from the south wall up to y ~ 1.0
    fa = FrontierAgent(MessageBus(), grid_ref=grid)
    fa.route = [(0.6, -1.2)]
    for k, (x, y) in enumerate([(-0.6, -1.2), (-0.6, -0.5), (-0.6, 0.3), (-0.4, 1.2), (0.3, 1.2), (0.6, 0.4)]):
        fa.on_pose(PoseMsg(timestamp=40.0 * k, x=x, y=y, yaw=0.0))
        fa.select_frontier(current_time=40.0 * k)
        assert fa.route_i == 0, f"waypoint skipped at ({x}, {y}) while walking round the wall"
    print("PASSED")


def test_patrol_after_exploration_instead_of_idle():
    """Live run 7 stood idle from 1736 s to the 3000 s mission end: every completion exit returned no target. Now the
    robot patrols to the reachable place it has not seen for longest, keeps that target until reached, then moves on;
    exploration resumes by itself when a reachable frontier appears."""
    print("Running test_patrol_after_exploration_instead_of_idle...", end=" ")
    grid = _walled_room()  # fully explored, no frontier
    fa = FrontierAgent(MessageBus(), grid_ref=grid)
    for k, (x, y) in enumerate([(-1.2, -1.2), (-0.8, -1.2), (-1.2, -0.8)]):  # only the south-west was visited
        fa.on_pose(PoseMsg(timestamp=float(k), x=x, y=y, yaw=0.0))
        fa.visits.add("visit", x, y, t=float(k))
    first = fa.select_frontier(current_time=100.0)
    assert first is not None and fa.state == "MISSION_COMPLETE", "robot idles after exploration"
    assert first.target_wx > 0.0 or first.target_wy > 0.0, f"patrol chose a recently seen spot ({first.target_wx:.2f}, {first.target_wy:.2f})"
    again = fa.select_frontier(current_time=101.0)
    assert (again.target_gx, again.target_gy) == (first.target_gx, first.target_gy), "patrol target switched mid-route"
    fa.on_pose(PoseMsg(timestamp=102.0, x=first.target_wx, y=first.target_wy, yaw=0.0))
    nxt = fa.select_frontier(current_time=102.0)
    assert nxt is not None and (nxt.target_gx, nxt.target_gy) != (first.target_gx, first.target_gy), "no next patrol point"
    grid.grid_log_odds[38:41, 20:23] = 0.0  # a new unknown pocket appears: exploration must take over
    resumed = fa.select_frontier(current_time=103.0)
    assert fa.state != "MISSION_COMPLETE" and fa.patrol_target is None, f"exploration did not resume ({fa.state})"
    print("PASSED")


def test_patrol_walks_unwalked_floor_first():
    """Run 33: the floor under the table was seen from 0.73 m, so it counted as explored and was never walked. Patrol
    prefers floor the robot has never stood near over floor it walked long ago."""
    print("Running test_patrol_walks_unwalked_floor_first...", end=" ")
    grid = _walled_room()
    fa = FrontierAgent(MessageBus(), grid_ref=grid)
    hole = (0.4, 0.4)                                   # never stood within 0.35 m of here
    for x in np.arange(-1.4, 1.41, 0.2):                # the rest of the room walked densely, long ago
        for y in np.arange(-1.4, 1.41, 0.2):
            if math.hypot(x - hole[0], y - hole[1]) > 0.45:
                fa.visits.add("visit", float(x), float(y), t=0.0)
    for x, y in ((-0.1, 0.4), (0.9, 0.4), (0.4, -0.1), (0.4, 0.9)):   # walked past the hole recently
        fa.visits.add("visit", x, y, t=490.0)
    fa.on_pose(PoseMsg(timestamp=500.0, x=-1.2, y=-1.2, yaw=0.0))
    tgt = fa.select_frontier(current_time=500.0)
    assert tgt is not None and math.hypot(tgt.target_wx - hole[0], tgt.target_wy - hole[1]) < 0.35,         f"patrol chose walked floor ({tgt.target_wx:.2f}, {tgt.target_wy:.2f}) over the unwalked spot {hole}"
    print("PASSED")


def test_far_tiny_frontier_is_left_to_patrol_near_one_is_explored():
    """Run 34 crossed the map for 3-8 cell frontier slivers for its last 13 minutes (+1 % coverage). A tiny sliver far
    away is left to the floor-coverage patrol; the same sliver near the robot is still explored (small gaps matter)."""
    print("Running test_far_tiny_frontier_is_left_to_patrol_near_one_is_explored...", end=" ")
    def run(robot):
        grid = BayesianOccupancyGrid(-4.0, 4.0, -4.0, 4.0, 0.05)
        lo = grid.grid_log_odds
        lo[:] = 0.0
        lo[10:151, 10:151] = 2.0
        lo[11:150, 11:150] = -2.0                      # 7 x 7 m explored room
        hx, hy = grid.world_to_grid(3.0, 3.0)
        lo[hy, hx - 1:hx + 2] = 0.0                    # 1 x 3 unknown cells: a tiny frontier sliver at (3, 3)
        fa = FrontierAgent(MessageBus(), grid_ref=grid)
        fa.on_pose(PoseMsg(timestamp=1.0, x=robot[0], y=robot[1], yaw=0.0))
        fa.visits.add("visit", robot[0], robot[1], t=1.0)
        tgt = fa.select_frontier(current_time=100.0)
        return fa.state, tgt
    state_far, _ = run((-3.0, -3.0))
    assert state_far == "MISSION_COMPLETE", f"robot sent 8 m for a tiny sliver ({state_far})"
    state_near, tgt = run((2.0, 2.2))
    assert state_near != "MISSION_COMPLETE" and tgt is not None and math.hypot(tgt.target_wx - 3.0, tgt.target_wy - 3.0) < 0.6,         f"a nearby small gap was not explored ({state_near})"
    print("PASSED")


def test_frontier_at_the_map_edge_is_not_a_target():
    """Rescue run 5: the floor runs past the 11 x 11 m map window; frontiers on the window edge were chased and found
    unreachable twice. Unknown space beyond the map cannot be explored."""
    print("Running test_frontier_at_the_map_edge_is_not_a_target...", end=" ")
    grid = BayesianOccupancyGrid(-2.0, 2.0, -2.0, 2.0, 0.05)
    lo = grid.grid_log_odds
    lo[:] = -2.0                     # all free ...
    lo[:, 0:2] = 0.0                 # ... except a strip of unknown on the map's west edge
    fa = FrontierAgent(MessageBus(), grid_ref=grid)
    fa.on_pose(PoseMsg(timestamp=1.0, x=0.5, y=0.0, yaw=0.0))
    fa.visits.add("visit", 0.5, 0.0, t=1.0)
    tgt = fa.select_frontier(current_time=10.0)
    assert fa.state == "MISSION_COMPLETE" or (tgt is not None and tgt.target_wx > -1.7),         f"robot sent to the map edge ({tgt.target_wx:.2f}, {tgt.target_wy:.2f})"
    print("PASSED")


def test_compiled_wavefront_matches_python():
    """The numba geodesic wavefront and the Python loop give a bit-identical distance field."""
    print("Running test_compiled_wavefront_matches_python...", end=" ")
    import agents.frontier_agent as fam
    if fam._wavefront_kernel is None:
        print("SKIPPED (numba not installed)")
        return
    rng = np.random.default_rng(4)
    grid = BayesianOccupancyGrid(-3.0, 3.0, -3.0, 3.0, 0.05)
    lo = grid.grid_log_odds
    lo[:] = -2.0
    lo[rng.random(lo.shape) < 0.04] = 2.0          # scattered obstacles
    lo[:, 60] = 2.0; lo[50:56, 60] = -2.0          # a wall with a doorway
    fields = []
    for kernel in (fam._wavefront_kernel, None):
        saved, fam._wavefront_kernel = fam._wavefront_kernel, kernel
        try:
            fa = FrontierAgent(MessageBus(), grid_ref=grid)
            fields.append(fa._compute_geodesic_field(30, 40)[0].tobytes())
        finally:
            fam._wavefront_kernel = saved
    assert fields[0] == fields[1], "compiled wavefront diverged from the Python loop"
    print("PASSED")


def test_detour_is_progress_not_stuck():
    """Following a path around a wall moves away from the goal for a while; that is progress along the path, not a
    stall (16 of 17 'stuck' events in run 4 were this)."""
    print("Running test_detour_is_progress_not_stuck...", end=" ")
    from agents import LocomotionAgent
    bus = MessageBus()
    events = []
    bus.subscribe("mission/event", lambda e: events.append(e.event_type))
    loco = LocomotionAgent(bus)
    loco.initialize()
    # U-shaped detour: go 1 m away from the goal, across, then back; goal is 0.6 m from the start in a straight line
    path = [(0.0, 0.0), (0.0, -1.0), (0.6, -1.0), (0.6, 0.0)]
    loco.on_path(PathMsg(timestamp=0.0, waypoints=path, active_idx=0, replan_count=1))
    dense = [(x0 + (x1 - x0) * s / 20, y0 + (y1 - y0) * s / 20) for (x0, y0), (x1, y1) in zip(path, path[1:]) for s in range(20)]
    for k, (x, y) in enumerate(dense[:50]):  # 25 s at 2 Hz along the first ~1.5 m of the detour
        loco.on_pose(PoseMsg(timestamp=k * 0.5, x=x, y=y, yaw=0.0))
        loco.step(k * 0.5)
    assert "ROBOT_STUCK" not in events, "detour along the path was reported as a stall"
    print("PASSED")


def test_replanned_longer_route_to_the_same_goal_restarts_progress():
    """The planner replaces a short path by a long detour to the same goal (the short way closed in the map). The detour's
    remaining length cannot beat the short path's best for a long time; that is not a stall (rescue run 16: both
    stuck events, and the building gap abandoned after 15 s)."""
    print("Running test_replanned_longer_route_to_the_same_goal_restarts_progress...", end=" ")
    from agents import LocomotionAgent
    bus = MessageBus()
    events = []
    bus.subscribe("mission/event", lambda e: events.append(e.event_type))
    loco = LocomotionAgent(bus)
    loco.initialize()
    loco.on_path(PathMsg(timestamp=0.0, waypoints=[(0.0, 0.0), (0.6, 0.0)], active_idx=0, replan_count=1))
    loco.on_pose(PoseMsg(timestamp=0.0, x=0.0, y=0.0, yaw=0.0))
    loco.step(0.0)
    detour = [(0.0, 0.0), (0.0, -4.0), (0.6, -4.0), (0.6, 0.0)]   # 8.6 m to the same goal
    dense = [(x0 + (x1 - x0) * s / 80, y0 + (y1 - y0) * s / 80) for (x0, y0), (x1, y1) in zip(detour, detour[1:]) for s in range(80)]
    for k, (x, y) in enumerate(dense[:60]):   # 30 s at 2 Hz, 3 m along the detour, re-planned every 5 s as live
        if k % 10 == 0:
            loco.on_path(PathMsg(timestamp=k * 0.5, waypoints=[(x, y)] + detour[1:], active_idx=0, replan_count=2 + k))
        loco.on_pose(PoseMsg(timestamp=k * 0.5, x=x, y=y, yaw=0.0))
        loco.step(k * 0.5)
    assert "ROBOT_STUCK" not in events, "a longer re-planned route was reported as a stall"
    print("PASSED")


def test_spatial_memory_matches_brute_force():
    """Hashed lookups return exactly the items a full scan finds, for any radius and bucket size."""
    print("Running test_spatial_memory_matches_brute_force...", end=" ")
    from agents.spatial_memory import SpatialMemory
    rng = np.random.default_rng(2)
    pts = rng.uniform(-5, 5, (400, 2))
    for bucket in (0.3, 1.0, 4.0):
        mem = SpatialMemory(bucket=bucket)
        for i, (x, y) in enumerate(pts):
            mem.add("a" if i % 2 else "b", x, y, idx=i)
        for qx, qy, r in rng.uniform(-6, 6, (50, 3)):
            r = abs(r) / 2
            got = sorted(it["idx"] for it in mem.near("a", qx, qy, r))
            want = sorted(i for i, (x, y) in enumerate(pts) if i % 2 and (x - qx) ** 2 + (y - qy) ** 2 <= r * r)
            assert got == want
    print("PASSED")


def test_supervisor_doorways_by_place_with_real_rooms():
    """F29: the same doorways reported in a different order are not duplicated, and each records the rooms it joins
    (previously keyed by list index and always recorded as joining rooms 1 and 2)."""
    print("Running test_supervisor_doorways_by_place_with_real_rooms...", end=" ")
    sup = SupervisorAgent(MessageBus())
    sup.initialize()
    g = lambda dws, rooms: GridMapMsg(timestamp=0.0, log_odds=np.zeros((2, 2)), free_cells=0, occupied_cells=0,
                                      unknown_cells=0, explored_pct=0.0, doorways=dws, doorway_rooms=rooms)
    sup.on_grid(g([(1.0, 0.0), (-2.0, 3.0)], [(3, 4), (4, 7)]))
    sup.step(1.0)
    sup.on_grid(g([(-2.05, 3.02), (1.02, 0.01)], [(4, 7), (3, 4)]))  # same doorways, re-ordered, small jitter
    sup.step(2.0)
    kg = sup.knowledge_graph.doorways
    assert len(kg) == 2, f"{len(kg)} doorways for 2"
    assert sorted(d.connects_rooms for d in kg.values()) == [(3, 4), (4, 7)]
    print("PASSED")


def test_doorway_between_finished_rooms_is_skipped():
    """A doorway joining two explored rooms leads nowhere new; the robot does not walk through it (run 5 spent 600 s
    touring such doorways after coverage had stopped growing)."""
    print("Running test_doorway_between_finished_rooms_is_skipped...", end=" ")
    grid = _walled_room()
    labels = np.zeros((grid.height, grid.width), dtype=np.int32)
    labels[:, :40], labels[:, 40:] = 4, 6
    for rooms, expect_door in (((4, 6), False), ((4, 8), True)):
        fa = FrontierAgent(MessageBus(), grid_ref=grid)
        fa.explored_rooms = {4, 6}
        fa.active_room_id = 4
        fa.on_grid(GridMapMsg(timestamp=0.0, log_odds=grid.grid_log_odds, free_cells=0, occupied_cells=0, unknown_cells=0,
                              explored_pct=0.0, room_labels=labels, doorways=[(0.6, 0.0)], doorway_rooms=[rooms]))
        fa.on_pose(PoseMsg(timestamp=0.0, x=-0.8, y=0.0, yaw=0.0))
        grid.grid_log_odds[38:41, 20:23] = 0.0  # keep one frontier so the agent does not report completion
        tgt = fa.select_frontier(current_time=1.0)
        grid.grid_log_odds[38:41, 20:23] = -2.0
        assert bool(tgt is not None and tgt.is_doorway) == expect_door, f"doorway {rooms}: {tgt}"
    print("PASSED")


def test_relabelled_explored_floor_is_not_swept_again():
    """F3 part 2: after room 4 is finished, re-segmentation may give the same floor a new label (run 4 used 11 labels for
    7 real rooms); the robot remembers the floor by place and closes the new label without sweeping it."""
    print("Running test_relabelled_explored_floor_is_not_swept_again...", end=" ")
    grid = _walled_room()
    grid.grid_log_odds[38:41, 58:61] = 0.0  # frontier left in the east room (6); the west room (4) is fully seen
    labels = np.zeros((grid.height, grid.width), dtype=np.int32)
    labels[:, :40] = 4
    labels[:, 40:] = 6
    fa = FrontierAgent(MessageBus(), grid_ref=grid)
    msg = lambda lab: GridMapMsg(timestamp=0.0, log_odds=grid.grid_log_odds, free_cells=0, occupied_cells=0,
                                 unknown_cells=0, explored_pct=0.0, room_labels=lab, doorways=[])
    fa.on_grid(msg(labels))
    fa.on_pose(PoseMsg(timestamp=0.0, x=-0.6, y=0.0, yaw=0.0))
    fa.select_frontier(current_time=1.0)
    assert 4 in fa.explored_rooms, "room 4 has no frontier and should be finished"
    relabelled = labels.copy()
    relabelled[:, :40] = 9  # same west floor, new label
    fa.on_grid(msg(relabelled))
    fa.select_frontier(current_time=2.0)
    assert 9 in fa.explored_rooms and fa.active_room_id == 4, "re-labelled explored floor was entered as a new room"
    print("PASSED")


def test_target_is_kept_until_reached_while_progressing():
    """A chosen frontier is kept while the robot gets closer, even past the old 9 s window and while it faces another
    frontier (the old window was about 0.8 m of walking at 0.09 m/s, so targets were swapped mid-route)."""
    print("Running test_target_is_kept_until_reached_while_progressing...", end=" ")
    grid = _walled_room()
    grid.grid_log_odds[66:69, 66:69] = 0.0  # far pocket, north-east corner
    grid.grid_log_odds[13:16, 13:16] = 0.0  # pocket in the south-west corner
    fa = FrontierAgent(MessageBus(), grid_ref=grid)
    fa.on_pose(PoseMsg(timestamp=0.0, x=0.0, y=0.0, yaw=math.radians(45)))
    first = fa.select_frontier(current_time=0.0)
    start = (0.0, 0.0)
    for k in range(1, 30):  # 0.09 m/s toward the target, facing away from it (as when skirting an obstacle)
        a = k / 30.0
        x, y = start[0] + a * 0.7 * (first.target_wx - start[0]), start[1] + a * 0.7 * (first.target_wy - start[1])
        fa.on_pose(PoseMsg(timestamp=float(k), x=x, y=y, yaw=math.atan2(first.target_wy - y, first.target_wx - x) + math.pi))
        tgt = fa.select_frontier(current_time=float(k))
        assert (tgt.target_gx, tgt.target_gy) == (first.target_gx, first.target_gy), f"switched target at {k} s"
    print("PASSED")


def test_target_without_progress_is_dropped():
    """A target the robot does not get closer to (blocked, mis-mapped) is dropped after the stall time; two stalls
    retire it, so a different target follows."""
    print("Running test_target_without_progress_is_dropped...", end=" ")
    grid = _walled_room()
    grid.grid_log_odds[66:69, 66:69] = 0.0
    grid.grid_log_odds[13:16, 13:16] = 0.0
    fa = FrontierAgent(MessageBus(), grid_ref=grid)
    fa.on_pose(PoseMsg(timestamp=0.0, x=0.0, y=0.0, yaw=0.0))
    first = fa.select_frontier(current_time=0.0)
    for k in range(1, 60):  # the robot never moves
        tgt = fa.select_frontier(current_time=float(k))
        if (tgt.target_gx, tgt.target_gy) != (first.target_gx, first.target_gy):
            break
    assert 35 <= k <= 50, f"stalled target replaced after {k} s"
    print("PASSED")


def test_grid_world_round_trip():
    """Every cell maps to a world point that maps back to the same cell (the corner convention lost 22 % of cells)."""
    print("Running test_grid_world_round_trip...", end=" ")
    g = BayesianOccupancyGrid(-5.5, 5.5, -5.5, 5.5, 0.05)
    assert all(g.world_to_grid(*g.grid_to_world(x, y)) == (x, y) for x in range(g.width) for y in range(g.height))
    print("PASSED")


def test_compiled_raycast_matches_python():
    """The numba ray-casting kernel and the Python loop produce a bit-identical map over evolving scans."""
    print("Running test_compiled_raycast_matches_python...", end=" ")
    import core.grid as cg
    if cg._raycast_kernel is None:
        print("SKIPPED (numba not installed)")
        return
    grids = []
    for kernel in (cg._raycast_kernel, None):
        saved, cg._raycast_kernel = cg._raycast_kernel, kernel
        try:
            g = BayesianOccupancyGrid(-3.0, 3.0, -3.0, 3.0, 0.05)
            r = np.random.default_rng(5)
            for k in range(40):
                x, y = r.uniform(-1, 1), r.uniform(-1, 1)
                ang = r.uniform(0, 2 * np.pi, 2000)
                d = r.uniform(0.1, 3.5, 2000)
                pts = np.column_stack([x + d * np.cos(ang), y + d * np.sin(ang), r.uniform(0.0, 0.7, 2000)])
                g.update_from_depth_points(pts, (x, y), sample_rate=3)
            grids.append(g.grid_log_odds.tobytes())
        finally:
            cg._raycast_kernel = saved
    assert grids[0] == grids[1], "compiled kernel diverged from the Python loop"
    print("PASSED")


def test_near_frontier_behind_beats_far_one_ahead():
    """Turning round costs a few seconds, not metres: a frontier 0.8 m behind beats one 5 m ahead (the old heading
    bonus and reversal penalty were worth about 12 m of walking, so rescue run 8 walked the west strip end to end and
    back for the frontiers it had passed). A frontier only slightly further ahead still beats one just behind."""
    print("Running test_near_frontier_behind_beats_far_one_ahead...", end=" ")
    g = BayesianOccupancyGrid(-5.5, 5.5, -5.5, 5.5, 0.05)
    fa = FrontierAgent(MessageBus(), grid_ref=g)
    fa.grid = g

    def pick(ahead_m, behind_m):
        a, b = g.world_to_grid(ahead_m, 0.0), g.world_to_grid(-behind_m, 0.0)
        best = fa._score_frontiers([((a[0], a[1], 20), ahead_m), ((b[0], b[1], 20), behind_m)], 0.0, 0.0, 0.0)
        return "ahead" if (best[0], best[1]) == a else "behind"

    assert pick(5.0, 0.8) == "behind", "walked 5 m ahead instead of turning for a frontier 0.8 m behind"
    assert pick(1.3, 1.0) == "ahead", "turned round for a frontier only 0.3 m nearer"
    print("PASSED")


def test_floor_calibration_uses_net_progress_not_swaying_path():
    """The body sways sideways with every step, so its path is longer than its progress. Calibrating leg odometry on
    path length left it 7 % long on the rescue floor (net 0.931 vs path 0.995), and with nothing to scan-match in open
    floor rescue run 9 drifted 3 cm every 5 s. The calibration must return net progress / odometry."""
    print("Running test_floor_calibration_uses_net_progress_not_swaying_path...", end=" ")
    import os, tempfile
    from evaluation.floor_calibration import ratio
    n, dt = 2400, 0.05
    t = np.arange(n) * dt
    odo = np.full(n, 0.004)                      # odometry: 4 mm per step straight ahead
    x = np.cumsum(0.9 * odo)                     # true progress: 90 % of it
    y = 0.01 * np.sin(2 * np.pi * t / 0.5)       # 1 cm sideways sway, two steps a second
    gt = np.column_stack([x, y, np.zeros(n)])
    act = np.column_stack([odo, np.zeros(n), np.zeros(n), np.full(n, 0.75), np.ones(n)])
    path = os.path.join(tempfile.mkdtemp(), "rec.npz")
    np.savez(path, t=t, gt=gt, act=act)
    r, _ = ratio(path)
    assert abs(r - 0.9) < 0.01, f"calibration {r:.3f}, true net progress ratio 0.900"
    print("PASSED")


def test_frontier_is_viewed_from_a_standoff_not_from_the_drop():
    """The rescue floor ends in a drop with no wall: beyond its edge stays unknown for ever. Rescue run 9 picked
    frontier viewpoints 7 cm from the edge (one past it) while its position error was 30 cm. The robot must stand
    back from unknown space when it goes to look at it; the camera sees well past its 0.22 m blind zone."""
    print("Running test_frontier_is_viewed_from_a_standoff_not_from_the_drop...", end=" ")
    from agents.frontier_agent import STANDOFF_M
    grid = BayesianOccupancyGrid(-3.0, 3.0, -3.0, 3.0, 0.05)
    lo = grid.grid_log_odds
    lo[:] = 0.0                      # unknown ...
    lo[:, :80] = -2.0                # ... floor seen up to x = +1.0, nothing past it (the edge of the floor)
    fa = FrontierAgent(MessageBus(), grid_ref=grid)
    fa.on_pose(PoseMsg(timestamp=1.0, x=-2.0, y=0.0, yaw=0.0))
    tgt = fa.select_frontier(current_time=10.0)
    assert tgt is not None, "no target"
    edge_x = grid.grid_to_world(80, 0)[0] - 0.5 * grid.res
    assert edge_x - tgt.target_wx >= STANDOFF_M - grid.res,         f"viewpoint {edge_x - tgt.target_wx:.2f} m from unknown space (standoff {STANDOFF_M} m)"
    print("PASSED")


def test_sensor_gaps_in_seen_floor_do_not_block_viewpoints():
    """The depth camera leaves unknown stripes and dots inside floor it has seen. Measuring the standoff from every
    unknown cell left rescue run 15 without viewpoints for 76 % of its frontier cells; it declared the north side
    unreachable at 76 % coverage. Specks are ignored; the real unknown edge still keeps its standoff (and the thin
    strip past the floor edge at the map window border still counts as unknown)."""
    print("Running test_sensor_gaps_in_seen_floor_do_not_block_viewpoints...", end=" ")
    from agents.frontier_agent import STANDOFF_M
    grid = BayesianOccupancyGrid(-3.0, 3.0, -3.0, 3.0, 0.05)
    lo = grid.grid_log_odds
    lo[:] = 0.0
    lo[:, :80] = -2.0                     # floor seen up to x = +1.0, unexplored beyond
    lo[2::4, 2:78:4] = 0.0                # unknown single-cell sensor gaps every 0.2 m across the seen floor
    fa = FrontierAgent(MessageBus(), grid_ref=grid)
    fa.on_pose(PoseMsg(timestamp=1.0, x=-2.0, y=0.1, yaw=0.0))
    tgt = fa.select_frontier(current_time=10.0)
    assert tgt is not None and fa.state != "MISSION_COMPLETE", f"no viewpoint through sensor gaps ({fa.state})"
    edge_x = grid.grid_to_world(80, 0)[0] - 0.5 * grid.res
    assert edge_x - 1.5 <= tgt.target_wx <= edge_x - STANDOFF_M + grid.res,         f"target x {tgt.target_wx:.2f}: not a standoff viewpoint of the unexplored edge at x {edge_x:.2f}"
    print("PASSED")


def test_narrow_first_view_does_not_end_the_mission():
    """At the start the robot has seen only a narrow strip of floor. No cell of it is STANDOFF_M clear of unknown space,
    so no frontier has a viewpoint yet; that is not the end of exploration (rescue run 10 declared the mission
    complete in its first second, which also ends the wasted-walking measurement)."""
    print("Running test_narrow_first_view_does_not_end_the_mission...", end=" ")
    grid = BayesianOccupancyGrid(-3.0, 3.0, -3.0, 3.0, 0.05)
    lo = grid.grid_log_odds
    lo[:] = 0.0
    lo[55:65, 30:100] = -2.0         # a 0.5 m x 3.5 m strip seen, unknown all round (700 reachable cells)
    fa = FrontierAgent(MessageBus(), grid_ref=grid)
    fa.on_pose(PoseMsg(timestamp=1.0, x=-1.0, y=0.0, yaw=0.0))
    fa.select_frontier(current_time=1.0)
    assert fa.state != "MISSION_COMPLETE", "mission declared complete with most of the map unknown"
    print("PASSED")


def test_pose_follows_gyro_and_legs_while_the_camera_sees_nothing():
    """Facing open floor the scan can be empty. SLAM skipped those steps, so the pose froze while the robot turned;
    the controller kept turning on a stale heading and dithered on the spot for 60 s (rescue run 10, heading error
    swinging +/-100 deg). With no scan the pose must still dead-reckon from gyro and leg odometry."""
    print("Running test_pose_follows_gyro_and_legs_while_the_camera_sees_nothing...", end=" ")
    from agents import SLAMAgent
    from agents.bus import ActuationMsg
    bus = MessageBus()
    slam = SLAMAgent(bus, initial_x=0.0, initial_y=0.0, initial_yaw=0.0)
    slam.initialize()
    for k in range(40):                                  # 2 s turning at 45 deg/s and walking 4 mm per step
        t = k * 0.05
        slam.on_actuation(ActuationMsg(timestamp=t, joint_targets={}, heading_error=0.0, speed=0.75, state_str="WALK",
                                       odom_forward=0.004, odom_lateral=0.0, gyro_dyaw=math.radians(45.0) * 0.05))
        slam.on_scan(ScanMsg(timestamp=t, points_2d=np.empty((0, 2))))
        slam.step(t)
    p = bus.get_latest("pose/slam")
    assert p is not None, "no pose published while the camera saw nothing"
    assert abs(math.degrees(p.yaw) - 90.0) < 3.0, f"heading {math.degrees(p.yaw):.1f} deg, gyro says 90"
    assert math.hypot(p.x, p.y) > 0.10, f"position did not follow leg odometry ({math.hypot(p.x, p.y):.3f} m)"
    print("PASSED")


def test_failures_count_by_place_when_the_frontier_point_wanders():
    """In rubble the frontier point next to an unreachable gap moves 0.25-0.5 m every time the map updates. Failures
    were counted per 0.3 m grid bucket, so rescue run 12 tried five such points within 0.6 m of each other for 300 s
    and never retired the place. Two failures within FAIL_RADIUS_M retire the place."""
    print("Running test_failures_count_by_place_when_the_frontier_point_wanders...", end=" ")
    grid = BayesianOccupancyGrid(-3.0, 3.0, -3.0, 3.0, 0.05)
    fa = FrontierAgent(MessageBus(), grid_ref=grid)
    fa.grid = grid
    a = grid.world_to_grid(0.00, 1.60)
    b = grid.world_to_grid(0.25, 1.75)             # the same gap, its frontier point moved 0.29 m
    c = grid.world_to_grid(-0.20, 1.90)            # and moved again
    fa._strike(a, limit=2)
    fa._strike(b, limit=2)
    assert fa._is_blacklisted(*c), "third wandering point of a twice-failed place is still a target"
    far = grid.world_to_grid(1.5, 0.0)
    assert not fa._is_blacklisted(*far), "a place 1.5 m away was retired too"
    print("PASSED")


def test_target_sliding_along_the_floor_edge_keeps_the_standoff():
    """Rescue run 13 fell off the floor: as the camera cleared the edge frontier, the committed target slid along it
    (0.62, 7.82) -> (1.43, 7.88) -> ... within 5 cm of the drop, while the position error was 35 cm. The slide must use
    the same standoff viewpoints as a fresh choice."""
    print("Running test_target_sliding_along_the_floor_edge_keeps_the_standoff...", end=" ")
    from agents.frontier_agent import STANDOFF_M
    grid = BayesianOccupancyGrid(-3.0, 3.0, -3.0, 3.0, 0.05)
    lo = grid.grid_log_odds
    lo[:] = 0.0
    edge0 = grid.world_to_grid(0.0, 0.0)[0]
    lo[:, :edge0] = -2.0                           # floor seen up to x = 0
    fa = FrontierAgent(MessageBus(), grid_ref=grid)
    fa.on_pose(PoseMsg(timestamp=0.0, x=-2.0, y=0.0, yaw=0.0))
    first = fa.select_frontier(current_time=0.0)
    assert first is not None
    edge1 = grid.world_to_grid(0.25, 0.0)[0]
    lo[:, :edge1] = -2.0                           # the camera sees 0.25 m further: the frontier moved on
    fa.on_pose(PoseMsg(timestamp=1.0, x=-1.8, y=0.0, yaw=0.0))
    tgt = fa.select_frontier(current_time=1.0)
    edge_x = grid.grid_to_world(edge1, 0)[0] - 0.5 * grid.res
    assert edge_x - tgt.target_wx >= STANDOFF_M - grid.res,         f"target {edge_x - tgt.target_wx:.2f} m from unknown space after the frontier moved (standoff {STANDOFF_M} m)"
    print("PASSED")


def test_drop_ahead_is_seen_and_walled_off():
    """Rescue run 13 walked off the unwalled floor edge with 35 cm of position error: nothing but the map kept it on
    the floor. The camera sees the floor from ~0.19 m (points at z ~ 0.01 in the body frame): where the floor ends
    within the window ahead and nothing stands there, a drop is ahead and a virtual wall goes into the obstacle points
    (bumper, map and planner then keep away), never into the scan used for localization."""
    print("Running test_drop_ahead_is_seen_and_walled_off...", end=" ")
    from agents.perception_agent import drop_ahead
    xs, ys = np.meshgrid(np.arange(0.20, 1.0, 0.01), np.arange(-0.4, 0.4, 0.01))
    floor = np.column_stack([xs.ravel(), ys.ravel(), np.full(xs.size, 0.012)])
    assert drop_ahead(floor) is None, "open floor read as a drop"
    edge = floor[floor[:, 0] < 0.33]                      # the floor ends 0.33 m ahead
    d = drop_ahead(edge)
    assert d is not None and 0.30 <= d <= 0.36, f"drop at 0.33 m not found ({d})"
    block = np.vstack([edge, [[0.30, y, 0.10] for y in np.arange(-0.1, 0.11, 0.02)]])   # a block hides the floor
    assert drop_ahead(block) is None, "a block in front read as a drop"
    far_edge = floor[floor[:, 0] < 0.70]                  # floor ends beyond the window: not yet a drop
    assert drop_ahead(far_edge) is None, "a drop 0.7 m away stopped the robot"

    # In the perception pipeline: virtual wall in obstacle_2d and the map points, not in the localization scan
    import agents.perception_agent as pa
    bus = MessageBus()
    agent = pa.PerceptionAgent(bus, camera_matrix={"T_base_cam": [0.0] * 12})
    saved = pa.depth_to_base_points
    pa.depth_to_base_points = lambda *a, **k: edge
    try:
        agent.process_frame(np.ones((4, 4), np.float32), None, 1.0)
    finally:
        pa.depth_to_base_points = saved
    scan = bus.get_latest("sensor/scan")
    assert scan is not None and len(scan.obstacle_2d) > 0, "no virtual wall for the bumper"
    assert np.all(np.abs(scan.obstacle_2d[:, 0] - d) < 0.10), "virtual wall not at the drop"
    assert len(scan.points_2d) == 0, "virtual wall leaked into the localization scan"
    assert (scan.base_points_3d[:, 2] >= 0.04).any(), "virtual wall missing from the map points"
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
    test_planner_avoids_unknown_space()
    test_locomotion_agent()
    test_frontier_boundary_room_association()
    test_semantic_entity_extraction()
    test_frontier_behind_wall_is_ignored()
    test_unobservable_frontier_is_retired_not_revisited()
    test_unfinished_room_is_not_abandoned_on_label_change()
    test_patrol_after_exploration_instead_of_idle()
    test_patrol_walks_unwalked_floor_first()
    test_frontier_at_the_map_edge_is_not_a_target()
    test_far_tiny_frontier_is_left_to_patrol_near_one_is_explored()
    test_blocked_legs_are_felt_and_phantom_odometry_stops()
    test_doorway_given_up_after_repeated_failures()
    test_tipped_robot_stops_and_map_ignores_camera()
    test_position_is_held_while_standing_still()
    test_gyro_bias_is_measured_while_standing_still()
    test_stall_threshold_follows_the_floor_calibration()
    test_narrow_gap_opens_only_with_the_narrow_setting()
    test_tabletop_is_scan_structure_but_not_an_obstacle()
    test_planner_escapes_a_ring_of_clearance_margins()
    test_planner_keeps_path_unless_clearly_shorter()
    test_contact_obstacle_stays_in_map()
    test_recovery_turns_away_and_never_reverses()
    test_turn_walk_dithering_is_not_a_stall()
    test_doorway_ahead_preferred_over_slightly_nearer_one_behind()
    test_compiled_wavefront_matches_python()
    test_detour_is_progress_not_stuck()
    test_spatial_memory_matches_brute_force()
    test_supervisor_doorways_by_place_with_real_rooms()
    test_relabelled_explored_floor_is_not_swept_again()
    test_doorway_between_finished_rooms_is_skipped()
    test_target_is_kept_until_reached_while_progressing()
    test_target_without_progress_is_dropped()
    test_grid_world_round_trip()
    test_compiled_raycast_matches_python()
    test_near_frontier_behind_beats_far_one_ahead()
    test_floor_calibration_uses_net_progress_not_swaying_path()
    test_frontier_is_viewed_from_a_standoff_not_from_the_drop()
    test_narrow_first_view_does_not_end_the_mission()
    test_pose_follows_gyro_and_legs_while_the_camera_sees_nothing()
    test_failures_count_by_place_when_the_frontier_point_wanders()
    test_target_sliding_along_the_floor_edge_keeps_the_standoff()
    test_drop_ahead_is_seen_and_walled_off()
    print("=== ALL TESTS PASSED SUCCESSFULLY! ===")


