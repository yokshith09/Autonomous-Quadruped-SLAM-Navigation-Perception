# Deferred task (set 2026-11-14, do in a few days): rescue scene full route

Requested by the user on 2026-11-14; deferred for lack of time. Drawing: docs/results/rescue_requested_route_2026-11-14.png
(cyan line over the run 19 map).

## What is asked
1. Follow the drawn route: east half, the north-east rubble, round both sides of the central rubble pile,
   through the gap in the damaged building, and the loops south and south-east of the start.
2. Pass through the building gap and narrow gaps between objects.
3. Explore every reachable part of the scene; no unexplored floor left.
4. Position error (ATE) below 11 cm; all accuracy figures above 95 %.
5. 0 stuck events and 0 collisions.

## Status at deferral (run 19, 20261114_171629, 12 min)
- Not met. Route waypoint 14 of 28 reached; only the west and south-west were covered (59 % of the floor).
- Building gap not passed (run 14 crossed it once, older code).
- ATE 12.3 cm, precision 95.8 %, recall 38.8 %, F1 55.2 %, 1 stuck (recovered), 0 collisions.

## Starting points
- Trace the new waypoints from the drawing into `route` in config/scenes/searchrescuescene.json (current 28 waypoints follow the earlier drawing).
- Full-length run (25-30 min) needed for coverage; watch ATE growth late in the run (9 cm at 8 min -> 12 cm at 12 min).
- Map the building gap: check the map's walls do not close it (inflation / ATE).
