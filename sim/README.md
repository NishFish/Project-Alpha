# Browser simulator

`ring_simulator.html` -- open it directly in a browser, no build step.

A 2D top-down model of the same conversion `src/depth_to_obstacle_ring.py`
performs: rays into 72 sectors, sectors into an avoidance decision. The controls
map onto real ArduPilot parameters, so the failure modes it demonstrates are the
ones the aircraft will actually meet.

Worth trying:

  * Turn off the 10th percentile with noise on -- one bad ray per sector is
    enough to trigger braking. This is why the Python takes a percentile.
  * Turn off yaw-into-travel -- it sidesteps into sectors it has never observed
    and the collision counter climbs. That is the argument for WP_YAW_BEHAVIOR.
  * Raise the speed cap to 5 m/s -- the stopping-distance circle grows past the
    field of view. Once that happens the geometry has decided the outcome and no
    amount of software helps.

What it does NOT model: it is 2D, so ground-plane rejection and pitch
compensation do not appear here. Those are covered by
`src/test_depth_to_ring.py`. The avoidance is a simplified stand-in for
BendyRuler, not ArduPilot's implementation. Gazebo remains the real proving
ground.

Also published as an artifact:
https://claude.ai/code/artifact/73e104c8-bc6a-4a76-8dd4-f2d4193a4194
