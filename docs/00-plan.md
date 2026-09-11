# Project plan

The full build plan for the F450 / Pixhawk 2.4.8 obstacle-avoidance system.
Kept out of the README so that stays a short description.

## The whole idea

The Pixhawk already knows how to fly and follow a path. We are not rebuilding
that. We are adding a small computer with a camera that sits on top and says
*"something solid, 3 metres ahead, slightly left."*

ArduPilot already knows what to do with that message. **The entire project is
producing that one message accurately.**

- **Flight stack:** ArduPilot Copter (not PX4 -- PX4 dropped support for this board)
- **Flight controller:** Pixhawk 2.4.8
- **Airframe:** F450, ~400-500 g spare payload
- **Sensing:** cameras only. No lidar, no sonar, no rangefinders.
- **Approach:** everything proven in simulation first

## Steps

Each step has a gate. Don't move on until it passes.

- [x] **1. Fix the PC.** Convert WSL1 to WSL2. **Done.**
      *Gate: `wsl -l -v` reports version 2.*

- [x] **2. Fake drone flying.** ArduPilot SITL + Gazebo. **Done 2026-09-11.**
      *Gate: a simulated drone flies 4 waypoints you drew in Mission Planner.*
      Gazebo Harmonic 8.15 with the ArduPilot plugin, ArduCopter 4.8.0-dev in
      SITL. Three traps cost an afternoon and are written up in
      [03-gazebo.md](03-gazebo.md): `FRAME_CLASS` was 0 so it would not arm, a
      disarmed multirotor does not move, and `PRX1_TYPE = 2` blocks arming until
      proximity data is flowing.

- [x] **3. Lie to the fake drone.** **PASSED 2026-09-11.**
      *Gate: the simulated drone curves around an obstacle that doesn't exist,
      then rejoins its path.*
      A straight 70 m AUTO mission deviated **8.34 m** around a virtual obstacle
      35 m north, slowed to 1.1 m/s abeam of it, rejoined the track and reached
      the waypoint. Detection began with the obstacle surface 17.5 m away,
      inside the 20 m sensor range, exactly as predicted. Cruise held 2.0 m/s.

      Use **`src/virtual_obstacle_publisher.py`** for this, not
      `fake_obstacle_publisher.py` -- the latter fixes the wall to the body
      frame, so it turns with the aircraft and can never be flown around. It is
      still the right tool for proving the plumbing works.

      **This was the most important step in the project.** The hard part is done;
      everything after is swapping fake numbers for real ones.

- [ ] **4. Fake camera.** Add a depth camera to the simulator and feed its output
      through `src/depth_to_obstacle_ring.py`, which emits the same message.
      The conversion is written and unit-tested; what remains is wiring it to a
      real depth source.
      *Gate: avoids a simulated wall it actually "saw", 10 runs, zero collisions.*

- [ ] **5. Buy hardware, test on the desk.** Raspberry Pi 5 + OAK-D Lite. No drone.
      *Gate: camera distance readings match a tape measure at 1, 2 and 4 m.*

- [ ] **6. Add the AI recognition layer.** YOLO on the OAK-D. Logging only, no steering.
      *Gate: walk in front of it, see "person, 2.4 m" in the logs at 15+ Hz.*

- [ ] **7. Bolt it on, props OFF.** Pi powered from its own BEC, never the Pixhawk 5V rail.
      *Gate: Mission Planner's proximity display matches reality.*

- [ ] **8. Fly it.** Low, slow, empty field. Hover first, then one waypoint mission.
      *Gate: avoids something real and finishes the mission.*

- [ ] **9. Only then, make it faster.**

**Do not buy anything until Step 4 passes.** If you get stuck before then you've
lost time, not money.

## Shopping list (Step 5)

| Thing | Why | Cost |
|---|---|---|
| Raspberry Pi 5 (4 GB) | The little brain that talks to the Pixhawk | ~$60 |
| OAK-D Lite camera | Two cameras + an AI chip. Does the seeing and the thinking | ~$150 |
| 5V/5A UBEC | Powers the Pi safely | ~$15 |
| Cables, mount, heatsink | Bits and pieces | ~$40 |

~$265. Weight ~200 g, inside the F450's margin.

## Rules that keep the drone in one piece

1. **Cap speed at 2 m/s.** At 2 m/s you need ~1.8 m of warning. At 5 m/s you'd
   need ~8 m, which is past where this camera is trustworthy. This single number
   is the difference between a test platform and a pile of broken arms.
2. **Never power the Pi from the Pixhawk's 5V rail.** Separate BEC, always.
3. **Check vibration before blaming the camera.** Pull a log, look at `VIBE`,
   want under ~30 m/s^2. Bad vibration wrecks the position estimate and makes
   perfectly good avoidance look broken.
4. **It fails unsafe.** If the Pi crashes, ArduPilot discards the stale obstacle
   data and *keeps flying the mission with no avoidance at all*. Mitigate with a
   companion heartbeat, a tight geofence, and the low speed cap. Test this
   deliberately in SITL by killing the script mid-mission.

## What cameras genuinely cannot do

Physics, not model quality. Design your test flights around these:

- **Thin things** -- wires, branches, chain-link, antennas. Stereo needs texture
  across a region; a 5 mm wire against sky gives it nothing. This is the number
  one cause of vision-guided drone crashes.
- **Blank surfaces** -- white walls, calm water, snow, uniform overcast sky.
- **Direct sun and glare.**
- **Fast-moving obstacles** -- birds, other drones. The pipeline assumes a
  mostly static world.

## Layout

```
Project-Alpha/
├── README.md                        this file
├── requirements.txt
├── docs/
│   ├── 01-setup.md                  Steps 1-2: WSL2, ArduPilot SITL, Gazebo
│   └── 02-ardupilot-params.md       the parameters that turn avoidance on
└── src/
    ├── check_avoidance_support.py   run first: does this firmware support it?
    ├── fake_obstacle_publisher.py   Step 3
    ├── obstacle_ring.py             shared ring format + MAVLink send
    ├── depth_to_obstacle_ring.py    Step 4: depth image -> 72 sectors
    └── test_depth_to_ring.py        self-tests, no hardware needed
```

## Trying it now

The depth pipeline runs against synthetic scenes, so it can be developed and
validated long before any hardware exists:

```
python src/test_depth_to_ring.py                 # 15 self-tests
python src/depth_to_obstacle_ring.py --demo both # a wall and a floor
python src/depth_to_obstacle_ring.py --demo ground --pitch -20 --no-compensate
```

That last one is worth running. It shows what happens when attitude is ignored:
the ground turns into a wall closing in at two metres, which is what an aircraft
would brake for every time it tilted to accelerate.

## Timeline

Steps 1-4 (all on the PC): 3-5 weeks of evenings.
Steps 5-8 (hardware): another 4-6 weeks.
