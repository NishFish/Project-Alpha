# Step 4 -- Gazebo

Two levels here, and they are worth keeping separate. Level A gets a drone
flying in a 3D world and is a short afternoon. Level B feeds a simulated depth
camera into `src/depth_to_obstacle_ring.py` and is the heaviest setup in the
whole project.

Do Level A first and confirm it flies before attempting Level B.

## Before anything: WSL2 has to start

**Resolved 2026-09-10** by the `.wslconfig` in step 2 below; kept here because
the failure is easy to hit again after a Windows update. The symptom was
`wsl -d Ubuntu` failing with:

```
Insufficient system resources exist to complete the requested service.
Error code: Wsl/Service/CreateInstance/CreateVm/HCS/0x800705aa
```

The machine has 16 GB with roughly 3 GB free -- Chrome, Steam and Discord hold
most of the rest. The pagefile is healthy (22 GB allocated, 2 GB used), so this
is available memory, not swap.

Fixes, cheapest first:

1. Close Chrome, Steam and Discord, then `wsl --shutdown` and retry.
2. Cap what WSL2 asks for by creating `C:\Users\gtvic\.wslconfig`:

   ```
   [wsl2]
   memory=6GB
   processors=4
   swap=8GB
   ```

   By default WSL2 requests up to half the system RAM. Naming a smaller figure
   often clears this error outright.
3. Reboot. Stale Hyper-V state causes this too.

Gazebo also needs **OpenGL hardware acceleration**. WSLg passes the RTX 3060
through, so this should work, but software rendering makes Gazebo unusable --
if it is crawling, that is the first thing to check.

## Everything below runs INSIDE Ubuntu

Not PowerShell. `gz`, `sim_vehicle.py`, `ros2` and `colcon` are Linux programs
living in the WSL distro. Windows has never heard of them, so running them in
PowerShell gives `The term 'gz' is not recognized` -- which looks like a broken
install but is only the wrong shell.

Open an Ubuntu shell first, by any of:

  * Start menu -> **Ubuntu**
  * Windows Terminal -> the dropdown beside the `+` tab -> **Ubuntu**
  * `wsl -d Ubuntu` from PowerShell

You want **two** of them side by side. The prompt tells you which you are in:
`PS C:\Users\gtvic>` is PowerShell, `nishanth@...:~$` is Ubuntu.

## Level A -- SITL and Gazebo, no ROS

Install Gazebo Harmonic (ArduPilot's recommended pairing on Ubuntu 22.04),
then:

```bash
sudo apt update
sudo apt install libgz-sim8-dev rapidjson-dev
sudo apt install libopencv-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
                 gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl
```

Build ArduPilot's Gazebo plugin:

```bash
mkdir -p ~/gz_ws/src && cd ~/gz_ws/src
git clone https://github.com/ArduPilot/ardupilot_gazebo
export GZ_VERSION=harmonic
cd ardupilot_gazebo && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j4
```

Put these in `~/.bashrc` so every new terminal has them:

```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/gz_ws/src/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
export GZ_SIM_RESOURCE_PATH=$HOME/gz_ws/src/ardupilot_gazebo/models:$HOME/gz_ws/src/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH
```

Run it in two **Ubuntu** terminals:

```bash
# terminal 1 -- the simulator. A Gazebo window opens on your Windows
# desktop through WSLg. Leave it running.
gz sim -v4 -r iris_runway.sdf
```

```bash
# terminal 2 -- the autopilot, talking to that simulator
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console \
               --out=udp:127.0.0.1:14551
```

Start Gazebo first and give it a few seconds. The other way round, SITL finds
no simulator and sits waiting.

Keep the `--out` flag -- it is the port our scripts listen on.

**Gate: the quadcopter takes off in Gazebo and flies a mission you drew in the
ground station.**

At this point `src/fake_obstacle_publisher.py` also works against it, which is
Step 3 running in a real 3D simulator rather than bare SITL.

## Level B -- a depth camera into our pipeline

The awkward part: Gazebo publishes camera images on its own transport, and
there is no comfortable way to read those from Python directly. The supported
route is ROS 2, which bridges Gazebo topics into `sensor_msgs/Image`.

ArduPilot documents this pairing. Note that ROS 2 Humble ships against Gazebo
Fortress, so Humble plus Harmonic is a non-default combination and `ros_gz`
must be built from source with `GZ_VERSION` set. That is expected, not a
mistake -- follow <https://ardupilot.org/dev/docs/ros2-gazebo.html>, which
covers adding the Gazebo apt sources to rosdep.

The shape of it:

```bash
export GZ_VERSION=harmonic
# clone ardupilot_gz with vcstool, then:
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to ardupilot_gz_bringup
ros2 launch ardupilot_gz_bringup iris_runway.launch.py
```

Then add a depth camera sensor to the vehicle model and run our bridge:

```bash
python3 src/gz_depth_bridge.py --topic /depth_camera --connect udpin:127.0.0.1:14551
```

**Gate: the aircraft avoids a wall in Gazebo that it detected through the depth
camera, ten runs, zero collisions.**

## Where our code plugs in

Nothing in `depth_to_obstacle_ring.py` changes between Gazebo and the real
OAK-D. Both produce a depth array in metres; both call the same `process()`.
Only the source differs:

```
Gazebo depth camera  --\
                        >-- DepthToRing.process() --> OBSTACLE_DISTANCE --> ArduPilot
OAK-D Lite stereo    --/
```

That is the whole reason the conversion was written and tested standalone
first.

## If Level B proves too heavy

A legitimate shortcut: give the Gazebo model a `gpu_lidar` sensor instead of a
depth camera. A 2D scan maps onto the 72 sectors almost directly, needs no ROS,
and still exercises ArduPilot's avoidance end to end. You lose the camera part
of the pipeline in simulation, but you keep it on the real aircraft where it
matters. Worth taking if ROS 2 setup starts eating weeks.

## Troubleshooting: "nothing moves in Gazebo"

Both of these cost an afternoon on 2026-09-11. Neither is a Gazebo fault.

### The clock is running but the drone sits there

First establish whether Gazebo is actually paused:

```bash
gz topic -e -t /world/iris_runway/clock -n 2
```

If `sim` seconds advance, Gazebo is fine and the problem is the autopilot. A
disarmed multirotor does not move -- that is correct behaviour, not a fault.
Check what the vehicle thinks:

```
mode      : STABILIZE
ARMED     : False        <- nothing will move
```

A takeoff command in `STABILIZE` is ignored outright. The vehicle has to be in
`GUIDED`, then armed, then commanded.

### PreArm: Motors: Check frame class and type

`FRAME_CLASS = 0` means the airframe is undefined, and ArduPilot refuses to
arm. Set `FRAME_CLASS = 1` (quad) and `FRAME_TYPE = 1` (X), then **reboot the
autopilot** -- `FRAME_CLASS` is not applied until restart.

### PreArm: PRX1: No Data

This one is a consequence of doing the right thing. Once `PRX1_TYPE = 2`,
ArduPilot expects proximity data and will not arm without it. So the ordering
is:

1. Start `fake_obstacle_publisher.py`
2. *Then* arm

Not the other way round. Worth knowing now, because the real aircraft behaves
identically -- the companion computer must be alive and publishing before the
aircraft will arm.

### Two clients cannot share UDP 14551

The publisher binds `udpin:127.0.0.1:14551`. Anything else wanting a MAVLink
link needs a different endpoint. SITL exposes spare TCP ports for exactly this:

```
tcp:127.0.0.1:5762
tcp:127.0.0.1:5763
```

Those ports stream **nothing** until a client asks, so a connection there
reporting `nan` altitude is not a grounded aircraft -- it is a client that never
requested data:

```python
m.mav.request_data_stream_send(m.target_system, m.target_component,
                               mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)
```

### Confirm from Gazebo's side, not the autopilot's

The definitive check, since with `--model JSON` the physics come from Gazebo:

```bash
gz topic -e -t /world/iris_runway/dynamic_pose/info -n 1 | grep -A12 'name: "iris'
```

A `z` of 10.19 means the model really is ten metres up.

## The obstacle course

`sim/gazebo/course.txt` defines the obstacles once, in ArduPilot NED metres
relative to home. Two things read it, so they cannot disagree:

* `sim/gazebo/make_world.py` bakes them into a world file as visible pillars
* `sim/run_course.py` publishes what a forward sensor would see of them

```bash
python3 sim/gazebo/make_world.py
gz sim -v4 -r sim/gazebo/obstacle_course.sdf     # terminal 1
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --no-mavproxy   # terminal 2
python3 sim/run_course.py                                                # terminal 3
```

### Put obstacles in the world, not into a live one

`spawn_course.sh` adds obstacles to an already-running world through Gazebo's
`create` service, and it works -- but spawning a static 14 m pillar around a
hovering aircraft shoves it out of the sky. That is exactly what happened on the
first attempt: the iris was parked at 70 m north, a pillar appeared at 70 m
north, and the aircraft was knocked sideways and landed. Use `make_world.py`
so the pillars exist before anything takes off.

### Gazebo is ENU, ArduPilot is NED

Measured, not assumed: with the aircraft at ArduPilot `north 69.9, east 0.0`,
Gazebo reported `x -0.03, y 69.88`. So **Gazebo x is east and Gazebo y is
north**, while `LOCAL_POSITION_NED` is x=north, y=east. The two are swapped, and
getting it wrong puts the pillars at right angles to where the aircraft believes
they are. `make_world.py` does the swap.

### Environment variables and non-interactive shells

The `GZ_SIM_RESOURCE_PATH` exports live in `~/.bashrc`, which a non-interactive
shell does not read, so launching Gazebo from a script fails with
`Unable to find uri[model://runway]`. Export them explicitly in any script.

### MAVProxy cannot run detached

`sim_vehicle.py` launched in the background reports `MAVProxy exited` and then
kills everything, because MAVProxy needs a terminal. Use `--no-mavproxy` for
scripted runs and talk to SITL directly on `tcp:127.0.0.1:5760`. You lose the
map and console, and the automatic forward to the Windows host on 14550, which
is how Mission Planner was connecting -- point it at `<wsl-ip>:5760` over TCP if
you want it back.

### Parameters are per working directory

`sim_vehicle.py` keeps `eeprom.bin` in whatever directory you launch it from, so
starting it somewhere new gives you **default parameters** -- `FRAME_CLASS` back
to 0, avoidance off. Launch from the same directory every time, or re-apply the
parameters after moving.

### Arming right after a reboot

The first attempt is normally refused with `Accels inconsistent` or
`GPS 1 still configuring this GPS`. The EKF is still aligning and the GPS driver
still probing; both clear within a minute. `run_course.py` retries for
`--arm-timeout` seconds rather than treating the first refusal as fatal.

## One command for all of it

```bash
bash sim/run_all.sh
```

Stops anything running, regenerates the world from `course.txt`, starts Gazebo,
starts SITL, applies the parameters, and flies the course. About four minutes,
most of it the flight. Gazebo and SITL are left running afterwards.

Every step in that script exists because of something that broke without it:
explicit `GZ_SIM_*` exports, `--no-mavproxy`, a fixed working directory for
`eeprom.bin`, waiting on the Gazebo clock and on port 5760 rather than sleeping
a guessed interval, and re-applying parameters on every startup.

### The route is not identical run to run

Two runs of the same course, same parameters:

| | run 1 | run 2 |
|---|---|---|
| max lateral deviation | 5.26 m | 11.38 m |
| clearance, obs_6 | 5.60 m | 14.69 m |
| worst clearance | 2.97 m | 3.06 m |

Run 1 threaded the gate at 95 m; run 2 went around the west side of it
entirely. Both are valid, both cleared every pillar by more than the 2 m margin.
BendyRuler searches from the current heading and state, so small timing
differences pick a different branch.

Which means **a single run proves very little.** Judge a change by worst
clearance over many runs, not by whether one flight looked tidy.

## Real detection, no predetermined positions

`make_world.py` bolts a forward-facing depth camera to the airframe -- 72
degrees, 320x240, 0.2-20 m at 15 Hz, matched to the OAK-D Lite. Then:

```bash
bash sim/run_all.sh --depth
# or directly:
python3 sim/run_course.py --source depth --min-range 1.5
```

With `--source depth` the flight loop is told nothing about where the pillars
are. It subscribes to the Gazebo depth image, runs it through
`src/depth_to_obstacle_ring.py` -- the same file that will run on the real
OAK-D -- and publishes the result. `course.txt` is still read, but only to score
the run afterwards.

**Result, 2026-09-11:** all seven pillars cleared, worst clearance 2.82 m
against a 2.00 m margin, max lateral deviation 4.97 m, waypoint reached.

### Gazebo's Python bindings remove the need for ROS

`python3-gz-transport13` and `python3-gz-msgs10` arrive with `gz-harmonic` and
are importable straight away:

```python
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image
node = Node()
node.subscribe(Image, "/depth_camera", callback)
```

Depth frames arrive as `float32` metres, one per pixel, with `+inf` for anything
beyond the far clip. This is far simpler than the ROS 2 bridge route described
earlier in this document, and it is what `run_course.py --source depth` uses.

### The camera is inlined into the model, not wrapped around it

`make_world.py` splices the depth camera into `iris_with_gimbal`'s own model
block rather than wrapping that model inside another. The ArduPilot plugin lives
in that model and refers to its links by relative name
(`iris_with_standoffs::rotor_0`), so adding a nesting level risks breaking
references that currently work. Inlining keeps the model name, nesting depth and
every internal reference identical and adds exactly one link and one joint.

Orientation: the world includes the drone with a 90 degree yaw and it flies
toward world +Y at ArduPilot yaw 0, so the model's +X is the nose. Gazebo
cameras look along their own +X, so the sensor needs no rotation.

### The ground is an obstacle when you are on the ground

The first `--source depth` attempt would not arm:

```
PreArm: Proximity 355 deg, 0.55m (want > 0.6m)
```

The camera sits 0.22 m above the runway, so the ground falls inside the +/-0.5 m
height band and reads as an obstacle half a metre ahead. ArduPilot is right to
refuse. The virtual source never hit this because it only ever knew about
pillars -- which is precisely the class of problem real perception introduces.

`--min-range 1.5` clears it: the nearest ground return moves out to about 1.6 m,
past the 0.6 m gate, and once airborne the ground leaves the height band
entirely. Real stereo has a minimum range anyway, so this is not a fudge -- but
be aware it blinds you inside 1.5 m. On the real aircraft, tilting the camera a
few degrees up is the better fix.
