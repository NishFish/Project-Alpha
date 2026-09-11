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
