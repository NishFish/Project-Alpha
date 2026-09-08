# Step 4 -- Gazebo

Two levels here, and they are worth keeping separate. Level A gets a drone
flying in a 3D world and is a short afternoon. Level B feeds a simulated depth
camera into `src/depth_to_obstacle_ring.py` and is the heaviest setup in the
whole project.

Do Level A first and confirm it flies before attempting Level B.

## Before anything: WSL2 has to start

Right now it does not. `wsl -d Ubuntu` fails with:

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

Run it in two terminals:

```bash
# terminal 1
gz sim -v4 -r iris_runway.sdf

# terminal 2
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console \
               --out=udp:127.0.0.1:14551
```

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
