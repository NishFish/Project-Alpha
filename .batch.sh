set -u
P=/mnt/c/Users/gtvic/OneDrive/Desktop/Project-Alpha
export GZ_VERSION=harmonic
export GZ_SIM_SYSTEM_PLUGIN_PATH="$HOME/gz_ws/src/ardupilot_gazebo/build"
export GZ_SIM_RESOURCE_PATH="$HOME/gz_ws/src/ardupilot_gazebo/models:$HOME/gz_ws/src/ardupilot_gazebo/worlds"
export PATH="$HOME/ardupilot/Tools/autotest:$PATH"

for pat in run_course sim_vehicle arducopter "gz sim" xterm; do pkill -9 -f "$pat" 2>/dev/null; done
sleep 6
cd "$HOME"
setsid nohup env GZ_SIM_SYSTEM_PLUGIN_PATH="$GZ_SIM_SYSTEM_PLUGIN_PATH" GZ_SIM_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH" \
  gz sim -v4 -r "$P/sim/gazebo/obstacle_course.sdf" > /tmp/gz.log 2>&1 &
for i in $(seq 1 30); do timeout 5 gz topic -e -t /world/iris_runway/clock -n 1 2>/dev/null | grep -q sim && break; sleep 2; done
setsid nohup sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --no-mavproxy --no-rebuild > /tmp/sitl.log 2>&1 &
for i in $(seq 1 40); do ss -ltn 2>/dev/null | grep -q ':5760 ' && break; sleep 2; done
sleep 10
python3 "$P/sim/set_params.py" > /tmp/params.log 2>&1
echo "setup done; drone should be at origin"
python3 -u "$P/sim/run_course.py" --source depth --min-range 1.5 --noise --runs 10
echo "BATCH_EXIT=$?"
