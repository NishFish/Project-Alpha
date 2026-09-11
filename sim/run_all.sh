#!/usr/bin/env bash
#
# Everything, from nothing running to a flown obstacle course.
#
#   bash sim/run_all.sh
#
# Run it from an Ubuntu shell. Takes about four minutes, most of which is the
# flight itself. Leaves Gazebo and SITL running afterwards so you can keep
# flying; run it again to start clean.
#
# The ordering here is not arbitrary -- each step exists because of something
# that went wrong without it. See docs/03-gazebo.md.
set -uo pipefail

PROJ="$(cd "$(dirname "$0")/.." && pwd)"
SITL_DIR="$HOME"          # eeprom.bin lives here; changing it resets every parameter
WORLD="$PROJ/sim/gazebo/obstacle_course.sdf"

# Gazebo needs these, and a non-interactive shell never reads ~/.bashrc.
export GZ_VERSION=harmonic
export GZ_SIM_SYSTEM_PLUGIN_PATH="$HOME/gz_ws/src/ardupilot_gazebo/build"
export GZ_SIM_RESOURCE_PATH="$HOME/gz_ws/src/ardupilot_gazebo/models:$HOME/gz_ws/src/ardupilot_gazebo/worlds"
export PATH="$HOME/ardupilot/Tools/autotest:$PATH"

step() { printf "\n\033[1;36m==> %s\033[0m\n" "$*"; }
die()  { printf "\n\033[1;31mFAILED: %s\033[0m\n" "$*"; exit 1; }

wait_for() {   # wait_for <seconds> <description> <command...>
  local limit=$1 what=$2; shift 2
  local t=0
  while [ "$t" -lt "$limit" ]; do
    if "$@" >/dev/null 2>&1; then echo "    $what ready after ${t}s"; return 0; fi
    sleep 2; t=$((t + 2))
  done
  return 1
}

clock_ticking() { timeout 6 gz topic -e -t /world/iris_runway/clock -n 1 2>/dev/null | grep -q sim; }
sitl_listening() { ss -ltn 2>/dev/null | grep -q ':5760 '; }

step "Stopping anything already running"
for pat in run_course virtual_obstacle_publisher fake_obstacle_publisher \
           sim_vehicle mavproxy arducopter "gz sim" xterm; do
  pkill -9 -f "$pat" 2>/dev/null
done
sleep 5
echo "    stopped"

step "Generating the world from course.txt"
python3 "$PROJ/sim/gazebo/make_world.py" || die "world generation"

step "Starting Gazebo"
cd "$HOME"
setsid nohup gz sim -v4 -r "$WORLD" > /tmp/gz.log 2>&1 &
wait_for 60 "Gazebo" clock_ticking || { grep -i '\[Err\]' /tmp/gz.log | head -5; die "Gazebo never started"; }

step "Starting ArduPilot SITL"
# --no-mavproxy because MAVProxy needs a terminal and exits when detached,
# taking sim_vehicle down with it.
cd "$SITL_DIR"
setsid nohup sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON \
  --no-mavproxy --no-rebuild > /tmp/sitl.log 2>&1 &
wait_for 90 "SITL" sitl_listening || { tail -12 /tmp/sitl.log; die "SITL never listened on 5760"; }
sleep 8

step "Applying parameters"
python3 "$PROJ/sim/set_params.py" || die "parameters"

step "Flying the course"
python3 -u "$PROJ/sim/run_course.py"
rc=$?

if [ "$rc" -eq 0 ]; then
  printf "\n\033[1;32m==> Course flown, all pillars cleared\033[0m\n"
else
  printf "\n\033[1;31m==> Course run reported a problem (exit %d)\033[0m\n" "$rc"
fi
echo "Gazebo and SITL are still running. Re-run this script to start clean."
exit "$rc"
