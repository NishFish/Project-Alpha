#!/usr/bin/env bash
#
# Project Alpha -- WSL2 setup for ArduPilot SITL, Gazebo Harmonic and ROS 2.
#
# Run this from a Windows terminal so sudo can prompt you:
#
#   wsl -d Ubuntu -- bash /mnt/c/Users/gtvic/OneDrive/Desktop/Project-Alpha/setup/wsl_setup.sh
#
# Expect roughly an hour and about 25 GB. Stages are marked as they complete, so
# re-running skips what is already done -- if a stage fails, fix it and run the
# script again rather than starting over.
#
# Run a single stage with:   wsl_setup.sh base|ardupilot|gazebo|plugin|ros2
#
set -euo pipefail

MARK="$HOME/.project-alpha-setup"
mkdir -p "$MARK"

say()  { printf "\n\033[1;36m==> %s\033[0m\n" "$*"; }
skip() { printf "\033[0;90m--- %s (already done)\033[0m\n" "$*"; }
done_() { touch "$MARK/$1"; }
todo() { [ ! -f "$MARK/$1" ]; }

CODENAME="$(lsb_release -cs)"
ARCH="$(dpkg --print-architecture)"

# ---------------------------------------------------------------- base ----
stage_base() {
  if todo base; then
    say "Base build tools"
    sudo apt-get update
    sudo apt-get install -y --no-install-recommends \
      cmake ccache build-essential curl wget gnupg lsb-release \
      python3-pip python3-venv rapidjson-dev
    done_ base
  else skip "base"; fi
}

# ----------------------------------------------------------- ardupilot ----
stage_ardupilot() {
  if todo ardupilot; then
    say "ArduPilot source and prerequisites"
    # Clone into the Linux filesystem, never /mnt/c -- builds across that
    # boundary are slow enough to be worth avoiding entirely.
    [ -d "$HOME/ardupilot" ] || \
      git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git "$HOME/ardupilot"
    cd "$HOME/ardupilot"
    Tools/environment_install/install-prereqs-ubuntu.sh -y
    done_ ardupilot
    say "ArduPilot installed. Open a new shell, or run:  . ~/.profile"
  else skip "ardupilot"; fi
}

# -------------------------------------------------------------- gazebo ----
stage_gazebo() {
  if todo gazebo; then
    say "Gazebo Harmonic"
    sudo curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
      -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
    echo "deb [arch=${ARCH} signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable ${CODENAME} main" \
      | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
    sudo apt-get update
    sudo apt-get install -y gz-harmonic libgz-sim8-dev
    sudo apt-get install -y libopencv-dev libgstreamer1.0-dev \
      libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad \
      gstreamer1.0-libav gstreamer1.0-gl
    done_ gazebo
  else skip "gazebo"; fi
}

# -------------------------------------------------- ardupilot_gazebo -----
stage_plugin() {
  if todo plugin; then
    say "ArduPilot Gazebo plugin"
    mkdir -p "$HOME/gz_ws/src"
    [ -d "$HOME/gz_ws/src/ardupilot_gazebo" ] || \
      git clone https://github.com/ArduPilot/ardupilot_gazebo "$HOME/gz_ws/src/ardupilot_gazebo"
    export GZ_VERSION=harmonic
    mkdir -p "$HOME/gz_ws/src/ardupilot_gazebo/build"
    cd "$HOME/gz_ws/src/ardupilot_gazebo/build"
    cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
    make -j"$(nproc)"

    if ! grep -q "GZ_SIM_SYSTEM_PLUGIN_PATH" "$HOME/.bashrc"; then
      cat >> "$HOME/.bashrc" <<'RC'

# --- Project Alpha: Gazebo + ArduPilot ---
export GZ_VERSION=harmonic
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/gz_ws/src/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}
export GZ_SIM_RESOURCE_PATH=$HOME/gz_ws/src/ardupilot_gazebo/models:$HOME/gz_ws/src/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH:-}
RC
    fi
    done_ plugin
    say "Plugin built. LEVEL A IS NOW COMPLETE -- see 'Next' at the end."
  else skip "plugin"; fi
}

# ---------------------------------------------------------------- ros2 ----
stage_ros2() {
  if todo ros2; then
    say "ROS 2 Humble"
    sudo apt-get install -y software-properties-common
    sudo add-apt-repository -y universe
    sudo curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=${ARCH} signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${CODENAME} main" \
      | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt-get update
    sudo apt-get install -y ros-humble-desktop ros-dev-tools python3-vcstool

    if ! grep -q "ros/humble/setup.bash" "$HOME/.bashrc"; then
      echo 'source /opt/ros/humble/setup.bash' >> "$HOME/.bashrc"
    fi
    done_ ros2
  else skip "ros2"; fi
}

# ----------------------------------------------------------------- run ----
case "${1:-all}" in
  base)      stage_base ;;
  ardupilot) stage_ardupilot ;;
  gazebo)    stage_gazebo ;;
  plugin)    stage_plugin ;;
  ros2)      stage_ros2 ;;
  all)       stage_base; stage_ardupilot; stage_gazebo; stage_plugin; stage_ros2 ;;
  *) echo "usage: $0 [base|ardupilot|gazebo|plugin|ros2|all]"; exit 2 ;;
esac

cat <<'NEXT'

============================================================
Done. Open a NEW shell so the environment variables take.

LEVEL A -- fly in Gazebo, two terminals:

  gz sim -v4 -r iris_runway.sdf

  sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON \
                 --map --console --out=udp:127.0.0.1:14551

Then, with it flying, Step 3 against a real 3D simulator:

  python3 src/fake_obstacle_publisher.py

LEVEL B -- the depth camera bridge still needs the ardupilot_gz
workspace built. Those steps are NOT in this script because the
repos manifest moves; follow the official page, which assumes the
ROS 2 Humble this script just installed:

  https://ardupilot.org/dev/docs/ros2-gazebo.html

  export GZ_VERSION=harmonic
  # vcs import the ardupilot_gz repos into ~/ros2_ws/src
  rosdep install --from-paths src --ignore-src -r -y
  colcon build --packages-up-to ardupilot_gz_bringup

Then:  python3 src/gz_depth_bridge.py --topic /depth_camera
============================================================
NEXT
