# Steps 1-2: Get a fake drone flying on your PC

## Step 1 -- Convert WSL1 to WSL2

Your Ubuntu distro is currently on WSL version 1, which cannot run Gazebo
properly and has no GPU access. Fix it first.

In **PowerShell as Administrator** on Windows:

```
wsl --set-version Ubuntu 2
```

It takes a few minutes. Confirm:

```
wsl -l -v
```

`Ubuntu` must show `VERSION 2`. That is the gate for Step 1.

## Step 2 -- ArduPilot SITL

SITL ("software in the loop") runs the *actual ArduPilot firmware* on your PC.
This is why sim results transfer so well to the real board -- it is the same code.

Inside WSL2 Ubuntu:

```bash
sudo apt update && sudo apt install -y git python3-pip
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
```

That last script installs a lot and takes a while. Then build and run:

```bash
cd ~/ardupilot
sim_vehicle.py -v ArduCopter --console --map --out=udp:127.0.0.1:14551
```

The `--out` flag matters: it is the port `fake_obstacle_publisher.py` listens on
by default.

First run also builds the firmware, so expect several minutes.

### Connect a ground station

Run **Mission Planner** on Windows (not inside WSL) and connect over UDP.
WSL2 forwards localhost, so `127.0.0.1:14550` normally works. If it doesn't,
find the WSL IP with `hostname -I` inside Ubuntu and use that.

### Fly a mission

In Mission Planner: Flight Plan tab, click 4 waypoints on the map, Write, then
back to Flight Data, arm, switch to AUTO.

**Gate: the simulated drone flies all 4 waypoints and lands, unattended.**

## Adding Gazebo (needed for Step 4, not Step 3)

You do not need Gazebo yet. Step 3 works against plain SITL. When you get to
Step 4 and need a simulated camera, install Gazebo Harmonic and the
`ardupilot_gazebo` plugin, then launch with `-f gazebo-iris`.

Skip it for now -- one thing at a time.

## A note on this folder

This project lives on OneDrive. Two things to know:

- OneDrive sync and git can conflict. If you `git init` here, consider pausing
  sync or moving the repo somewhere outside OneDrive.
- From WSL this path is `/mnt/c/Users/gtvic/OneDrive/Desktop/Project-Alpha`,
  and file access across that boundary is slow. For the ArduPilot source tree
  specifically, clone it inside the Linux filesystem (`~/ardupilot`) as shown
  above, not under `/mnt/c`. Your own scripts here are small enough not to care.
