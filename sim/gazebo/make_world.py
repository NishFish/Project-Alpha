#!/usr/bin/env python3
"""
Build a Gazebo world with the obstacle course baked in and a depth camera bolted
to the aircraft.

    python3 sim/gazebo/make_world.py
    gz sim -v4 -r sim/gazebo/obstacle_course.sdf

Two jobs:

1. Pillars from course.txt, as visible geometry. Spawning obstacles into a world
   that already has something flying in it shoves the aircraft out of the sky, so
   they belong in the world file.

2. A forward-facing depth camera on the airframe, matched to the OAK-D Lite:
   72 degrees horizontal, 0.2-20 m. This is what makes real detection possible --
   the aircraft finds the pillars by looking at them rather than being told where
   they are.

The drone is spliced in by INLINING iris_with_gimbal's model rather than wrapping
it in another model. Wrapping adds a nesting level, and the ArduPilot plugin
inside that model refers to its links by relative name (iris_with_standoffs::
rotor_0), so the less its structure changes the better. Inlining keeps the model
name, nesting depth and every internal reference byte-identical, and adds exactly
one link and one joint.

Camera orientation: the world includes the drone with a 90 degree yaw, and the
aircraft flies toward world +Y (north) at ArduPilot yaw 0, so the model's +X axis
is the nose. Gazebo cameras look along their own +X, so an unrotated sensor on
base_link faces forward with no rotation needed.
"""

import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
GZ_WS = os.path.expanduser("~/gz_ws/src/ardupilot_gazebo")
BASE_WORLD = os.path.join(GZ_WS, "worlds", "iris_runway.sdf")
DRONE_MODEL = os.path.join(GZ_WS, "models", "iris_with_gimbal", "model.sdf")
COURSE = os.path.join(HERE, "course.txt")
OUT = os.path.join(HERE, "obstacle_course.sdf")

PILLAR_HEIGHT = 14.0

# Matched to the OAK-D Lite so the simulated sensor lies about the same things
# the real one will. Resolution is deliberately low: 320x240 at 15 Hz is plenty
# for a 72-sector ring and keeps the Python side comfortably real-time.
CAM_FOV_DEG = 72.0
CAM_WIDTH = 320
CAM_HEIGHT = 240
CAM_NEAR = 0.2
CAM_FAR = 20.0
CAM_RATE = 15
CAM_TOPIC = "depth_camera"

CYL = """
    <model name="obs_{n}">
      <static>true</static>
      <pose>{east} {north} {z} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry>
        </collision>
        <visual name="visual">
          <geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry>
          <material>
            <ambient>0.62 0.26 0.08 1</ambient>
            <diffuse>0.85 0.36 0.11 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""

BOX = """
    <model name="obs_{n}">
      <static>true</static>
      <pose>{east} {north} {z} 0 0 {yaw_rad:.6f}</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{sx} {sy} {h}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{sx} {sy} {h}</size></box></geometry>
          <material>
            <ambient>0.30 0.31 0.36 1</ambient>
            <diffuse>0.46 0.48 0.55 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""

# Markers are visual only -- no <collision>. They sit flat on the ground so the
# depth camera never reports them: at cruise altitude the ground is far outside
# the height band. A vertical marker would be seen as an obstacle.
MARKER = """
    <model name="{name}">
      <static>true</static>
      <pose>{east} {north} 0.08 0 0 0</pose>
      <link name="link">
        <visual name="disc">
          <geometry><cylinder><radius>{r}</radius><length>0.15</length></cylinder></geometry>
          <material>
            <ambient>{col} 1</ambient>
            <diffuse>{col} 1</diffuse>
            <emissive>{emis} 1</emissive>
          </material>
        </visual>
      </link>
    </model>
"""

DEPTH_CAMERA = """
      <!-- Added by make_world.py: forward-facing depth camera, OAK-D Lite class.
           Gazebo cameras look along +X, and the model's +X is the nose, so no
           rotation is needed here. -->
      <link name="depth_link">
        <pose>0.12 0 0.03 0 0 0</pose>
        <inertial>
          <mass>0.02</mass>
          <inertia>
            <ixx>1e-5</ixx><iyy>1e-5</iyy><izz>1e-5</izz>
            <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
          </inertia>
        </inertial>
        <visual name="visual">
          <geometry><box><size>0.03 0.09 0.02</size></box></geometry>
          <material>
            <ambient>0.05 0.05 0.05 1</ambient>
            <diffuse>0.12 0.12 0.12 1</diffuse>
          </material>
        </visual>
        <sensor name="depth_camera" type="depth_camera">
          <always_on>1</always_on>
          <update_rate>{rate}</update_rate>
          <visualize>true</visualize>
          <topic>{topic}</topic>
          <camera>
            <horizontal_fov>{fov_rad:.6f}</horizontal_fov>
            <image>
              <width>{width}</width>
              <height>{height}</height>
              <format>R_FLOAT32</format>
            </image>
            <clip>
              <near>{near}</near>
              <far>{far}</far>
            </clip>
          </camera>
        </sensor>
      </link>

      <joint name="depth_joint" type="fixed">
        <parent>iris_with_standoffs::base_link</parent>
        <child>depth_link</child>
      </joint>
"""


def read_course(path):
    obstacles = []
    with open(path) as fh:
        for lineno, raw in enumerate(fh, 1):
            line = raw.split("#", 1)[0].strip()
            if not line:
                continue
            parts = line.split()
            if len(parts) != 3:
                sys.exit("%s:%d: expected 'north east radius', got %r"
                         % (path, lineno, line))
            obstacles.append(tuple(float(p) for p in parts))
    return obstacles


def drone_with_camera():
    """iris_with_gimbal's model block, with a depth camera link and joint added."""
    text = open(DRONE_MODEL).read()

    m = re.search(r"(<model\b.*?</model>)", text, re.S)
    if not m:
        sys.exit("no <model> found in %s" % DRONE_MODEL)
    model = m.group(1)

    sensor = DEPTH_CAMERA.format(
        rate=CAM_RATE, topic=CAM_TOPIC,
        fov_rad=CAM_FOV_DEG * 3.141592653589793 / 180.0,
        width=CAM_WIDTH, height=CAM_HEIGHT, near=CAM_NEAR, far=CAM_FAR)

    idx = model.rindex("</model>")
    return model[:idx] + sensor + model[idx:]


def main():
    for path in (BASE_WORLD, DRONE_MODEL):
        if not os.path.exists(path):
            sys.exit("not found: %s -- run setup/wsl_setup.sh plugin first" % path)

    sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(HERE)), "src"))
    from course import load

    world = open(BASE_WORLD).read()
    crs = load(COURSE)

    inc = re.search(
        r"[ 	]*<include>\s*<uri>model://iris_with_gimbal</uri>\s*"
        r"(<pose[^>]*>[^<]*</pose>)?\s*</include>", world, re.S)
    if not inc:
        sys.exit("could not find the iris_with_gimbal <include> in %s" % BASE_WORLD)

    pose = inc.group(1) or         '<pose degrees="true">0 0 0.195 0 0 90</pose>'
    model = drone_with_camera()
    open_tag = re.search(r"<model\s[^>]*>", model).group(0)
    model = model.replace(open_tag, open_tag + chr(10) + "      " + pose, 1)
    world = world[:inc.start()] + "    " + model + world[inc.end():]

    # Gazebo is ENU: x is east, y is north. Compass yaw runs clockwise, Gazebo
    # yaw counter-clockwise, so the sign flips. One conversion, one place.
    blocks = []
    for i, shp in enumerate(crs.shapes, 1):
        if shp.kind == "cyl":
            blocks.append(CYL.format(n=i, east=shp.east, north=shp.north,
                                     z=PILLAR_HEIGHT / 2.0, r=shp.radius,
                                     h=PILLAR_HEIGHT))
        else:
            blocks.append(BOX.format(n=i, east=shp.east, north=shp.north,
                                     z=PILLAR_HEIGHT / 2.0,
                                     sx=shp.size_east, sy=shp.size_north,
                                     h=PILLAR_HEIGHT,
                                     yaw_rad=-math.radians(shp.yaw_deg)))

    blocks.append(MARKER.format(name="marker_start", north=crs.start[0],
                                east=crs.start[1], r=5.0,
                                col="0.10 0.65 0.25", emis="0.05 0.30 0.12"))
    blocks.append(MARKER.format(name="marker_goal", north=crs.goal[0],
                                east=crs.goal[1], r=5.0,
                                col="0.85 0.15 0.15", emis="0.40 0.05 0.05"))

    idx = world.rindex("</world>")
    world = world[:idx] + "".join(blocks) + world[idx:]

    with open(OUT, "w") as fh:
        fh.write(world)

    print("base world : %s" % BASE_WORLD)
    print("depth cam  : %.0f deg, %dx%d, %.1f-%.1f m, %d Hz on /%s"
          % (CAM_FOV_DEG, CAM_WIDTH, CAM_HEIGHT, CAM_NEAR, CAM_FAR,
             CAM_RATE, CAM_TOPIC))
    print("course     :")
    for line in crs.describe().split(chr(10)):
        print("  " + line)
    print("markers    : green disc at start, red disc at goal (visual only)")
    print("wrote      : %s" % OUT)


if __name__ == "__main__":
    main()
