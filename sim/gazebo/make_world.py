#!/usr/bin/env python3
"""
Build a Gazebo world with the obstacle course baked in.

Spawning obstacles into a world that already has an aircraft flying in it is a
good way to knock the aircraft down -- a static 14 m pillar appearing around a
hovering drone simply shoves it out of the sky. Putting the pillars in the world
file instead means they exist before anything takes off, and they are visible
from the moment Gazebo opens.

    python3 sim/gazebo/make_world.py
    gz sim -v4 -r sim/gazebo/obstacle_course.sdf

Reads the same course.txt the publisher reads, so the geometry the aircraft
sees and the geometry you see can never drift apart.

Coordinate note, measured rather than assumed: this world frame is ENU, so
Gazebo x is EAST and Gazebo y is NORTH, while ArduPilot's LOCAL_POSITION_NED is
x=north, y=east. course.txt is written in ArduPilot's convention, so north and
east get swapped on the way in.
"""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
BASE = os.path.expanduser("~/gz_ws/src/ardupilot_gazebo/worlds/iris_runway.sdf")
COURSE = os.path.join(HERE, "course.txt")
OUT = os.path.join(HERE, "obstacle_course.sdf")

PILLAR_HEIGHT = 14.0

TEMPLATE = """
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
            <specular>0.10 0.10 0.10 1</specular>
          </material>
        </visual>
      </link>
    </model>
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


def main():
    if not os.path.exists(BASE):
        sys.exit("base world not found: %s\n"
                 "Run setup/wsl_setup.sh plugin first." % BASE)

    world = open(BASE).read()
    obstacles = read_course(COURSE)

    blocks = "".join(
        TEMPLATE.format(n=i + 1, east=east, north=north, z=PILLAR_HEIGHT / 2.0,
                        r=radius, h=PILLAR_HEIGHT)
        for i, (north, east, radius) in enumerate(obstacles))

    marker = "</world>"
    if marker not in world:
        sys.exit("no </world> in %s -- cannot splice" % BASE)
    idx = world.rindex(marker)
    out = world[:idx] + blocks + world[idx:]

    with open(OUT, "w") as fh:
        fh.write(out)

    print("base   : %s" % BASE)
    print("course : %d pillars from %s" % (len(obstacles), COURSE))
    for i, (north, east, radius) in enumerate(obstacles, 1):
        print("  obs_%-2d north %6.1f  east %6.1f  r %.1f" % (i, north, east, radius))
    print("wrote  : %s" % OUT)
    print("\nlaunch with:\n  gz sim -v4 -r %s" % OUT)


if __name__ == "__main__":
    main()
