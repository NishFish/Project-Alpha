#!/usr/bin/env python3
"""Geometry checks for course.py. No simulator, no MAVLink."""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from course import Box, Cyl, load

passed = failed = 0


def check(name, cond, detail=""):
    global passed, failed
    if cond:
        passed += 1
        print("  ok   %s" % name)
    else:
        failed += 1
        print("  FAIL %s  %s" % (name, detail))


# --- cylinder --------------------------------------------------------------
c = Cyl(30.0, 0.0, 2.0)
check("cyl distance is to the surface", abs(c.distance(0, 0) - 28.0) < 1e-6,
      "%.3f" % c.distance(0, 0))
check("cyl distance is negative inside", c.distance(30, 0) < 0)

t = c.ray(0, 0, 1.0, 0.0, 100.0)     # due north
check("cyl ray hits the near face", t is not None and abs(t - 28.0) < 1e-6,
      "%s" % t)
check("cyl ray misses when aimed away", c.ray(0, 0, -1.0, 0.0, 100.0) is None)
check("cyl ray respects max range", c.ray(0, 0, 1.0, 0.0, 10.0) is None)

# --- axis-aligned box ------------------------------------------------------
b = Box(20.0, 0.0, 10.0, 4.0, 0.0)   # 10 m north-south, 4 m east-west
check("box distance along north", abs(b.distance(0, 0) - 15.0) < 1e-6,
      "%.3f" % b.distance(0, 0))     # 20 - 10/2 = 15
check("box distance along east", abs(b.distance(20, 10) - 8.0) < 1e-6,
      "%.3f" % b.distance(20, 10))   # 10 - 4/2 = 8
check("box distance negative inside", b.distance(20, 0) < 0)

# Diagonal: nearest point is the corner at (25, 2)
d = b.distance(30, 7)
check("box distance uses the corner diagonally",
      abs(d - math.hypot(5.0, 5.0)) < 1e-6, "%.3f" % d)

t = b.ray(0, 0, 1.0, 0.0, 100.0)
check("box ray hits the near face", t is not None and abs(t - 15.0) < 1e-6, "%s" % t)
check("box ray misses to the side", b.ray(0, 20, 1.0, 0.0, 100.0) is None)

# --- rotated box -----------------------------------------------------------
# 20 m long, rotated 90 deg -> now lies east-west across the track.
r = Box(40.0, 0.0, 20.0, 2.0, 90.0)
check("rotated box blocks the track", abs(r.distance(0, 0) - 39.0) < 1e-6,
      "%.3f" % r.distance(0, 0))     # half of the 2 m thickness faces us
check("rotated box extends sideways", r.distance(40.0, 9.0) < 0.1,
      "%.3f" % r.distance(40.0, 9.0))
tr = r.ray(0, 0, 1.0, 0.0, 100.0)
check("rotated box ray hits at 39 m", tr is not None and abs(tr - 39.0) < 1e-6,
      "%s" % tr)

# A ray 9 m east still hits it, because it is 20 m wide.
tr2 = r.ray(0, 9, 1.0, 0.0, 100.0)
check("wide wall is hit off-centre too", tr2 is not None and abs(tr2 - 39.0) < 1e-6,
      "%s" % tr2)
# 11 m east clears the end of it.
check("beyond the end of the wall the ray passes",
      r.ray(0, 11, 1.0, 0.0, 100.0) is None)

# --- the real course file --------------------------------------------------
here = os.path.dirname(os.path.abspath(__file__))
path = os.path.join(os.path.dirname(here), "sim", "gazebo", "course.txt")
crs = load(path)
check("course parses", len(crs.shapes) > 0, "%d shapes" % len(crs.shapes))
check("start is defined", crs.start == (0.0, 0.0), "%s" % (crs.start,))
check("goal is defined", crs.goal[0] > 100, "%s" % (crs.goal,))
check("course mixes cylinders and boxes",
      any(s.kind == "cyl" for s in crs.shapes) and
      any(s.kind == "box" for s in crs.shapes))
check("start is not inside an obstacle",
      crs.nearest_surface(*crs.start) > 2.0,
      "%.2f m" % crs.nearest_surface(*crs.start))
check("goal is not inside an obstacle",
      crs.nearest_surface(*crs.goal) > 2.0,
      "%.2f m" % crs.nearest_surface(*crs.goal))

# Straight line from start to goal must actually be blocked, or the course
# proves nothing.
blocked = crs.ray_range(crs.start[0], crs.start[1], 0.0, 200.0)
check("the direct path is blocked", blocked is not None,
      "nothing in the way -- the course is trivial")
print("      first obstacle on the straight line: %.1f m" % blocked)

print("\n" + "=" * 44)
print("%d passed, %d failed" % (passed, failed))
sys.exit(1 if failed else 0)
