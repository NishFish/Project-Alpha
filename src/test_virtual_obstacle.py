#!/usr/bin/env python3
"""Geometry checks for virtual_obstacle_publisher.build_ring.

Runs without MAVLink or a simulator: python3 src/test_virtual_obstacle.py
"""

import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
_src = open(os.path.join(HERE, "virtual_obstacle_publisher.py")).read()
_ns = {}
exec(compile(_src.replace("from pymavlink import mavutil", "mavutil = None"),
             "virtual_obstacle_publisher.py", "exec"), _ns)
build_ring = _ns["build_ring"]

passed = failed = 0


def check(name, cond, detail=""):
    global passed, failed
    if cond:
        passed += 1
        print("  ok   %s" % name)
    else:
        failed += 1
        print("  FAIL %s  %s" % (name, detail))


def lit_sectors(ring, max_cm):
    return [i for i, v in enumerate(ring) if v <= max_cm]


# Obstacle 30 m due north, radius 4, facing it. Surface is at 26 m.
ring, _, mx = build_ring([(30.0, 0.0, 4.0)], 0.0, 0.0, 0.0, 72.0, 0.2, 40.0)
lit = lit_sectors(ring, mx)
check("sees an obstacle dead ahead", 0 in lit, lit)
check("reports range to the SURFACE, not the centre",
      abs(ring[0] / 100.0 - 26.0) < 0.15, "%.2f m" % (ring[0] / 100.0))

# asin(4/30) = 7.66 deg half-width -> about 3 sectors of 5 deg
check("angular width is plausible", 2 <= len(lit) <= 5, "%d sectors" % len(lit))

# Turned 90 deg: the obstacle is abeam, outside a 72 deg forward FOV.
ring2, _, mx2 = build_ring([(30.0, 0.0, 4.0)], 0.0, 0.0, math.radians(90), 72.0, 0.2, 40.0)
check("blind to what is outside the FOV", lit_sectors(ring2, mx2) == [])

# 45 deg off the nose is outside +/-36.
ring3, _, mx3 = build_ring([(20.0, 20.0, 3.0)], 0.0, 0.0, 0.0, 72.0, 0.2, 60.0)
check("45 deg off is out of a 72 deg FOV", lit_sectors(ring3, mx3) == [])

# Widen the FOV and it appears, to the RIGHT (east is +y, so low indices).
ring4, _, mx4 = build_ring([(20.0, 20.0, 3.0)], 0.0, 0.0, 0.0, 180.0, 0.2, 60.0)
lit4 = lit_sectors(ring4, mx4)
check("visible with a wide FOV", bool(lit4), lit4)
check("bears to the right, near sector 9 (45 deg)",
      bool(lit4) and min(lit4) <= 9 <= max(lit4), lit4)

# Beyond sensor range -> nothing.
ring5, _, mx5 = build_ring([(100.0, 0.0, 4.0)], 0.0, 0.0, 0.0, 72.0, 0.2, 20.0)
check("beyond max range reports nothing", all(v > mx5 for v in ring5))

# Inside the obstacle -> nothing, never a negative range.
ring6, _, mx6 = build_ring([(1.0, 0.0, 4.0)], 0.0, 0.0, 0.0, 72.0, 0.2, 40.0)
check("inside the obstacle yields no bogus ranges", all(v > mx6 for v in ring6))

# Nearer of two overlapping obstacles wins.
ring7, _, mx7 = build_ring([(30.0, 0.0, 4.0), (15.0, 0.0, 3.0)],
                           0.0, 0.0, 0.0, 72.0, 0.2, 40.0)
check("reports the nearer of two in line",
      abs(ring7[0] / 100.0 - 12.0) < 0.15, "%.2f m" % (ring7[0] / 100.0))

# Every ring is exactly 72 entries.
check("ring is always 72 sectors", all(len(r) == 72 for r in
      (ring, ring2, ring3, ring4, ring5, ring6, ring7)))

print("\n" + "=" * 44)
print("%d passed, %d failed" % (passed, failed))
sys.exit(1 if failed else 0)
