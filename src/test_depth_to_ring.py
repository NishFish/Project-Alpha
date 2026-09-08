#!/usr/bin/env python3
"""
Self-tests for the depth pipeline. No camera, no drone, no MAVLink required.

    python src/test_depth_to_ring.py

These are not decoration. Every one of them corresponds to a specific way this
conversion goes wrong in the air, and each failure mode is named in the test so
that when one breaks you know what it would have cost you.
"""

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from depth_to_obstacle_ring import DepthToRing, synthetic_depth
from obstacle_ring import NUM_SECTORS, TemporalFilter, bearing_to_sector

PASSED = []
FAILED = []


def check(name, condition, detail=""):
    if condition:
        PASSED.append(name)
        print("  PASS  {}".format(name))
    else:
        FAILED.append(name)
        print("  FAIL  {}  {}".format(name, detail))


def make(depth, fx, fy, cx, cy, **kw):
    h, w = depth.shape
    return DepthToRing(fx, fy, cx, cy, w, h, **kw)


def test_wall_distance():
    """A wall at 3 m must read 3 m dead ahead.

    Fails if: the pipeline is broken at the most basic level.
    """
    depth, fx, fy, cx, cy = synthetic_depth(wall_m=3.0)
    ring = make(depth, fx, fy, cx, cy).process(depth)
    check("wall at 3 m reads ~300 cm ahead", 297 <= ring[0] <= 303,
          "got {} cm".format(ring[0]))


def test_radial_conversion():
    """Off-axis sectors of a flat wall must read FURTHER than dead ahead.

    A fronto-parallel wall at z = 3 m is 3/cos(theta) away at bearing theta.
    Sector 3 sits at 15 degrees, so it must read about 308 cm, not 300.

    Fails if: depth z is being published instead of true range. The aircraft
    then thinks obstacles near the frame edge are closer than they are and
    brakes for phantoms exactly when it is trying to sidestep.
    """
    depth, fx, fy, cx, cy = synthetic_depth(wall_m=3.0, wall_half_deg=25.0)
    ring = make(depth, fx, fy, cx, cy).process(depth)
    expected = 300.0 / math.cos(math.radians(15.0))
    check("15 deg sector reads range not depth",
          abs(ring[3] - expected) <= 8 and ring[3] > ring[0],
          "got {} cm, expected ~{:.0f} cm".format(ring[3], expected))


def test_ground_is_rejected():
    """A ground plane 1 m below must produce no obstacles at all.

    Fails if: the height filter is missing or wrong. This is the single most
    common way this pipeline fails -- the floor becomes the nearest obstacle in
    every frame and the aircraft refuses to move.
    """
    depth, fx, fy, cx, cy = synthetic_depth(ground_below_m=1.0)
    conv = make(depth, fx, fy, cx, cy, height_band_m=0.5)
    conv.process(depth)
    check("level ground is not an obstacle", conv.last_stats["sectors"] == 0,
          "got {} populated sectors".format(conv.last_stats["sectors"]))


def test_wall_survives_ground():
    """With both present, the wall must still be seen.

    Fails if: the height filter is so aggressive it discards real obstacles too.
    """
    depth, fx, fy, cx, cy = synthetic_depth(wall_m=3.0, ground_below_m=1.0)
    conv = make(depth, fx, fy, cx, cy, height_band_m=0.5)
    ring = conv.process(depth)
    check("wall still detected alongside ground",
          297 <= ring[0] <= 303 and conv.last_stats["sectors"] > 0,
          "ahead={} cm, sectors={}".format(ring[0], conv.last_stats["sectors"]))


def test_pitch_compensation():
    """Pitched nose-down at ground, compensation decides whether it is an obstacle.

    The scene is the ground seen by a camera pitched 20 degrees down -- exactly
    what the aircraft sees while accelerating. Told the true attitude, the
    pipeline must still reject the ground. Told the aircraft is level, it must
    report the ground as an obstacle a few metres ahead.

    Fails if: attitude is not being applied, or its sign is inverted. The
    symptom in flight is an aircraft that brakes hard every time it accelerates.
    """
    depth, fx, fy, cx, cy = synthetic_depth(ground_below_m=1.0, pitch_deg=-20.0)

    conv = make(depth, fx, fy, cx, cy, height_band_m=0.5)
    conv.process(depth, pitch_rad=math.radians(-20.0))
    compensated = conv.last_stats["sectors"]

    conv2 = make(depth, fx, fy, cx, cy, height_band_m=0.5)
    conv2.process(depth, pitch_rad=0.0)
    uncompensated = conv2.last_stats["sectors"]

    check("pitch compensation rejects tilted ground", compensated == 0,
          "got {} sectors".format(compensated))
    check("without compensation the ground becomes an obstacle", uncompensated > 0,
          "got {} sectors -- the test scene may be wrong".format(uncompensated))


def test_invalid_depth_is_not_zero_distance():
    """An all-invalid frame must report 'nothing', never 'obstacle at 0 cm'.

    Fails if: stereo dropouts are treated as distances. Zero means an obstacle
    is touching the drone, so this failure hard-stops the aircraft mid-mission.
    """
    depth = np.zeros((400, 640), dtype=np.float32)
    conv = DepthToRing(430.0, 430.0, 319.5, 199.5, 640, 400)
    ring = conv.process(depth)
    check("all-invalid frame reports nothing",
          all(v > conv.max_cm for v in ring) and len(ring) == NUM_SECTORS,
          "min value {}".format(min(ring)))


def test_unseen_sectors_stay_empty():
    """Sectors outside the lens must not be invented.

    A 640x400 frame at fx=430 covers roughly +/- 37 degrees, so only about 15 of
    72 sectors can ever hold data. The rest must stay empty.
    """
    depth, fx, fy, cx, cy = synthetic_depth(wall_m=3.0, wall_half_deg=90.0)
    conv = make(depth, fx, fy, cx, cy)
    ring = conv.process(depth)
    seen = [i for i, v in enumerate(ring) if v <= conv.max_cm]
    behind = [i for i in seen if 20 <= i <= 52]
    check("nothing is reported behind the aircraft", not behind,
          "sectors {} populated".format(behind))
    check("field of view covers a plausible sector count", 8 <= len(seen) <= 24,
          "{} sectors populated".format(len(seen)))


def test_temporal_filter_rejects_single_frame_noise():
    """One bad frame in three must not move the output.

    Fails if: the aircraft brakes for isolated stereo noise.
    """
    clear = [601] * NUM_SECTORS
    spike = list(clear)
    spike[0] = 80

    filt = TemporalFilter(frames=3)
    filt.update(clear)
    filt.update(spike)
    out = filt.update(clear)
    check("single spurious frame is outvoted", out[0] == 601,
          "got {}".format(out[0]))

    filt2 = TemporalFilter(frames=3)
    filt2.update(clear)
    filt2.update(spike)
    out2 = filt2.update(spike)
    check("a persistent obstacle does get through", out2[0] == 80,
          "got {}".format(out2[0]))


def test_bearing_to_sector_wraps():
    check("bearing 0 maps to sector 0", bearing_to_sector(0.0) == 0)
    check("bearing 5 maps to sector 1", bearing_to_sector(5.0) == 1)
    check("bearing -5 wraps to sector 71", bearing_to_sector(-5.0) == 71)
    check("bearing 360 wraps to sector 0", bearing_to_sector(360.0) == 0)


def main():
    print("\nDepth -> obstacle ring, self-tests\n" + "=" * 44)
    for fn in [test_wall_distance,
               test_radial_conversion,
               test_ground_is_rejected,
               test_wall_survives_ground,
               test_pitch_compensation,
               test_invalid_depth_is_not_zero_distance,
               test_unseen_sectors_stay_empty,
               test_temporal_filter_rejects_single_frame_noise,
               test_bearing_to_sector_wraps]:
        print("\n{}".format(fn.__name__))
        fn()

    print("\n" + "=" * 44)
    print("{} passed, {} failed".format(len(PASSED), len(FAILED)))
    return 1 if FAILED else 0


if __name__ == "__main__":
    sys.exit(main())
