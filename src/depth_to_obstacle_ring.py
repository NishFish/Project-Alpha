#!/usr/bin/env python3
"""
STEP 4 -- Turn a depth image into the 72-sector obstacle ring.

This is the piece that decides whether the aircraft flies or crashes, so the
reasoning behind each stage is written down rather than left implicit.

The camera frame used throughout is the standard optical one:

    x -> right,  y -> down,  z -> forward (along the optical axis)

and a "depth image" holds z per pixel, NOT distance to the point. Those differ
by more than 20 percent at the edges of a wide lens, which is exactly where you
are trying to sidestep.

Run it with no hardware:

    python depth_to_obstacle_ring.py --demo wall
    python depth_to_obstacle_ring.py --demo ground
    python depth_to_obstacle_ring.py --demo both --pitch 10

Run it against a recorded frame:

    python depth_to_obstacle_ring.py --npy frame.npy

Run it for real, later:

    python depth_to_obstacle_ring.py --npy frame.npy --send --connect /dev/serial0
"""

import argparse

import numpy as np

from obstacle_ring import (NUM_SECTORS, SECTOR_WIDTH_DEG, empty_ring,
                           render_ring)


class DepthToRing:
    """Converts depth frames into ArduPilot's 72-sector distance ring.

    Parameters worth understanding rather than copying:

    height_band_m
        Only points within this many metres of the drone's own height, measured
        along true gravity, count as obstacles. Without it the ground is the
        nearest thing in every frame and the aircraft never moves.

        Note this is a filter on real 3D height, not on image rows. A fixed band
        of rows corresponds to a different physical height at every distance, so
        it simultaneously includes the ground far away and misses obstacles up
        close. Filtering on Y is range-correct for free.

    percentile
        Per sector we take a low percentile of the ranges, not the minimum. The
        minimum is a single pixel, and single pixels in stereo depth are often
        wrong. Ten percent keeps you conservative without braking for noise.

    min_pixels
        A sector backed by three pixels is not a measurement. Below this count
        the sector is reported as unobserved.

    stride
        Subsample the depth image. A Pi 5 does not need every pixel of a
        640x400 frame to decide whether a tree is there, and this is the
        cheapest available speedup.
    """

    def __init__(self, fx, fy, cx, cy, width, height,
                 min_range_m=0.2, max_range_m=6.0,
                 height_band_m=0.5, min_pixels=30, percentile=10.0,
                 stride=2, mask=None):
        self.min_range_m = float(min_range_m)
        self.max_range_m = float(max_range_m)
        self.height_band_m = float(height_band_m)
        self.min_pixels = int(min_pixels)
        self.percentile = float(percentile)
        self.stride = int(stride)

        self.min_cm = int(self.min_range_m * 100)
        self.max_cm = int(self.max_range_m * 100)

        # Precompute the per-pixel ray directions once. These never change, so
        # doing it per frame would just be burning battery.
        rows = np.arange(0, height, self.stride)
        cols = np.arange(0, width, self.stride)
        self._rows = rows
        self._cols = cols
        self._ax = ((cols - cx) / float(fx))[None, :]   # x / z
        self._ay = ((rows - cy) / float(fy))[:, None]   # y / z

        if mask is None:
            self._mask = None
        else:
            # True means "ignore this pixel" -- propellers, arms, landing gear.
            self._mask = np.asarray(mask, dtype=bool)[np.ix_(rows, cols)]

        self.last_stats = {}

    def process(self, depth_m, pitch_rad=0.0, roll_rad=0.0):
        """One depth frame in, one 72-element ring (centimetres) out.

        pitch_rad / roll_rad come from the autopilot's ATTITUDE message, using
        MAVLink's convention: pitch positive is nose up, roll positive is right
        side down. Supplying them keeps the height filter aligned with real
        gravity instead of with the camera, which matters as soon as the
        aircraft tilts to accelerate -- otherwise it starts reading the ground
        as an obstacle the moment it moves.
        """
        depth = np.asarray(depth_m, dtype=np.float32)[np.ix_(self._rows, self._cols)]

        # Stereo reports 0 where it could not find a match, and the far field is
        # not trustworthy. Both must be discarded, not clamped: a 0 treated as a
        # distance means "obstacle touching the drone".
        valid = np.isfinite(depth) & (depth >= self.min_range_m) & (depth <= self.max_range_m)
        if self._mask is not None:
            valid &= ~self._mask

        total_valid = int(valid.sum())
        if total_valid == 0:
            self.last_stats = {"valid_px": 0, "in_band_px": 0, "sectors": 0}
            return empty_ring(self.max_cm)

        # Neutralise the rejected pixels before any arithmetic. Gazebo reports
        # the sky as +inf, and 0 * inf is NaN, which raises warnings and then
        # quietly fails every comparison it touches. They are excluded by `valid`
        # anyway, so their value is irrelevant -- it just has to be finite.
        depth = np.where(valid, depth, 0.0)

        x = self._ax * depth
        y = self._ay * depth
        z = depth

        # Gravity-down expressed in camera coordinates. Check it at the corners:
        # level gives (0, 1, 0); nose up 90 degrees gives (0, 0, -1) because
        # "down" is then behind the lens; rolled 90 degrees right gives (1, 0, 0).
        sr, cr = np.sin(roll_rad), np.cos(roll_rad)
        sp, cp = np.sin(pitch_rad), np.cos(pitch_rad)
        gx, gy, gz = sr, cr * cp, -cr * sp

        # Projection onto that axis is how far the point sits below the drone.
        height_below = x * gx + y * gy + z * gz
        valid &= np.abs(height_below) <= self.height_band_m

        in_band = int(valid.sum())
        if in_band == 0:
            self.last_stats = {"valid_px": total_valid, "in_band_px": 0, "sectors": 0}
            return empty_ring(self.max_cm)

        xv = x[valid]
        zv = z[valid]

        # Bearing from the geometry itself, which is the same as atan2(u - cx, fx)
        # but harder to get wrong. Positive x is to the right, and the ring's
        # index increases clockwise, so no sign juggling is needed.
        bearing_deg = np.degrees(np.arctan2(xv, zv))

        # Horizontal range to the point. ArduPilot's proximity ring is a 2D
        # horizontal picture, so this -- not the raw depth z -- is what it wants.
        ranges_m = np.sqrt(xv * xv + zv * zv)

        sectors = np.rint(bearing_deg / SECTOR_WIDTH_DEG).astype(np.int32) % NUM_SECTORS

        # Group by sector with one sort rather than 72 passes over the array.
        order = np.argsort(sectors, kind="stable")
        sec_sorted = sectors[order]
        rng_sorted = ranges_m[order]
        bounds = np.searchsorted(sec_sorted, np.arange(NUM_SECTORS + 1))

        ring = empty_ring(self.max_cm)
        populated = 0
        for i in range(NUM_SECTORS):
            lo, hi = bounds[i], bounds[i + 1]
            if hi - lo < self.min_pixels:
                continue
            value_m = np.percentile(rng_sorted[lo:hi], self.percentile)
            cm = int(round(value_m * 100))
            if self.min_cm <= cm <= self.max_cm:
                ring[i] = cm
                populated += 1

        self.last_stats = {"valid_px": total_valid, "in_band_px": in_band,
                           "sectors": populated}
        return ring


def synthetic_depth(width=640, height=400, fx=430.0, fy=430.0,
                    cx=None, cy=None, wall_m=None, wall_half_deg=20.0,
                    ground_below_m=None, pitch_deg=0.0):
    """Build a depth frame for testing, so the pipeline can be developed and
    validated long before the camera arrives.

    wall_m
        A flat wall at this distance, spanning +/- wall_half_deg of bearing.
    ground_below_m
        A ground plane this far below the camera. Include it: the ground is the
        single most effective way to prove the height filter works, because a
        pipeline without one reports the floor as an obstacle every frame.
    pitch_deg
        Tilt the camera relative to the ground plane, nose up positive. Lets a
        test generate the scene an aircraft actually sees while accelerating,
        which is the case that breaks a naive implementation.
    """
    cx = (width - 1) / 2.0 if cx is None else cx
    cy = (height - 1) / 2.0 if cy is None else cy

    u = np.arange(width)[None, :]
    v = np.arange(height)[:, None]
    ax = (u - cx) / fx
    ay = (v - cy) / fy

    depth = np.full((height, width), np.inf, dtype=np.float32)

    if ground_below_m is not None:
        # A ray t * (ax, ay, 1) reaches ground_below_m along true gravity where
        # t * dot((ax, ay, 1), g) = ground_below_m, with g the gravity-down
        # vector in camera coordinates -- the same one process() uses.
        sp, cp = np.sin(np.radians(pitch_deg)), np.cos(np.radians(pitch_deg))
        denom = ay * cp - sp
        with np.errstate(divide="ignore", invalid="ignore"):
            z_ground = np.where(denom > 1e-6, ground_below_m / denom, np.inf)
        depth = np.minimum(depth, np.broadcast_to(z_ground, depth.shape))

    if wall_m is not None:
        bearing = np.degrees(np.arctan2(ax, 1.0))
        on_wall = np.broadcast_to(np.abs(bearing) <= wall_half_deg, depth.shape)
        depth = np.where(on_wall, np.minimum(depth, wall_m), depth)

    # Whatever was never hit is "no return", which stereo reports as zero.
    depth = np.where(np.isfinite(depth), depth, 0.0)
    return depth.astype(np.float32), fx, fy, cx, cy


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--demo", choices=["wall", "ground", "both"],
                        help="run against a synthetic scene instead of real data")
    parser.add_argument("--npy", help="load a depth frame from a .npy file (metres)")
    parser.add_argument("--mm", action="store_true",
                        help="the .npy holds millimetres (the OAK-D native format)")
    parser.add_argument("--distance", type=float, default=3.0,
                        help="wall distance for --demo, metres")
    parser.add_argument("--ground", type=float, default=1.0,
                        help="ground depth below camera for --demo, metres")
    parser.add_argument("--pitch", type=float, default=0.0,
                        help="pitch in degrees, nose up positive")
    parser.add_argument("--roll", type=float, default=0.0,
                        help="roll in degrees, right side down positive")
    parser.add_argument("--max-range", type=float, default=6.0)
    parser.add_argument("--height-band", type=float, default=0.5)
    parser.add_argument("--no-compensate", action="store_true",
                        help="pretend the aircraft is level even when it is not, "
                             "to show what skipping attitude compensation costs")
    parser.add_argument("--send", action="store_true",
                        help="publish the ring over MAVLink")
    parser.add_argument("--connect", default="udpin:127.0.0.1:14551")
    parser.add_argument("--baud", type=int, default=460800)
    args = parser.parse_args()

    if args.demo:
        wall = args.distance if args.demo in ("wall", "both") else None
        ground = args.ground if args.demo in ("ground", "both") else None
        depth, fx, fy, cx, cy = synthetic_depth(wall_m=wall, ground_below_m=ground,
                                                pitch_deg=args.pitch)
        print("Synthetic scene: wall={} ground_below={} pitch={} deg".format(
            wall, ground, args.pitch))
    elif args.npy:
        depth = np.load(args.npy)
        if args.mm:
            depth = depth.astype(np.float32) / 1000.0
        h, w = depth.shape
        # Without real intrinsics this is a guess. Replace it with the values the
        # OAK-D reports for your own device before trusting any of these numbers.
        fx = fy = 430.0
        cx, cy = (w - 1) / 2.0, (h - 1) / 2.0
        print("Loaded {} {} -- USING PLACEHOLDER INTRINSICS".format(args.npy, depth.shape))
    else:
        parser.error("pass --demo or --npy")

    h, w = depth.shape
    converter = DepthToRing(fx, fy, cx, cy, w, h,
                            max_range_m=args.max_range,
                            height_band_m=args.height_band)

    told_pitch = 0.0 if args.no_compensate else args.pitch
    told_roll = 0.0 if args.no_compensate else args.roll
    ring = converter.process(depth,
                             pitch_rad=np.radians(told_pitch),
                             roll_rad=np.radians(told_roll))

    print("stats: {}".format(converter.last_stats))
    print("\nforward sectors ('-' means unobserved or clear):\n")
    print(render_ring(ring, converter.max_cm))

    if args.send:
        from obstacle_ring import send_obstacle_distance
        from pymavlink import mavutil
        master = mavutil.mavlink_connection(args.connect, baud=args.baud)
        master.wait_heartbeat()
        send_obstacle_distance(master, ring, converter.min_cm, converter.max_cm)
        print("\nsent one OBSTACLE_DISTANCE message")


if __name__ == "__main__":
    main()
