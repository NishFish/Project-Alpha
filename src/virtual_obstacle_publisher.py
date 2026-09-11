#!/usr/bin/env python3
"""
Publish OBSTACLE_DISTANCE for obstacles fixed in the WORLD, not the body frame.

`fake_obstacle_publisher.py` reports a wall a constant distance straight ahead,
which is all you need to prove the plumbing works -- but it is useless for
testing avoidance, because the wall turns with the aircraft. Nothing can fly
around an obstacle that follows it.

This places obstacles at fixed positions relative to home, reads the vehicle's
own position and yaw, and works out what a forward-facing sensor would see. So
the aircraft can actually get past them, which is what makes a deviation test
mean anything.

Still no camera involved -- the geometry is computed from known obstacle
positions. It sits between fake_obstacle_publisher.py (plumbing) and
depth_to_obstacle_ring.py (real sensor).

    # two obstacles, 30 m and 55 m north of home
    python3 virtual_obstacle_publisher.py --obstacle 30,0,4 --obstacle 55,12,4

Connect over one of SITL's spare TCP ports so this does not fight anything
already bound to UDP 14551.
"""

import argparse
import math
import time

from pymavlink import mavutil

NUM_SECTORS = 72
SECTOR_WIDTH_DEG = 360.0 / NUM_SECTORS


def parse_obstacle(text):
    """'north,east,radius' in metres relative to home."""
    parts = text.split(",")
    if len(parts) != 3:
        raise argparse.ArgumentTypeError(
            "expected north,east,radius (e.g. 30,0,4), got %r" % text)
    return tuple(float(p) for p in parts)


def build_ring(obstacles, x, y, yaw, fov_deg, min_m, max_m):
    """What a forward-facing sensor at (x, y, yaw) would report, in cm.

    x is north and y is east, matching LOCAL_POSITION_NED. Sector 0 points
    along the aircraft's nose because we send MAV_FRAME_BODY_FRD, and the
    index increases clockwise.
    """
    min_cm, max_cm = int(min_m * 100), int(max_m * 100)
    empty = max_cm + 1
    ring = [empty] * NUM_SECTORS
    half_fov = fov_deg / 2.0

    for (ox, oy, radius) in obstacles:
        dx, dy = ox - x, oy - y
        centre_dist = math.hypot(dx, dy)
        if centre_dist <= radius:
            continue                      # already inside it; nothing useful to say

        surface = centre_dist - radius
        if not (min_m <= surface <= max_m):
            continue

        # Bearing relative to the nose, wrapped to +/-180.
        rel = math.degrees(math.atan2(dy, dx) - yaw)
        rel = (rel + 540.0) % 360.0 - 180.0

        # How wide the obstacle looks from here.
        half_width = math.degrees(math.asin(min(1.0, radius / centre_dist)))

        steps = int(half_width / SECTOR_WIDTH_DEG) + 1
        for k in range(-steps, steps + 1):
            bearing = rel + k * SECTOR_WIDTH_DEG
            if abs(bearing - rel) > half_width:
                continue
            if abs(bearing) > half_fov:          # outside what the camera sees
                continue
            idx = int(round(bearing / SECTOR_WIDTH_DEG)) % NUM_SECTORS
            ring[idx] = min(ring[idx], int(surface * 100))

    return ring, min_cm, max_cm


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--connect", default="tcp:127.0.0.1:5762")
    parser.add_argument("--obstacle", action="append", type=parse_obstacle,
                        metavar="N,E,R", default=None,
                        help="obstacle as north,east,radius in metres from home; repeatable")
    parser.add_argument("--fov", type=float, default=72.0,
                        help="sensor field of view in degrees (OAK-D Lite is 72)")
    parser.add_argument("--min-range", type=float, default=0.2)
    parser.add_argument("--max-range", type=float, default=20.0)
    parser.add_argument("--rate", type=float, default=10.0)
    args = parser.parse_args()

    obstacles = args.obstacle or [(30.0, 0.0, 4.0)]

    print("connecting to %s ..." % args.connect)
    m = mavutil.mavlink_connection(args.connect)
    m.wait_heartbeat()
    print("connected, sys %d" % m.target_system)

    # The spare TCP ports send nothing until asked.
    m.mav.request_data_stream_send(m.target_system, m.target_component,
                                   mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)

    print("obstacles (north, east, radius):")
    for o in obstacles:
        print("  %6.1f, %6.1f  r=%.1f m" % o)
    print("sensor: %.0f deg FOV, %.1f m range" % (args.fov, args.max_range))
    print("publishing OBSTACLE_DISTANCE at %.0f Hz\n" % args.rate)

    x = y = yaw = 0.0
    period = 1.0 / args.rate
    sent = 0
    next_report = time.monotonic() + 2.0

    try:
        while True:
            # Drain whatever has arrived; keep the freshest pose.
            while True:
                msg = m.recv_match(type=["LOCAL_POSITION_NED", "ATTITUDE"],
                                   blocking=False)
                if msg is None:
                    break
                if msg.get_type() == "LOCAL_POSITION_NED":
                    x, y = msg.x, msg.y
                else:
                    yaw = msg.yaw

            ring, min_cm, max_cm = build_ring(obstacles, x, y, yaw,
                                              args.fov, args.min_range, args.max_range)

            m.mav.obstacle_distance_send(
                int(time.monotonic() * 1e6),
                mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER,
                ring, 0, min_cm, max_cm,
                SECTOR_WIDTH_DEG, 0.0,
                mavutil.mavlink.MAV_FRAME_BODY_FRD)
            sent += 1

            now = time.monotonic()
            if now >= next_report:
                next_report = now + 2.0
                lit = [i for i, v in enumerate(ring) if v <= max_cm]
                nearest = min([ring[i] for i in lit], default=None)
                print("  pos N%7.1f E%7.1f  yaw %6.1f deg  |  %2d sectors, "
                      "nearest %s" % (x, y, math.degrees(yaw),
                                      len(lit),
                                      "%.1f m" % (nearest / 100.0) if nearest else "-"))

            time.sleep(period)
    except KeyboardInterrupt:
        print("\nstopped after %d messages. ArduPilot now has stale proximity "
              "data and will fly on with NO avoidance." % sent)


if __name__ == "__main__":
    main()
