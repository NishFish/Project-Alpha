#!/usr/bin/env python3
"""
STEP 3 -- Fake obstacle publisher.

Pretends there is a wall in front of the drone and tells ArduPilot about it,
10 times a second, using the standard MAVLink OBSTACLE_DISTANCE message.

There is no camera and no real drone involved. The whole point is to prove
that the *control path* works -- that ArduPilot listens to obstacle messages
and steers around them -- before any computer vision exists to complicate it.

Once this works, Step 4 replaces the made-up numbers with numbers from a
camera. Nothing downstream changes.

Usage
-----
    # against SITL (see docs/01-setup.md for the sim_vehicle.py command)
    python fake_obstacle_publisher.py

    # wall at 4 metres instead of the default 5
    python fake_obstacle_publisher.py --distance 4.0

    # against the real Pixhawk over TELEM2, later on
    python fake_obstacle_publisher.py --connect /dev/serial0 --baud 460800
"""

import argparse
import math
import time

from pymavlink import mavutil

from obstacle_ring import (NUM_SECTORS, SECTOR_WIDTH_DEG, empty_ring,
                          send_obstacle_distance)


def build_distance_ring(distance_m, width_deg, min_m, max_m):
    """Build the 72-element distance ring, in centimetres.

    Index 0 points straight ahead (because we send MAV_FRAME_BODY_FRD) and
    the index increases clockwise, so index 1 is 5 degrees to the right and
    index 71 is 5 degrees to the left.

    Sectors with nothing in them are filled with max_distance + 1, which is
    how ArduPilot's own example scripts say "no obstacle here". Do not use 0
    for that -- 0 means "an obstacle is touching the drone".
    """
    min_cm = int(min_m * 100)
    max_cm = int(max_m * 100)

    distances = empty_ring(max_cm)

    # How many sectors on each side of straight-ahead the wall covers.
    half_span = int(round((width_deg / 2.0) / SECTOR_WIDTH_DEG))
    obstacle_cm = int(distance_m * 100)

    # Anything outside the sensor's honest range must stay "empty",
    # otherwise ArduPilot will act on a reading you cannot back up.
    if not (min_cm <= obstacle_cm <= max_cm):
        return distances, min_cm, max_cm

    for offset in range(-half_span, half_span + 1):
        distances[offset % NUM_SECTORS] = obstacle_cm

    return distances, min_cm, max_cm


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--connect", default="udpin:127.0.0.1:14551",
                        help="MAVLink connection string (default: %(default)s)")
    parser.add_argument("--baud", type=int, default=460800,
                        help="baud rate, only used for serial connections")
    parser.add_argument("--distance", type=float, default=5.0,
                        help="how far away to pretend the wall is, in metres")
    parser.add_argument("--width", type=float, default=40.0,
                        help="how wide the wall is, in degrees of field of view")
    parser.add_argument("--min-range", type=float, default=0.2,
                        help="closest distance the pretend sensor can measure, metres")
    parser.add_argument("--max-range", type=float, default=8.0,
                        help="furthest distance the pretend sensor can measure, metres")
    parser.add_argument("--rate", type=float, default=10.0,
                        help="messages per second")
    args = parser.parse_args()

    print("Connecting to {} ...".format(args.connect))
    master = mavutil.mavlink_connection(args.connect, baud=args.baud)

    master.wait_heartbeat()
    print("Connected. system {} component {}".format(master.target_system,
                                                     master.target_component))

    distances, min_cm, max_cm = build_distance_ring(
        args.distance, args.width, args.min_range, args.max_range)

    filled = sum(1 for d in distances if d <= max_cm)
    print("Pretending there is a wall {:.1f} m ahead, {:.0f} degrees wide "
          "({} of {} sectors).".format(args.distance, args.width, filled, NUM_SECTORS))
    print("Sending OBSTACLE_DISTANCE at {:.0f} Hz. Ctrl-C to stop.\n".format(args.rate))

    started = time.monotonic()
    period = 1.0 / args.rate
    sent = 0

    try:
        while True:
            time_usec = int((time.monotonic() - started) * 1e6)

            send_obstacle_distance(master, distances, min_cm, max_cm,
                                   time_usec=time_usec)

            sent += 1
            if sent % int(args.rate) == 0:
                print("\r  sent {} messages".format(sent), end="", flush=True)

            time.sleep(period)
    except KeyboardInterrupt:
        print("\nStopped after {} messages.".format(sent))
        print("NOTE: ArduPilot will now treat the obstacle data as stale and "
              "carry on with NO avoidance. That is the fail-unsafe behaviour "
              "described in the README -- do not forget it.")


if __name__ == "__main__":
    main()
