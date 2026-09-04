#!/usr/bin/env python3
"""
Checks whether the connected autopilot actually supports obstacle avoidance.

This matters because your Pixhawk 2.4.8 may be a 1 MB flash board, and the
1 MB ArduPilot builds have optional features compiled out to make room.
Object avoidance is exactly the kind of thing that gets cut.

Run this right after flashing. If OA_TYPE is missing, no amount of clever
computer vision will help -- the firmware physically cannot act on it.

Usage
-----
    python check_avoidance_support.py
    python check_avoidance_support.py --connect COM5 --baud 115200
"""

import argparse

from pymavlink import mavutil

# Parameters we need, and what each one is for.
REQUIRED = [
    ("OA_TYPE", "Path planning around obstacles in AUTO/Guided. THE critical one."),
    ("AVOID_ENABLE", "Simple stop-short avoidance in Loiter/PosHold."),
]

# Proximity parameter naming changed across versions, so accept either.
EITHER_OF = [
    (("PRX1_TYPE", "PRX_TYPE"), "Accepts proximity data over MAVLink."),
]

NICE_TO_HAVE = [
    ("OA_MARGIN_MAX", "How far to stay off obstacles."),
    ("OA_BR_LOOKAHEAD", "BendyRuler lookahead distance."),
]


def fetch_param(master, name, timeout=2.0):
    """Ask for one parameter. Returns its value, or None if it does not exist."""
    master.mav.param_request_read_send(
        master.target_system, master.target_component, name.encode("utf-8"), -1)

    msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=timeout)
    while msg is not None:
        if msg.param_id.strip("\x00") == name:
            return msg.param_value
        # Not the one we asked for (params stream in the background); keep looking.
        msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=timeout)
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--connect", default="udpin:127.0.0.1:14551")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()

    print("Connecting to {} ...".format(args.connect))
    master = mavutil.mavlink_connection(args.connect, baud=args.baud)
    master.wait_heartbeat()
    print("Connected.\n")

    ok = True

    print("REQUIRED")
    for name, why in REQUIRED:
        value = fetch_param(master, name)
        if value is None:
            print("  [MISSING] {:<16} {}".format(name, why))
            ok = False
        else:
            print("  [ok]      {:<16} = {:g}".format(name, value))

    for names, why in EITHER_OF:
        found = None
        for name in names:
            value = fetch_param(master, name)
            if value is not None:
                found = (name, value)
                break
        if found is None:
            print("  [MISSING] {:<16} {}".format("/".join(names), why))
            ok = False
        else:
            print("  [ok]      {:<16} = {:g}".format(found[0], found[1]))

    print("\nNICE TO HAVE")
    for name, why in NICE_TO_HAVE:
        value = fetch_param(master, name)
        status = "[ok]     " if value is not None else "[missing]"
        shown = "= {:g}".format(value) if value is not None else why
        print("  {} {:<16} {}".format(status, name, shown))

    print()
    if ok:
        print("PASS -- this firmware can do obstacle avoidance. Continue to Step 3.")
    else:
        print("FAIL -- required parameters are missing.")
        print("Most likely you are on the 1 MB 'Pixhawk1-1M' build with features")
        print("stripped out. Options:")
        print("  1. Check whether your board actually has 2 MB and reflash 'Pixhawk1'.")
        print("  2. Replace the flight controller (Pixhawk 6C ~$200, Matek H743 ~$120).")
        print("     Your frame, motors, ESCs, battery and GPS all carry over.")


if __name__ == "__main__":
    main()
