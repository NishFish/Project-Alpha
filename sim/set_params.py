#!/usr/bin/env python3
"""
Apply the parameters the obstacle course needs, and reboot only if something
that requires it actually changed.

Worth having as its own script because `sim_vehicle.py` keeps `eeprom.bin` in
whatever directory it was launched from. Start it somewhere new and every
parameter silently reverts to default -- `FRAME_CLASS` back to 0, avoidance off
-- with no warning at all. Running this is cheap and idempotent, so do it on
every startup rather than trying to remember whether it is needed.

    python3 sim/set_params.py
"""

import argparse
import sys
import time

from pymavlink import mavutil

# Reboot is only needed for parameters ArduPilot reads once at startup.
NEEDS_REBOOT = {"FRAME_CLASS", "FRAME_TYPE", "SERIAL1_PROTOCOL", "SERIAL2_PROTOCOL"}

WANTED = [
    ("FRAME_CLASS",   1),    # quad -- without this it refuses to arm
    ("FRAME_TYPE",    1),    # X
    ("PRX1_TYPE",     2),    # accept proximity over MAVLink
    ("OA_TYPE",       1),    # BendyRuler path planning in AUTO and GUIDED
    ("OA_MARGIN_MAX", 2),    # metres of clearance to hold
    ("AVOID_ENABLE",  7),    # stop-short avoidance in Loiter/PosHold
]

# Speed cap. The name and units changed: stable releases use WPNAV_SPEED in
# cm/s, ArduCopter 4.8-dev uses WP_SPD in m/s. Try each in turn.
SPEED_ALTERNATIVES = [("WP_SPD", 2.0), ("WPNAV_SPEED", 200.0)]


class Params(object):
    def __init__(self, conn):
        print("connecting to %s ..." % conn)
        self.m = mavutil.mavlink_connection(conn)
        self.m.wait_heartbeat()
        print("connected, sys %d\n" % self.m.target_system)

    def get(self, name, timeout=3.0):
        self.m.mav.param_request_read_send(
            self.m.target_system, self.m.target_component, name.encode(), -1)
        t0 = time.time()
        while time.time() - t0 < timeout:
            msg = self.m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
            if msg and msg.param_id.strip("\x00") == name:
                return msg.param_value
        return None

    def set(self, name, value, tries=5):
        """Returns 'unchanged', 'set', or 'missing'."""
        current = self.get(name)
        if current is None:
            return "missing"
        if abs(current - value) < 1e-4:
            return "unchanged"
        for _ in range(tries):
            self.m.mav.param_set_send(
                self.m.target_system, self.m.target_component, name.encode(),
                float(value), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            time.sleep(0.3)
            got = self.get(name, 2.0)
            if got is not None and abs(got - value) < 1e-4:
                return "set"
        return "failed"

    def reboot(self):
        self.m.mav.command_long_send(
            self.m.target_system, self.m.target_component,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
            0, 1, 0, 0, 0, 0, 0, 0)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--connect", default="tcp:127.0.0.1:5760")
    args = p.parse_args()

    pr = Params(args.connect)
    reboot_needed = False
    failures = []

    for name, value in WANTED:
        result = pr.set(name, value)
        print("  %-16s %-5g %s" % (name, value, result))
        if result == "set" and name in NEEDS_REBOOT:
            reboot_needed = True
        if result in ("failed", "missing"):
            failures.append(name)

    for name, value in SPEED_ALTERNATIVES:
        result = pr.set(name, value)
        if result != "missing":
            print("  %-16s %-5g %s" % (name, value, result))
            break
    else:
        failures.append("speed cap (no known name matched)")

    if failures:
        print("\ncould not set: %s" % ", ".join(failures))

    if reboot_needed:
        print("\nrebooting (a startup-only parameter changed) ...")
        pr.reboot()
        time.sleep(12)
        pr2 = mavutil.mavlink_connection(args.connect)
        if pr2.wait_heartbeat(timeout=30) is None:
            sys.exit("no heartbeat after reboot")
        print("back up")
    else:
        print("\nno reboot needed")

    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
