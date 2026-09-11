#!/usr/bin/env python3
"""
Fly the obstacle course in SITL, publishing the obstacles and flying the mission
over a single MAVLink connection.

Earlier attempts split these into two processes on two ports, which needs SITL's
spare TCP ports (5762/5763) to exist -- and they do not always. One connection
driven by one 10 Hz loop avoids the problem entirely, and pymavlink connections
are not thread-safe anyway, so single-threaded is the right shape regardless.

    python3 sim/run_course.py

What it does, in order: upload a straight mission through the course, arm (which
requires proximity data to already be flowing -- see docs/03-gazebo.md), take
off, switch to AUTO, then log the track and measure how close it came to every
pillar.

The obstacles must match what Gazebo is showing. Both read sim/gazebo/course.txt,
so they cannot drift apart; regenerate the world with make_world.py after editing
it.
"""

import argparse
import math
import os
import sys
import time

from pymavlink import mavutil

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "src"))

_vop = os.path.join(os.path.dirname(HERE), "src", "virtual_obstacle_publisher.py")
_ns = {}
exec(compile(open(_vop).read().replace("from pymavlink import mavutil",
                                       "from pymavlink import mavutil"),
             _vop, "exec"), _ns)
build_ring = _ns["build_ring"]
SECTOR_WIDTH_DEG = _ns["SECTOR_WIDTH_DEG"]


def read_course(path):
    obstacles = []
    with open(path) as fh:
        for lineno, raw in enumerate(fh, 1):
            line = raw.split("#", 1)[0].strip()
            if not line:
                continue
            parts = line.split()
            if len(parts) != 3:
                sys.exit("%s:%d: expected 'north east radius'" % (path, lineno))
            obstacles.append(tuple(float(p) for p in parts))
    return obstacles


class Runner(object):
    def __init__(self, args):
        self.a = args
        self.obstacles = read_course(args.course)
        self.x = self.y = self.z = self.yaw = 0.0
        self.armed = False
        self.texts = []
        self.track = []
        self.closest = dict((i, 1e9) for i in range(len(self.obstacles)))

        print("connecting to %s ..." % args.connect)
        self.m = mavutil.mavlink_connection(args.connect)
        self.m.wait_heartbeat()
        print("connected, sys %d\n" % self.m.target_system)
        self.m.mav.request_data_stream_send(
            self.m.target_system, self.m.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)

    # -- telemetry ---------------------------------------------------------
    def drain(self):
        while True:
            msg = self.m.recv_match(blocking=False)
            if msg is None:
                return
            t = msg.get_type()
            if t == "LOCAL_POSITION_NED":
                self.x, self.y, self.z = msg.x, msg.y, msg.z
                self.track.append((self.x, self.y))
                for i, (ox, oy, r) in enumerate(self.obstacles):
                    d = math.hypot(ox - self.x, oy - self.y) - r
                    if d < self.closest[i]:
                        self.closest[i] = d
            elif t == "ATTITUDE":
                self.yaw = msg.yaw
            elif t == "HEARTBEAT" and msg.get_srcComponent() == 1:
                self.armed = bool(msg.base_mode &
                                  mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            elif t == "STATUSTEXT":
                self.texts.append(msg.text)

    def publish(self):
        ring, min_cm, max_cm = build_ring(self.obstacles, self.x, self.y, self.yaw,
                                          self.a.fov, 0.2, self.a.max_range)
        self.m.mav.obstacle_distance_send(
            int(time.monotonic() * 1e6),
            mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER,
            ring, 0, min_cm, max_cm, SECTOR_WIDTH_DEG, 0.0,
            mavutil.mavlink.MAV_FRAME_BODY_FRD)
        return ring, max_cm

    def pump(self, seconds, until=None, log_every=None):
        """Run the 10 Hz loop: drain, publish, optionally log, until a predicate."""
        t0 = time.time()
        next_log = 0.0
        while time.time() - t0 < seconds:
            self.drain()
            ring, max_cm = self.publish()
            el = time.time() - t0
            if log_every and el >= next_log:
                next_log = el + log_every
                lit = [v for v in ring if v <= max_cm]
                print("  %5.0fs  N %6.1f  E %6.1f  alt %5.1f  %2d sectors  "
                      "nearest %s" %
                      (el, self.x, self.y, -self.z, len(lit),
                       "%5.1f m" % (min(lit) / 100.0) if lit else "    -"))
            if until is not None and until():
                return True
            time.sleep(0.1)
        return False

    # -- mission -----------------------------------------------------------
    def upload_mission(self):
        g = None
        while g is None:
            self.drain()
            g = self.m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=5)
        lat0, lon0 = g.lat / 1e7, g.lon / 1e7

        def offset(north, east):
            return (lat0 + north / 111320.0,
                    lon0 + east / (111320.0 * math.cos(math.radians(lat0))))

        items = [(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0.0, 0.0, self.a.alt),
                 (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, self.a.north, 0.0, self.a.alt)]

        self.m.mav.mission_count_send(self.m.target_system, self.m.target_component,
                                      len(items) + 1,
                                      mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        sent, t0 = 0, time.time()
        while sent < len(items) + 1 and time.time() - t0 < 25:
            req = self.m.recv_match(type=["MISSION_REQUEST", "MISSION_REQUEST_INT"],
                                    blocking=True, timeout=4)
            if req is None:
                self.publish()
                continue
            if req.seq == 0:
                lat, lon, alt = lat0, lon0, 0.0
                cmd = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
            else:
                cmd, north, east, alt = items[req.seq - 1]
                lat, lon = offset(north, east)
            self.m.mav.mission_item_int_send(
                self.m.target_system, self.m.target_component, req.seq,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                cmd, 0, 1, 0, 0, 0, 0,
                int(lat * 1e7), int(lon * 1e7), alt,
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
            sent += 1
        ack = self.m.recv_match(type="MISSION_ACK", blocking=True, timeout=8)
        return ack.type if ack else None

    def run(self):
        print("course: %d pillars" % len(self.obstacles))
        for (n, e, r) in self.obstacles:
            print("  north %6.1f  east %6.1f  r %.1f" % (n, e, r))
        print("mission: straight to %.0f m north at %.0f m altitude" %
              (self.a.north, self.a.alt))
        print("sensor : %.0f deg FOV, %.0f m range\n" % (self.a.fov, self.a.max_range))

        print("priming proximity data ...")
        self.pump(3.0)

        print("uploading mission ... ", end="")
        print("ack=%s\n" % self.upload_mission())

        self.m.mav.mission_set_current_send(self.m.target_system,
                                            self.m.target_component, 1)
        self.pump(1.0)

        print("GUIDED + arm")
        self.m.set_mode_apm("GUIDED")
        self.pump(2.0)

        # Straight after a reboot the EKF is still aligning and the GPS driver is
        # still probing, so the first arm attempt is usually refused with
        # "Accels inconsistent" or "GPS still configuring". Those clear on their
        # own within a minute, so retry rather than give up.
        deadline = time.time() + self.a.arm_timeout
        attempt = 0
        while time.time() < deadline and not self.armed:
            attempt += 1
            self.m.mav.command_long_send(self.m.target_system, self.m.target_component,
                                         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                         0, 1, 0, 0, 0, 0, 0, 0)
            if self.pump(8.0, until=lambda: self.armed):
                break
            refusals = [t for t in self.texts if "rm:" in t]
            print("  attempt %d refused: %s"
                  % (attempt, refusals[-1] if refusals else "no reason given"))
        if not self.armed:
            print("  FAILED to arm after %.0f s:" % self.a.arm_timeout)
            for t in self.texts[-10:]:
                print("    %s" % t)
            return 1
        print("  armed\n")

        print("takeoff to %.0f m" % self.a.alt)
        self.m.mav.command_long_send(self.m.target_system, self.m.target_component,
                                     mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                                     0, 0, 0, 0, 0, 0, 0, self.a.alt)
        self.pump(40.0, until=lambda: -self.z > self.a.alt * 0.92)
        print("  at %.1f m\n" % -self.z)

        print("AUTO -- flying the course")
        self.m.set_mode_apm("AUTO")
        print("      t      north    east   alt  sectors   nearest")
        print("  ---------------------------------------------------")
        reached = self.pump(self.a.timeout,
                            until=lambda: self.x > self.a.north - 3.0,
                            log_every=4.0)

        print("\n%s" % ("waypoint reached" if reached else "TIMED OUT"))
        max_east = max((abs(y) for _, y in self.track), default=0.0)
        print("max lateral deviation : %.2f m" % max_east)
        print("closest approach to each pillar (surface distance):")
        worst = 1e9
        for i, (n, e, r) in enumerate(self.obstacles):
            d = self.closest[i]
            worst = min(worst, d)
            flag = "  <-- BREACH" if d < 0 else ""
            print("  obs_%-2d north %6.1f east %6.1f : %6.2f m%s" % (i + 1, n, e, d, flag))
        print("\nworst clearance: %.2f m  (margin asked for: 2.00 m)" % worst)
        return 0 if (reached and worst > 0) else 1


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--connect", default="tcp:127.0.0.1:5760")
    p.add_argument("--course", default=os.path.join(HERE, "gazebo", "course.txt"))
    p.add_argument("--north", type=float, default=130.0)
    p.add_argument("--alt", type=float, default=10.0)
    p.add_argument("--fov", type=float, default=72.0)
    p.add_argument("--max-range", type=float, default=20.0)
    p.add_argument("--timeout", type=float, default=200.0)
    p.add_argument("--arm-timeout", type=float, default=120.0,
                   help="how long to keep retrying the arm while the EKF settles")
    sys.exit(Runner(p.parse_args()).run())


if __name__ == "__main__":
    main()
