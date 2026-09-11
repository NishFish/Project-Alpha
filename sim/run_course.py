#!/usr/bin/env python3
"""
Fly the obstacle course in SITL, publishing obstacle data and flying the mission
over a single MAVLink connection.

    python3 sim/run_course.py                          # computed geometry
    python3 sim/run_course.py --source depth --min-range 1.5    # real camera
    python3 sim/run_course.py --source depth --min-range 1.5 --noise

Two perception sources, the same everything else:

  virtual  the ring is computed from known obstacle positions. Proves the
           control path without involving a sensor at all.
  depth    a Gazebo depth camera through src/depth_to_obstacle_ring.py -- the
           same file that will run on the real OAK-D. The flight loop is told
           nothing about where anything is; course.txt is read only to score
           the run afterwards.

--noise corrupts the depth frame with a derived stereo error model before the
conversion sees it. Gazebo's depth is perfect, which makes the percentile and
temporal filters look pointless; they are not, and this is where that gets
tested.

One connection, one 10 Hz loop. Splitting the publisher and the controller across
two ports needs SITL's spare 5762/5763, which do not always exist, and pymavlink
connections are not thread-safe anyway.
"""

import argparse
import math
import os
import sys
import time

from pymavlink import mavutil

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(os.path.dirname(HERE), "src")
sys.path.insert(0, SRC)

from course import SECTOR_WIDTH_DEG, load, ring_from_course


class Runner(object):
    def __init__(self, args):
        self.a = args
        self.crs = load(args.course)
        self.x = self.y = self.z = self.yaw = 0.0
        self.pitch = self.roll = 0.0
        self.armed = False
        self.texts = []
        self.track = []
        self.closest = dict((i, 1e9) for i in range(len(self.crs.shapes)))
        self.blind_frames = 0
        self.total_frames = 0

        print("connecting to %s ..." % args.connect)
        self.m = mavutil.mavlink_connection(args.connect)
        self.m.wait_heartbeat()
        print("connected, sys %d" % self.m.target_system)
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
                for i, shp in enumerate(self.crs.shapes):
                    d = shp.distance(self.x, self.y)
                    if d < self.closest[i]:
                        self.closest[i] = d
            elif t == "ATTITUDE":
                self.yaw, self.pitch, self.roll = msg.yaw, msg.pitch, msg.roll
            elif t == "HEARTBEAT" and msg.get_srcComponent() == 1:
                self.armed = bool(msg.base_mode &
                                  mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            elif t == "STATUSTEXT":
                self.texts.append(msg.text)

    # -- perception --------------------------------------------------------
    def start_depth(self):
        """Subscribe to the Gazebo depth camera and build the real converter.

        Gazebo's Python bindings ship with gz-harmonic, so no ROS is involved.
        """
        import numpy as np
        from gz.transport13 import Node
        from gz.msgs10.image_pb2 import Image
        from depth_to_obstacle_ring import DepthToRing

        self._np = np
        self._frame = None

        if self.a.noise:
            from stereo_noise import corrupt, OAKD_LITE_BASELINE_M
            self._corrupt = corrupt
            self._baseline = OAKD_LITE_BASELINE_M
            self._rng = np.random.default_rng(self.a.seed)

        w, h = self.a.cam_width, self.a.cam_height
        self._fx = (w / 2.0) / math.tan(math.radians(self.a.fov) / 2.0)
        self.converter = DepthToRing(
            self._fx, self._fx, (w - 1) / 2.0, (h - 1) / 2.0, w, h,
            min_range_m=self.a.min_range, max_range_m=self.a.max_range,
            height_band_m=self.a.height_band, stride=2)

        def on_image(msg):
            if msg.width == w and msg.height == h:
                self._frame = msg.data

        self._node = Node()
        if not self._node.subscribe(Image, self.a.topic, on_image):
            sys.exit("could not subscribe to %s" % self.a.topic)
        print("subscribed to %s (%dx%d, fx=%.1f)" % (self.a.topic, w, h, self._fx))

        t0 = time.time()
        while self._frame is None and time.time() - t0 < 15:
            time.sleep(0.2)
        if self._frame is None:
            sys.exit("no depth frames on %s -- is Gazebo running the "
                     "obstacle_course world?" % self.a.topic)
        print("first depth frame received")

    def ring_from_depth(self):
        depth = self._np.frombuffer(self._frame, dtype=self._np.float32).reshape(
            self.a.cam_height, self.a.cam_width)
        if self.a.noise:
            depth = self._corrupt(depth, self._fx, baseline_m=self._baseline,
                                  max_range_m=self.a.max_range, rng=self._rng)
        ring = self.converter.process(depth, pitch_rad=self.pitch,
                                      roll_rad=self.roll)
        return ring, self.converter.min_cm, self.converter.max_cm

    def publish(self):
        if self.a.source == "depth":
            ring, min_cm, max_cm = self.ring_from_depth()
        else:
            ring, min_cm, max_cm = ring_from_course(
                self.crs, self.x, self.y, self.yaw,
                self.a.fov, self.a.min_range, self.a.max_range)

        self.m.mav.obstacle_distance_send(
            int(time.monotonic() * 1e6),
            mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER,
            ring, 0, min_cm, max_cm, SECTOR_WIDTH_DEG, 0.0,
            mavutil.mavlink.MAV_FRAME_BODY_FRD)
        return ring, max_cm

    def pump(self, seconds, until=None, log_every=None):
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
            g = self.m.recv_match(type="GLOBAL_POSITION_INT", blocking=True,
                                  timeout=5)
        lat0, lon0 = g.lat / 1e7, g.lon / 1e7

        def offset(north, east):
            return (lat0 + north / 111320.0,
                    lon0 + east / (111320.0 * math.cos(math.radians(lat0))))

        gn, ge = self.crs.goal
        items = [(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0.0, 0.0, self.a.alt),
                 (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, gn, ge, self.a.alt)]

        self.m.mav.mission_count_send(self.m.target_system,
                                      self.m.target_component, len(items) + 1,
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

    def reset_scoring(self):
        """Clear per-run measurements so each flight is scored on its own."""
        self.track = []
        self.closest = dict((i, 1e9) for i in range(len(self.crs.shapes)))
        self.texts = []

    def return_to_start(self):
        """Fly home and land, ready for another run. Returns True on success.

        Uses RTL rather than a hand-rolled goto-then-land. The first version of
        this landed wherever it happened to be once its timeout expired, which
        put the aircraft against a wall at 68 degrees nose-up and made every
        subsequent run fail `Arm: Leaning` without moving. Landing anywhere but
        home is never the right answer -- if it cannot get back, the batch has
        to stop and say so.

        The return leg keeps avoidance live, and is deliberately not scored: a
        run means start to goal.
        """
        sn, se = self.crs.start
        print("  RTL ...")
        self.m.set_mode_apm("RTL")
        self.pump(2.0)

        if not self.pump(self.a.rtl_timeout, until=lambda: not self.armed):
            print("  RTL did not finish within %.0f s" % self.a.rtl_timeout)
            return False

        drift = math.hypot(sn - self.x, se - self.y)
        print("  disarmed at N %.1f E %.1f, %.1f m from start" % (self.x, self.y, drift))
        if drift > self.a.home_tolerance:
            print("  landed %.1f m from the start (tolerance %.1f m)"
                  % (drift, self.a.home_tolerance))
            return False
        return True

    # -- run ---------------------------------------------------------------
    def run(self):
        print()
        print(self.crs.describe())
        gn, ge = self.crs.goal
        print()
        print("mission : straight to north %.0f east %.0f at %.0f m" %
              (gn, ge, self.a.alt))
        print("sensor  : %.0f deg FOV, %.1f-%.0f m" %
              (self.a.fov, self.a.min_range, self.a.max_range))

        if self.a.source == "depth":
            print("percept : REAL depth camera%s, no obstacle positions given"
                  % (", WITH stereo noise" if self.a.noise else ""))
            self.start_depth()
        else:
            print("percept : computed from known obstacle positions")

        if self.a.runs > 1:
            print("runs    : %d, scored together" % self.a.runs)
            return self.batch()
        print()
        return self.fly_once()

    def batch(self):
        results = []
        for i in range(self.a.runs):
            print()
            print("=" * 60)
            print("RUN %d of %d" % (i + 1, self.a.runs))
            print("=" * 60)
            self.reset_scoring()
            t0 = time.time()
            rc = self.fly_once(verbose=False)
            worst = min((self.closest[k] for k in self.closest), default=float("inf"))
            lat = max((abs(y) for _, y in self.track), default=0.0)
            results.append({"rc": rc, "worst": worst, "lat": lat,
                            "secs": time.time() - t0,
                            "closest": dict(self.closest)})
            print("  run %d: %s, worst clearance %.2f m, max deviation %.2f m"
                  % (i + 1, "reached" if rc == 0 else "FAILED", worst, lat))
            if i < self.a.runs - 1:
                if not self.return_to_start():
                    # Continuing from a bad position produces a column of
                    # identical bogus failures that look like an avoidance
                    # problem and are not. Stop and say what happened.
                    print()
                    print("  ABORTING after run %d: could not get back to the "
                          "start." % (i + 1))
                    print("  The aircraft is at N %.1f E %.1f, %.1f m from the "
                          "nearest obstacle surface."
                          % (self.x, self.y, self.crs.nearest_surface(self.x, self.y)))
                    print("  Restart the simulator before running again.")
                    break

        print()
        print("=" * 60)
        print("BATCH SUMMARY -- %d runs" % len(results))
        print("=" * 60)
        print("  run   result    worst clearance   max deviation   time")
        for i, r in enumerate(results, 1):
            print("  %3d   %-8s  %13.2f m  %13.2f m  %5.0fs"
                  % (i, "ok" if r["rc"] == 0 else "FAIL", r["worst"], r["lat"],
                     r["secs"]))

        reached = sum(1 for r in results if r["rc"] == 0)
        worsts = [r["worst"] for r in results]
        breaches = [i for i, r in enumerate(results, 1) if r["worst"] <= 0]

        print()
        print("  goal reached      : %d of %d" % (reached, len(results)))
        print("  worst clearance   : %.2f m  (margin asked for %.2f m)"
              % (min(worsts), self.a.margin))
        print("  median clearance  : %.2f m" % sorted(worsts)[len(worsts) // 2])
        print("  spread            : %.2f m to %.2f m" % (min(worsts), max(worsts)))
        print("  collisions        : %d" % len(breaches))

        # Which obstacle is the marginal one, across every run.
        per_obs = []
        for k, shp in enumerate(self.crs.shapes):
            vals = [r["closest"][k] for r in results]
            per_obs.append((min(vals), k, shp))
        per_obs.sort()
        print()
        print("  tightest obstacles across all runs:")
        for worst_d, k, shp in per_obs[:4]:
            print("    obs_%-2d %-22s %6.2f m" % (k + 1, shp.describe(), worst_d))

        ok = (reached == len(results)) and not breaches
        print()
        print("  GATE: %s" % ("PASSED -- %d runs, zero collisions" % len(results)
                              if ok else "FAILED"))
        return 0 if ok else 1

    def fly_once(self, verbose=True):
        gn, ge = self.crs.goal
        near = self.crs.nearest_surface(self.x, self.y)
        if near < 1.0:
            print("  refusing to start: the aircraft is %.2f m from an "
                  "obstacle surface at N %.1f E %.1f" % (near, self.x, self.y))
            return 1
        print()
        print("priming proximity data ...")
        self.pump(3.0)

        print("uploading mission ... ", end="")
        print("ack=%s" % self.upload_mission())

        self.m.mav.mission_set_current_send(self.m.target_system,
                                            self.m.target_component, 1)
        self.pump(1.0)

        print("GUIDED + arm")
        self.m.set_mode_apm("GUIDED")
        self.pump(2.0)

        # Straight after a reboot the EKF is still aligning and the GPS driver
        # still probing. Those refusals clear on their own, so retry.
        deadline = time.time() + self.a.arm_timeout
        attempt = 0
        while time.time() < deadline and not self.armed:
            attempt += 1
            self.m.mav.command_long_send(
                self.m.target_system, self.m.target_component,
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
        print("  armed")

        print("takeoff to %.0f m" % self.a.alt)
        self.m.mav.command_long_send(
            self.m.target_system, self.m.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, self.a.alt)
        self.pump(45.0, until=lambda: -self.z > self.a.alt * 0.92)
        print("  at %.1f m" % -self.z)

        print()
        print("AUTO -- flying the course")
        self.m.set_mode_apm("AUTO")
        print("      t      north    east   alt  sectors   nearest")
        print("  ---------------------------------------------------")

        def arrived():
            return math.hypot(gn - self.x, ge - self.y) < 4.0

        reached = self.pump(self.a.timeout, until=arrived, log_every=4.0)

        print()
        print("%s" % ("GOAL REACHED" if reached else "TIMED OUT"))
        worst, worst_i = 1e9, None
        for i in range(len(self.crs.shapes)):
            if self.closest[i] < worst:
                worst, worst_i = self.closest[i], i + 1

        if verbose:
            max_lat = max((abs(y) for _, y in self.track), default=0.0)
            print("max lateral deviation : %.2f m" % max_lat)
            print("closest approach to each obstacle (surface distance):")
            for i, shp in enumerate(self.crs.shapes):
                d = self.closest[i]
                flag = "  <-- BREACH" if d < 0 else ""
                print("  obs_%-2d north %6.1f east %6.1f  %-22s %6.2f m%s"
                      % (i + 1, shp.north, shp.east, shp.describe(), d, flag))
            print()
            print("worst clearance: %.2f m at obs_%s  (margin asked for: %.2f m)"
                  % (worst, worst_i, self.a.margin))
        return 0 if (reached and worst > 0) else 1


def main():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--connect", default="tcp:127.0.0.1:5760")
    p.add_argument("--course", default=os.path.join(HERE, "gazebo", "course.txt"))
    p.add_argument("--source", choices=("virtual", "depth"), default="virtual")
    p.add_argument("--noise", action="store_true",
                   help="corrupt the depth frame with a stereo error model "
                        "before converting it (depth source only)")
    p.add_argument("--seed", type=int, default=None,
                   help="seed for the noise, so a run can be repeated exactly")
    p.add_argument("--topic", default="/depth_camera")
    p.add_argument("--cam-width", type=int, default=320)
    p.add_argument("--cam-height", type=int, default=240)
    p.add_argument("--alt", type=float, default=10.0)
    p.add_argument("--fov", type=float, default=72.0)
    p.add_argument("--min-range", type=float, default=0.5)
    p.add_argument("--max-range", type=float, default=20.0)
    p.add_argument("--height-band", type=float, default=0.5)
    p.add_argument("--margin", type=float, default=2.0,
                   help="only used for reporting; the autopilot's own value is "
                        "OA_MARGIN_MAX")
    p.add_argument("--timeout", type=float, default=260.0)
    p.add_argument("--runs", type=int, default=1,
                   help="fly the course this many times, returning to the "
                        "start between them, and score them together. "
                        "BendyRuler is not deterministic, so one clean "
                        "flight proves very little.")
    p.add_argument("--rtl-timeout", type=float, default=240.0,
                   help="how long RTL gets to fly home and land between runs")
    p.add_argument("--home-tolerance", type=float, default=8.0,
                   help="how far from the start a landing may be and still "
                        "count as home")
    p.add_argument("--arm-timeout", type=float, default=120.0)
    args = p.parse_args()

    if args.noise and args.source != "depth":
        p.error("--noise only applies to --source depth")
    sys.exit(Runner(args).run())


if __name__ == "__main__":
    main()
