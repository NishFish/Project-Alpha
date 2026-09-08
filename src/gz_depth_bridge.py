#!/usr/bin/env python3
"""
Bridge a simulated depth camera in Gazebo into ArduPilot's obstacle ring.

Subscribes to a ROS 2 sensor_msgs/Image carrying depth, runs each frame through
the same DepthToRing used for the real camera, and publishes OBSTACLE_DISTANCE
over MAVLink. Attitude is taken from the autopilot so the height filter stays
aligned with gravity.

    python3 gz_depth_bridge.py --topic /depth_camera --connect udpin:127.0.0.1:14551

REQUIRES ROS 2 and a running Gazebo bridge -- see docs/03-gazebo.md. It cannot
run on Windows, and unlike the rest of src/ it has NOT been executed yet: the
maths it calls is covered by test_depth_to_ring.py, but this plumbing is
unverified until you run it. Treat the first run as a bring-up, not a given.

Deliberately avoids cv_bridge. Depth images are a flat buffer of float32 metres
or uint16 millimetres, and numpy reads both in one line, so the dependency buys
nothing but version conflicts.
"""

import argparse
import math
import threading

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from depth_to_obstacle_ring import DepthToRing
from obstacle_ring import TemporalFilter, send_obstacle_distance


def decode_depth(msg):
    """sensor_msgs/Image -> float32 array of metres."""
    if msg.encoding in ("32FC1", "32FC"):
        buf = np.frombuffer(msg.data, dtype=np.float32)
        return buf.reshape(msg.height, msg.width)
    if msg.encoding in ("16UC1", "mono16"):
        buf = np.frombuffer(msg.data, dtype=np.uint16)
        return buf.reshape(msg.height, msg.width).astype(np.float32) / 1000.0
    raise ValueError("unsupported depth encoding: {}".format(msg.encoding))


class Bridge(Node):
    def __init__(self, args, master):
        super().__init__("gz_depth_bridge")
        self.args = args
        self.master = master
        self.converter = None
        self.filter = TemporalFilter(frames=3)
        self.pitch = 0.0
        self.roll = 0.0
        self.frames = 0

        self.create_subscription(Image, args.topic, self.on_depth,
                                 qos_profile_sensor_data)
        self.get_logger().info("subscribed to {}".format(args.topic))

        # ATTITUDE arrives far faster than we need it, so it is read on its own
        # thread and simply latched rather than queued.
        threading.Thread(target=self._attitude_loop, daemon=True).start()

    def _attitude_loop(self):
        while True:
            msg = self.master.recv_match(type="ATTITUDE", blocking=True, timeout=5)
            if msg is not None:
                self.pitch = msg.pitch
                self.roll = msg.roll

    def on_depth(self, msg):
        try:
            depth = decode_depth(msg)
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return

        if self.converter is None:
            h, w = depth.shape
            # A horizontal FOV is the one number a simulated camera always
            # states, and fx follows from it. Read the real value out of the
            # SDF rather than trusting this default.
            fx = (w / 2.0) / math.tan(math.radians(self.args.hfov) / 2.0)
            self.converter = DepthToRing(
                fx, fx, (w - 1) / 2.0, (h - 1) / 2.0, w, h,
                min_range_m=self.args.min_range,
                max_range_m=self.args.max_range,
                height_band_m=self.args.height_band,
                stride=self.args.stride)
            self.get_logger().info(
                "camera {}x{} hfov={} deg -> fx={:.1f}".format(w, h, self.args.hfov, fx))

        ring = self.converter.process(depth, pitch_rad=self.pitch, roll_rad=self.roll)
        ring = self.filter.update(ring)

        send_obstacle_distance(self.master, ring,
                               self.converter.min_cm, self.converter.max_cm)

        self.frames += 1
        if self.frames % 30 == 0:
            self.get_logger().info("{} frames, {}".format(
                self.frames, self.converter.last_stats))


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--topic", default="/depth_camera")
    parser.add_argument("--connect", default="udpin:127.0.0.1:14551")
    parser.add_argument("--baud", type=int, default=460800)
    parser.add_argument("--hfov", type=float, default=72.0,
                        help="camera horizontal field of view in degrees")
    parser.add_argument("--min-range", type=float, default=0.2)
    parser.add_argument("--max-range", type=float, default=6.0)
    parser.add_argument("--height-band", type=float, default=0.5)
    parser.add_argument("--stride", type=int, default=2)
    args = parser.parse_args()

    from pymavlink import mavutil
    print("connecting to {} ...".format(args.connect))
    master = mavutil.mavlink_connection(args.connect, baud=args.baud)
    master.wait_heartbeat()
    print("autopilot found: system {}".format(master.target_system))

    # Ask for ATTITUDE explicitly; without it the height filter runs as though
    # the aircraft were level, and the ground becomes an obstacle the moment it
    # tilts to accelerate.
    master.mav.request_data_stream_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 20, 1)

    rclpy.init()
    node = Bridge(args, master)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("\nstopped -- ArduPilot will now treat the obstacle data as stale "
              "and continue with NO avoidance")


if __name__ == "__main__":
    main()
