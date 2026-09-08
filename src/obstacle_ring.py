#!/usr/bin/env python3
"""
Shared definition of the obstacle ring that ArduPilot consumes.

ArduPilot's proximity library wants exactly 72 distance readings arranged in a
circle around the vehicle. Everything in this project -- the fake publisher, the
depth pipeline, anything added later -- produces that same structure, so the
conventions live here in one place rather than being reinvented per file.

Conventions, all of which matter:

  * Index 0 points straight ahead, and the index increases clockwise. That is
    only true because we send MAV_FRAME_BODY_FRD with angle_offset = 0.
  * Distances are centimetres, as unsigned 16-bit integers.
  * A sector with no obstacle -- or one we simply cannot see -- is filled with
    max_distance + 1. Do NOT use 0 for this. Zero is a valid reading and means
    an obstacle is touching the drone.

That last point is worth dwelling on: there is no value meaning "I cannot see
here". A sector we have never observed looks identical to open space. The camera
covers roughly 72 of 360 degrees, so most of the ring is permanently unobserved.
Set WP_YAW_BEHAVIOR so the aircraft yaws into its direction of travel, and keep
the speed cap -- those are what compensate for it.
"""

from collections import deque

import numpy as np

NUM_SECTORS = 72
SECTOR_WIDTH_DEG = 360.0 / NUM_SECTORS


def empty_ring(max_cm):
    """A ring reporting 'nothing anywhere', in the encoding ArduPilot expects."""
    return [int(max_cm) + 1] * NUM_SECTORS


def bearing_to_sector(bearing_deg):
    """Map a bearing in degrees (0 = straight ahead, clockwise) to a sector index."""
    return int(round(bearing_deg / SECTOR_WIDTH_DEG)) % NUM_SECTORS


class TemporalFilter:
    """Median of the last N rings, per sector.

    Single-frame stereo noise is significant: an isolated bad pixel becomes an
    isolated close reading, and the aircraft brakes for a ghost. Taking the
    median across three frames rejects that while still reacting to anything
    that persists. It costs roughly 200 ms of latency, which is already in the
    stopping-distance budget.

    The 'empty' fill value participates in the median normally, which gives the
    behaviour you want for free: two empty frames out of three outvote a single
    spurious detection, and two detections out of three outvote a dropout.
    """

    def __init__(self, frames=3):
        if frames < 1:
            raise ValueError("frames must be at least 1")
        self.frames = frames
        self._history = deque(maxlen=frames)

    def update(self, ring):
        self._history.append(list(ring))
        stacked = np.array(self._history, dtype=np.int32)
        return np.median(stacked, axis=0).astype(int).tolist()

    def reset(self):
        self._history.clear()


def send_obstacle_distance(master, distances, min_cm, max_cm, time_usec=None):
    """Publish one OBSTACLE_DISTANCE message.

    pymavlink is imported here rather than at module scope so the depth maths
    can be developed and tested on a machine that has no MAVLink stack.
    """
    from pymavlink import mavutil

    if len(distances) != NUM_SECTORS:
        raise ValueError("expected {} distances, got {}".format(NUM_SECTORS, len(distances)))

    if time_usec is None:
        import time
        time_usec = int(time.monotonic() * 1e6)

    master.mav.obstacle_distance_send(
        time_usec,
        mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER,
        [int(d) for d in distances],
        0,                  # increment: 0 means "use increment_f"
        int(min_cm),
        int(max_cm),
        SECTOR_WIDTH_DEG,   # increment_f: degrees per sector
        0.0,                # angle_offset: 0 means index 0 is straight ahead
        mavutil.mavlink.MAV_FRAME_BODY_FRD,
    )


def render_ring(distances, max_cm, span_sectors=9, width=44):
    """A text picture of the sectors in front, for desk testing.

    Being able to see what the pipeline thinks it is looking at, without a
    ground station attached, is worth the thirty lines it costs.
    """
    lines = []
    for offset in range(span_sectors, -span_sectors - 1, -1):
        idx = offset % NUM_SECTORS
        cm = distances[idx]
        label = "{:+4.0f} deg".format(-offset * SECTOR_WIDTH_DEG)
        if cm > max_cm:
            lines.append("  {}  {:>7}  {}".format(label, "-", ""))
        else:
            filled = max(1, int(width * (1.0 - min(cm, max_cm) / float(max_cm))))
            lines.append("  {}  {:>6.2f}m  {}".format(label, cm / 100.0, "#" * filled))
    return "\n".join(lines)
