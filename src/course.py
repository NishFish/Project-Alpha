#!/usr/bin/env python3
"""
The course: what shapes are in it, where the start and goal are, and the
geometry every other module needs from them.

One definition, read by three consumers that must never disagree:

  * sim/gazebo/make_world.py  -- renders them into Gazebo so you can see them
  * src/virtual_obstacle_publisher.py -- computes what a sensor would see
  * sim/run_course.py         -- flies to the goal and scores the clearances

Coordinates are ArduPilot NED metres relative to home: north and east, with
`yaw` measured clockwise from north like a compass. Gazebo is ENU and wants the
opposite sign; make_world.py does that conversion, deliberately in one place.

File format, one record per line:

    start  <north> <east>
    goal   <north> <east>
    cyl    <north> <east> <radius>
    box    <north> <east> <size_north> <size_east> <yaw_deg>

A wall is just a long thin box, and an L or a U is two or three of them. Blank
lines and # comments are ignored.
"""

import math


class Cyl(object):
    kind = "cyl"

    def __init__(self, north, east, radius):
        self.north, self.east, self.radius = north, east, radius

    def distance(self, n, e):
        """Distance to the surface. Negative means inside."""
        return math.hypot(self.north - n, self.east - e) - self.radius

    def ray(self, n0, e0, dn, de, max_r):
        """Distance along a unit ray to the first surface hit, or None."""
        fn, fe = n0 - self.north, e0 - self.east
        b = 2.0 * (fn * dn + fe * de)
        c = fn * fn + fe * fe - self.radius * self.radius
        disc = b * b - 4.0 * c
        if disc < 0.0:
            return None
        sq = math.sqrt(disc)
        for t in ((-b - sq) / 2.0, (-b + sq) / 2.0):
            if 0.0 <= t <= max_r:
                return t
        return None

    def describe(self):
        return "cyl  r=%.1f" % self.radius


class Box(object):
    kind = "box"

    def __init__(self, north, east, size_north, size_east, yaw_deg):
        self.north, self.east = north, east
        self.size_north, self.size_east = size_north, size_east
        self.yaw_deg = yaw_deg
        self._c = math.cos(math.radians(yaw_deg))
        self._s = math.sin(math.radians(yaw_deg))

    def _to_local(self, dn, de):
        """Rotate a world offset into the box's own axes."""
        return (dn * self._c + de * self._s,
                -dn * self._s + de * self._c)

    def distance(self, n, e):
        ln, le = self._to_local(n - self.north, e - self.east)
        hn, he = self.size_north / 2.0, self.size_east / 2.0
        ox = max(abs(ln) - hn, 0.0)
        oy = max(abs(le) - he, 0.0)
        if ox == 0.0 and oy == 0.0:                  # inside
            return max(abs(ln) - hn, abs(le) - he)
        return math.hypot(ox, oy)

    def ray(self, n0, e0, dn, de, max_r):
        # Slab test in the box's local frame.
        ln, le = self._to_local(n0 - self.north, e0 - self.east)
        rn, re = self._to_local(dn, de)
        hn, he = self.size_north / 2.0, self.size_east / 2.0

        tmin, tmax = 0.0, max_r
        for origin, direction, half in ((ln, rn, hn), (le, re, he)):
            if abs(direction) < 1e-9:
                if abs(origin) > half:
                    return None
                continue
            t1 = (-half - origin) / direction
            t2 = (half - origin) / direction
            if t1 > t2:
                t1, t2 = t2, t1
            tmin = max(tmin, t1)
            tmax = min(tmax, t2)
            if tmin > tmax:
                return None
        return tmin if 0.0 <= tmin <= max_r else None

    def describe(self):
        return "box  %.1f x %.1f @ %.0f deg" % (self.size_north, self.size_east,
                                                self.yaw_deg)


class Course(object):
    def __init__(self, shapes, start, goal):
        self.shapes = shapes
        self.start = start
        self.goal = goal

    def nearest_surface(self, n, e):
        return min((s.distance(n, e) for s in self.shapes), default=float("inf"))

    def ray_range(self, n0, e0, bearing_rad, max_r):
        """Nearest surface along a bearing, or None if the ray hits nothing."""
        dn, de = math.cos(bearing_rad), math.sin(bearing_rad)
        best = None
        for s in self.shapes:
            t = s.ray(n0, e0, dn, de, max_r)
            if t is not None and (best is None or t < best):
                best = t
        return best

    def describe(self):
        out = ["start  north %6.1f east %6.1f" % self.start,
               "goal   north %6.1f east %6.1f" % self.goal]
        for i, s in enumerate(self.shapes, 1):
            out.append("obs_%-2d north %6.1f east %6.1f  %s"
                       % (i, s.north, s.east, s.describe()))
        return "\n".join(out)


def load(path):
    shapes = []
    start = (0.0, 0.0)
    goal = None

    with open(path) as fh:
        for lineno, raw in enumerate(fh, 1):
            line = raw.split("#", 1)[0].strip()
            if not line:
                continue
            parts = line.split()
            kind = parts[0].lower()
            try:
                nums = [float(p) for p in parts[1:]]
            except ValueError:
                raise ValueError("%s:%d: non-numeric field in %r" % (path, lineno, line))

            if kind == "start" and len(nums) == 2:
                start = (nums[0], nums[1])
            elif kind == "goal" and len(nums) == 2:
                goal = (nums[0], nums[1])
            elif kind == "cyl" and len(nums) == 3:
                shapes.append(Cyl(*nums))
            elif kind == "box" and len(nums) == 5:
                shapes.append(Box(*nums))
            else:
                raise ValueError(
                    "%s:%d: unrecognised record %r -- expected start/goal n e, "
                    "cyl n e r, or box n e size_n size_e yaw" % (path, lineno, line))

    if goal is None:
        raise ValueError("%s: no 'goal' line; the mission has nowhere to go" % path)
    return Course(shapes, start, goal)


NUM_SECTORS = 72
SECTOR_WIDTH_DEG = 360.0 / NUM_SECTORS


def ring_from_course(course, n, e, yaw_rad, fov_deg, min_m, max_m,
                     rays_per_sector=3):
    """What a forward-facing range sensor at (n, e, yaw) would report, in cm.

    Ray-casts rather than computing angular extents, because extents only work
    for circles and the course now contains walls and corners. Several rays per
    sector so a thin wall between two sector centres is not missed -- which is
    also a real failure mode for a real sensor, just at a finer scale.

    Sector 0 is straight ahead and the index increases clockwise, matching
    MAV_FRAME_BODY_FRD.
    """
    min_cm, max_cm = int(min_m * 100), int(max_m * 100)
    empty = max_cm + 1
    ring = [empty] * NUM_SECTORS
    half_fov = fov_deg / 2.0

    step = SECTOR_WIDTH_DEG / float(rays_per_sector)
    offsets = [(k - (rays_per_sector - 1) / 2.0) * step
               for k in range(rays_per_sector)]

    sector = 0
    while sector < NUM_SECTORS:
        centre = sector * SECTOR_WIDTH_DEG
        if centre > 180.0:
            centre -= 360.0
        if abs(centre) > half_fov:
            sector += 1
            continue

        best = None
        for off in offsets:
            bearing = yaw_rad + math.radians(centre + off)
            hit = course.ray_range(n, e, bearing, max_m)
            if hit is not None and (best is None or hit < best):
                best = hit
        if best is not None and min_m <= best <= max_m:
            ring[sector] = int(best * 100)
        sector += 1

    return ring, min_cm, max_cm
