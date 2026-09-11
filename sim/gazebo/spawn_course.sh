#!/usr/bin/env bash
#
# Spawn the obstacle course from course.txt into a RUNNING Gazebo world.
#
# WARNING: do not use this while the aircraft is airborne. A static 14 m
# pillar appearing around a hovering drone shoves it out of the sky -- that
# is exactly how the first attempt ended, with the iris knocked off a pillar
# and landed. Prefer make_world.py, which bakes the course into the world so
# the pillars exist before anything takes off. Use this only to add
# obstacles to a world where the aircraft is on the ground and well clear.
#
#   bash sim/gazebo/spawn_course.sh [world_name]
#
# Coordinate note, established by measurement rather than assumption: Gazebo's
# world frame here is ENU, so Gazebo x is EAST and Gazebo y is NORTH, while
# ArduPilot's LOCAL_POSITION_NED is x=north, y=east. The two are swapped. Get
# this wrong and the pillars appear at right angles to where the aircraft
# thinks they are.
set -uo pipefail

WORLD="${1:-iris_runway}"
HERE="$(cd "$(dirname "$0")" && pwd)"
MODEL="$HERE/pillar.sdf"
COURSE="$HERE/course.txt"

echo "removing any previous course ..."
for i in $(seq 1 30); do
  gz service -s "/world/$WORLD/remove" \
    --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 400 \
    --req "name: \"obs_$i\", type: MODEL" >/dev/null 2>&1
done

n=0
while read -r north east radius; do
  case "$north" in ''|\#*) continue ;; esac
  n=$((n + 1))
  z=7.0                       # half the pillar height, so it sits on the ground
  # Gazebo x = east, Gazebo y = north.
  if gz service -s "/world/$WORLD/create" \
      --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 4000 \
      --req "sdf_filename: \"$MODEL\", name: \"obs_$n\", allow_renaming: false, pose: {position: {x: $east, y: $north, z: $z}}" \
      >/dev/null 2>&1; then
    printf "  obs_%-2d  north %6s  east %6s  r %s\n" "$n" "$north" "$east" "$radius"
  else
    printf "  obs_%-2d  FAILED\n" "$n"
  fi
done < "$COURSE"

echo
echo "spawned $n pillars. Publisher arguments for the same course:"
printf "  --course %s\n" "$COURSE"
