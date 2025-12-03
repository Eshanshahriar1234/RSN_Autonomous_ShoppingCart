#!/usr/bin/env bash
set -e

WORLD=/world/stopandshop_brigham_world

move_entity () {
  local name=$1
  local x=$2
  local y=$3
  local z=$4
  gz service -s ${WORLD}/set_pose \
    --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean \
    --timeout 200 \
    --req "name: \"${name}\", position: { x: ${x}, y: ${y}, z: ${z} }" > /dev/null
}

echo "Make sure Gazebo is running, simulation is unpaused,"
echo "and that models 'autocart' and 'moving_car' exist."
echo "Press Enter to start the smooth pose demo..."
read

echo "Resetting initial poses..."
# Cart starts left of parked row, above them
move_entity autocart   -18.0  8.0 0.3
# Moving car starts far to the left, below parked cars (lane in front)
move_entity moving_car -24.0  0.0 0.7
sleep 1.0

echo "Phase 1: cart rolling smoothly down towards the lane..."
# y: 8 -> 1 in steps of -0.1 (about 7 seconds)
for y in $(seq 8.0 -0.1 1.0); do
  move_entity autocart -18.0 ${y} 0.3
  sleep 0.1
done

echo "Cart stopped near lane, waiting for car..."
sleep 1.0

echo "Phase 2: moving car crosses in front, smooth horizontal motion..."
# x: -24 -> -12 at y = 0 (about 3 seconds)
for x in $(seq -24.0 0.4 -12.0); do
  move_entity moving_car ${x} 0.0 0.7
  sleep 0.1
done

echo "Car has passed. Short pause..."
sleep 1.0

echo "Phase 3: cart continues towards store front..."
# y: 1 -> -10 in steps of -0.1 (about 11 seconds)
for y in $(seq 1.0 -0.1 -10.0); do
  move_entity autocart -18.0 ${y} 0.3
  sleep 0.1
done

echo "Cart reached near the store front. Demo finished."
sleep 1.0

