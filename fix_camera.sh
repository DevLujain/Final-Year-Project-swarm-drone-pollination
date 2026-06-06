#!/bin/bash
for i in 1 2 3 4 5; do
  gz service -s /gui/follow --reqtype gz.msgs.StringMsg \
    --reptype gz.msgs.Boolean --timeout 2000 --req 'data: ""' 2>/dev/null || true
  gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera \
    --reptype gz.msgs.Boolean --timeout 2000 \
    --req 'pose: {position: {x: 4, y: -7, z: 7}, orientation: {x: -0.1922, y: 0.1922, z: 0.6805, w: 0.6805}}' 2>/dev/null || true
  sleep 2
done
echo "Camera locked behind the field (4,-7,7). If it still tracks a"
echo "drone, drag once in the Gazebo window to detach."
