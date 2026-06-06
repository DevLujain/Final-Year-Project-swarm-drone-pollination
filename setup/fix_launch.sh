#!/bin/bash
# ============================================================
# FIX launch — your launch_stage14.sh is stale (points to old
# deleted nodes: yolo_detection_sim / visual_servoing_controller
# / swarm_coordinator_2drone). This rewrites it to run the
# CURRENT swarm nodes: field_survey, swarm_drone_controller,
# swarm_monitor. Also (re)writes swarm_monitor.py + fix_camera.sh
# so all three nodes are guaranteed present.
#
# Does NOT touch field_survey.py or swarm_drone_controller.py
# (already fixed by fix_overwrite_nodes.sh).
#
# RUN:  bash fix_launch.sh
# THEN: bash ~/Desktop/FYP/launch_stage14.sh
# ============================================================

set -e
FYP_DIR=~/Desktop/FYP
ROS2_WS=$FYP_DIR/ros2_ws
NODES_DIR=$ROS2_WS/src/precision_pollination/precision_pollination

echo ""
echo "=============================================="
echo "  Fixing stale launch script"
echo "=============================================="
echo ""

# ── Verify the two main nodes exist (from fix_overwrite_nodes) ─
echo "[check] node files:"
for f in field_survey.py swarm_drone_controller.py; do
    if [ -f "$NODES_DIR/$f" ]; then
        echo "  OK   $f"
    else
        echo "  MISSING $f  <- run fix_overwrite_nodes.sh first!"
    fi
done

# ── (Re)create swarm_monitor.py (passive display) ─────────────
echo ""
echo "[write] swarm_monitor.py..."
cat > $NODES_DIR/swarm_monitor.py << 'PYEOF'
#!/usr/bin/env python3
"""swarm_monitor.py - PASSIVE display. Subscribes to /swarm/peer and
shows status. Makes NO decisions (coordination is peer-to-peer)."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json, time


class Monitor(Node):
    def __init__(self):
        super().__init__('swarm_monitor')
        self.d = {0: {}, 1: {}}
        self.t0 = time.time()
        self.create_subscription(String, '/swarm/peer', self._cb, 10)
        self.create_timer(2.0, self._dash)
        self.get_logger().info('SWARM MONITOR (passive display - no decisions)')

    def _cb(self, m):
        try:
            x = json.loads(m.data); self.d[x['drone_id']] = x
        except Exception: pass

    def _dash(self):
        t = round(time.time()-self.t0, 0)
        L = ['', f'===== SWARM  T+{t:.0f}s =====']
        done = 0
        for i in (0, 1):
            x = self.d.get(i, {})
            ph = x.get('phase', '-'); cl = x.get('claimed', [])
            pos = (f'({x.get("x")},{x.get("y")},{x.get("z")})'
                   if x.get('x') is not None else 'unknown')
            arm = 'ARMED' if x.get('armed') else 'disarmed'
            if ph == 'DONE': done += 1
            L += [f'  Drone {i} [{arm}] {ph}',
                  f'     pos {pos}  claims {cl}']
        L += [f'  completed: {done}/2', '==========================']
        for s in L: self.get_logger().info(s)


def main(args=None):
    rclpy.init(args=args)
    n = Monitor()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    finally: n.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
PYEOF
python3 -c "import ast; ast.parse(open('$NODES_DIR/swarm_monitor.py').read()); print('  swarm_monitor.py OK')"

# ── (Re)create fix_camera.sh ──────────────────────────────────
echo ""
echo "[write] fix_camera.sh..."
cat > $FYP_DIR/fix_camera.sh << 'CAMEOF'
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
CAMEOF
chmod +x $FYP_DIR/fix_camera.sh
echo "  fix_camera.sh OK"

# ── Rewrite launch_stage14.sh -> CURRENT swarm nodes ──────────
echo ""
echo "[write] launch_stage14.sh (current swarm nodes)..."
cat > $FYP_DIR/launch_stage14.sh << 'LAUNCHEOF'
#!/bin/bash
FYP_DIR=~/Desktop/FYP
ROS2_WS=$FYP_DIR/ros2_ws
NODES=$ROS2_WS/src/precision_pollination/precision_pollination

echo "STAGE 14 - Swarm Pollination (peer-to-peer, 6 flowers)"

# fresh random field each launch
python3 $FYP_DIR/generate_field.py

# hard-kill any stale processes so Gazebo reloads the new world
pkill -9 -f px4 2>/dev/null||true
pkill -9 -f 'gz sim' 2>/dev/null||true
pkill -9 -f gz-sim 2>/dev/null||true
pkill -9 -f gzserver 2>/dev/null||true
pkill -9 -f ruby 2>/dev/null||true
pkill -9 -f MicroXRCEAgent 2>/dev/null||true
sleep 6

# 1. XRCE Agent first
gnome-terminal --title="XRCE Bridge" -- bash -c "source ~/.bashrc; MicroXRCEAgent udp4 -p 8888; bash" &
sleep 6

# 2. PX4 drones
gnome-terminal --title="PX4 Drone 0" -- bash -c "
  source ~/.bashrc; cd ~/PX4-Autopilot
  PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE='2.0,0.0,0.1,0,0,0' \
  PX4_GZ_MODEL=x500 PX4_GZ_WORLD=demo_2flower \
  ./build/px4_sitl_default/bin/px4 -i 0; bash" &
sleep 14
gnome-terminal --title="PX4 Drone 1" -- bash -c "
  source ~/.bashrc; cd ~/PX4-Autopilot
  PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE='6.0,0.0,0.1,0,0,0' \
  PX4_GZ_MODEL=x500 PX4_GZ_WORLD=demo_2flower \
  ./build/px4_sitl_default/bin/px4 -i 1; bash" &
sleep 8

# 3. Current swarm nodes: survey + controller per drone
for D in 0 1; do
gnome-terminal --title="Drone $D survey" -- bash -c "
  source ~/.bashrc; source $ROS2_WS/install/setup.bash
  python3 $NODES/field_survey.py --ros-args -p drone_id:=$D; bash" &
sleep 1
gnome-terminal --title="Drone $D controller" -- bash -c "
  source ~/.bashrc; source $ROS2_WS/install/setup.bash
  python3 $NODES/swarm_drone_controller.py --ros-args -p drone_id:=$D; bash" &
sleep 1
done

gnome-terminal --title="Swarm Monitor" -- bash -c "
  source ~/.bashrc; source $ROS2_WS/install/setup.bash
  python3 $NODES/swarm_monitor.py; bash" &

# 4. Lock external camera after PX4 sets its follow
( sleep 18; bash $FYP_DIR/fix_camera.sh ) &

echo ""
echo "Launched. Tabs: XRCE / PX4 0 / PX4 1 / Drone 0 survey / Drone 0"
echo "controller / Drone 1 survey / Drone 1 controller / Swarm Monitor."
echo "Each controller should print 'subscribed ... topic' then 'Position lock'"
echo "then arm -> lift -> climb to 3m together -> survey -> negotiate ->"
echo "pollinate -> return -> land."
LAUNCHEOF
chmod +x $FYP_DIR/launch_stage14.sh
echo "  launch_stage14.sh OK"

echo ""
echo "=============================================="
echo "  DONE"
echo "=============================================="
echo ""
echo "Now run:"
echo "  bash ~/Desktop/FYP/launch_stage14.sh"
echo ""
echo "Tabs will now be: Drone 0/1 survey + Drone 0/1 controller +"
echo "Swarm Monitor (NOT the old YOLO/VS Controller tabs)."
echo ""
