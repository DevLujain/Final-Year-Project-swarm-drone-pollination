#!/bin/bash
# ============================================================
# FIX topic discovery
#
# Root cause: PX4 v1.16 versions the TOPIC NAME
# (vehicle_status_v4, vehicle_local_position_v1) but keeps the
# message TYPE unversioned (VehicleStatus, VehicleLocalPosition).
# The controller wrongly looked for a TYPE 'VehicleStatusV4',
# failed, and subscribed to the wrong topic 'vehicle_status'
# (no suffix) -> VehicleStatus ABSENT.
#
# Fix: import the unversioned types and AUTO-DISCOVER the real
# topic names at runtime (matching any _vN suffix). No rebuild
# needed (symlink-install).
#
# RUN: bash fix_topic_discovery.sh
# THEN relaunch:
#   pkill -9 -f px4; pkill -9 -f 'gz sim'; pkill -9 -f gz-sim
#   pkill -9 -f ruby; pkill -9 -f MicroXRCEAgent; sleep 3
#   bash ~/Desktop/FYP/launch_stage14.sh
# ============================================================

set -e
NODES_DIR=~/Desktop/FYP/ros2_ws/src/precision_pollination/precision_pollination
CTRL=$NODES_DIR/swarm_drone_controller.py
SURV=$NODES_DIR/field_survey.py

echo ""
echo "=============================================="
echo "  Patching PX4 topic discovery"
echo "=============================================="
echo ""

python3 << 'PYEOF'
import os

ctrl = os.path.expanduser('~/Desktop/FYP/ros2_ws/src/precision_pollination/precision_pollination/swarm_drone_controller.py')
surv = os.path.expanduser('~/Desktop/FYP/ros2_ws/src/precision_pollination/precision_pollination/field_survey.py')

with open(ctrl) as f:
    s = f.read()

old_imp = """try:
    from px4_msgs.msg import VehicleLocalPositionV1 as VLP
    POS_TOPIC = 'vehicle_local_position_v1'
except ImportError:
    from px4_msgs.msg import VehicleLocalPosition as VLP
    POS_TOPIC = 'vehicle_local_position'
try:
    from px4_msgs.msg import VehicleStatusV4 as VS
    STATUS_TOPIC = 'vehicle_status_v4'
except ImportError:
    from px4_msgs.msg import VehicleStatus as VS
    STATUS_TOPIC = 'vehicle_status'"""
new_imp = """from px4_msgs.msg import VehicleLocalPosition as VLP
from px4_msgs.msg import VehicleStatus as VS"""
assert old_imp in s, "controller import block not found"
s = s.replace(old_imp, new_imp)

old_sub = """        self.create_subscription(VLP, f'{ns}/fmu/out/{POS_TOPIC}', self._pos_cb, out_qos)
        self.create_subscription(VS, f'{ns}/fmu/out/{STATUS_TOPIC}', self._vs_cb, out_qos)"""
new_sub = """        self._out_qos = out_qos
        self._pos_sub = None
        self._vs_sub = None
        self._resolver = self.create_timer(1.0, self._resolve_px4_topics)"""
assert old_sub in s, "controller subscription block not found"
s = s.replace(old_sub, new_sub)

resolver = '''    def _resolve_px4_topics(self):
        ns = f'/px4_{self.did}/fmu/out/'
        names = dict(self.get_topic_names_and_types())
        def find(base):
            if ns + base in names:
                return ns + base
            for t in names:
                if t.startswith(ns + base + '_v'):
                    return t
            return None
        if self._pos_sub is None:
            t = find('vehicle_local_position')
            if t:
                self._pos_sub = self.create_subscription(VLP, t, self._pos_cb, self._out_qos)
                self.get_logger().info(f'[D{self.did}] subscribed position topic: {t}')
        if self._vs_sub is None:
            t = find('vehicle_status')
            if t:
                self._vs_sub = self.create_subscription(VS, t, self._vs_cb, self._out_qos)
                self.get_logger().info(f'[D{self.did}] subscribed status topic: {t}')
        if self._pos_sub and self._vs_sub:
            self._resolver.cancel()
            self.get_logger().info(f'[D{self.did}] PX4 topic discovery complete')

'''
marker = "    def _pos_cb(self, m):"
assert marker in s, "controller _pos_cb not found"
s = s.replace(marker, resolver + marker, 1)

with open(ctrl, 'w') as f:
    f.write(s)
print("controller patched: unversioned types + topic auto-discovery")

with open(surv) as f:
    s2 = f.read()

old_imp2 = """try:
    from px4_msgs.msg import VehicleLocalPositionV1 as VLP
    POS_TOPIC = 'vehicle_local_position_v1'
except ImportError:
    from px4_msgs.msg import VehicleLocalPosition as VLP
    POS_TOPIC = 'vehicle_local_position'"""
new_imp2 = "from px4_msgs.msg import VehicleLocalPosition as VLP"
assert old_imp2 in s2, "survey import block not found"
s2 = s2.replace(old_imp2, new_imp2)

old_sub2 = "        self.create_subscription(VLP, f'{ns}/fmu/out/{POS_TOPIC}', self._pos, qos)"
new_sub2 = """        self._out_qos = qos
        self._pos_sub = None
        self._resolver = self.create_timer(1.0, self._resolve_px4_topics)"""
assert old_sub2 in s2, "survey subscription line not found"
s2 = s2.replace(old_sub2, new_sub2)

resolver2 = '''    def _resolve_px4_topics(self):
        ns = f'/px4_{self.did}/fmu/out/'
        names = dict(self.get_topic_names_and_types())
        def find(base):
            if ns + base in names:
                return ns + base
            for t in names:
                if t.startswith(ns + base + '_v'):
                    return t
            return None
        if self._pos_sub is None:
            t = find('vehicle_local_position')
            if t:
                self._pos_sub = self.create_subscription(VLP, t, self._pos, self._out_qos)
                self.get_logger().info(f'[D{self.did}] subscribed position topic: {t}')
                self._resolver.cancel()

'''
marker2 = "    def _pos(self, m):"
assert marker2 in s2, "survey _pos not found"
s2 = s2.replace(marker2, resolver2 + marker2, 1)

with open(surv, 'w') as f:
    f.write(s2)
print("survey patched: unversioned type + topic auto-discovery")
PYEOF

echo ""
echo "[verify] syntax check..."
python3 -c "import ast; ast.parse(open('$CTRL').read()); print('  controller OK')"
python3 -c "import ast; ast.parse(open('$SURV').read()); print('  survey OK')"

echo ""
echo "=============================================="
echo "  DONE - no rebuild needed (symlink-install)"
echo "=============================================="
echo ""
echo "Relaunch:"
echo "  pkill -9 -f px4; pkill -9 -f 'gz sim'; pkill -9 -f gz-sim"
echo "  pkill -9 -f ruby; pkill -9 -f MicroXRCEAgent; sleep 3"
echo "  bash ~/Desktop/FYP/launch_stage14.sh"
echo ""
echo "Within ~5s each controller should print:"
echo "  subscribed position topic: /px4_0/fmu/out/vehicle_local_position_v1"
echo "  subscribed status topic:   /px4_0/fmu/out/vehicle_status_v4"
echo "  Position lock"
echo "then arm -> lift -> climb together. No more 'VehicleStatus ABSENT'."
echo ""
