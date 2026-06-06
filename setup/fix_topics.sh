#!/bin/bash
# fix_topics.sh
# ─────────────────────────────────────────────────────────────────────────────
# Root cause: PX4 v1.16 publishes on versioned topic NAMES (vehicle_status_v4,
# vehicle_local_position_v1) but with standard message TYPES (VehicleStatus,
# VehicleLocalPosition).  The try/except import in the nodes tried to import
# VehicleStatusV4 / VehicleLocalPositionV1 types → ImportError → fell back to
# standard types BUT also reset the topic name to the unversioned form, which
# PX4 v1.16 doesn't publish → nothing received → VehicleStatus ABSENT forever.
#
# Fix: use standard message types unconditionally; subscribe to BOTH the
# versioned and unversioned topic names so the node works on any PX4 version.
#
# Run ONCE after bash stage14_swarm.sh, then bash launch_stage14.sh as normal.
# ─────────────────────────────────────────────────────────────────────────────

NODES=~/Desktop/FYP/ros2_ws/src/precision_pollination/precision_pollination
SURVEY=$NODES/field_survey.py
CTRL=$NODES/swarm_drone_controller.py

echo "============================================================"
echo "  fix_topics.sh — patch versioned PX4 topic subscriptions"
echo "============================================================"
echo ""

for F in "$SURVEY" "$CTRL"; do
  if [ ! -f "$F" ]; then
    echo "ERROR: $F not found. Run 'bash stage14_swarm.sh' first."
    exit 1
  fi
done

# ── Patch field_survey.py ──────────────────────────────────────────────────
python3 - << 'PYEOF'
import os, sys

path = os.path.expanduser(
    '~/Desktop/FYP/ros2_ws/src/precision_pollination/precision_pollination/field_survey.py')
with open(path) as f:
    src = f.read()

# ── Replace try/except import block ──
OLD_IMPORT = """\
try:
    from px4_msgs.msg import VehicleLocalPositionV1 as VLP
    POS_TOPIC = 'vehicle_local_position_v1'
except ImportError:
    from px4_msgs.msg import VehicleLocalPosition as VLP
    POS_TOPIC = 'vehicle_local_position'"""

NEW_IMPORT = """\
# PX4 v1.16 uses versioned topic NAMES with standard message TYPES.
# Subscribe to both; whichever PX4 publishes, the callback fires.
from px4_msgs.msg import VehicleLocalPosition as VLP
POS_TOPICS = ['vehicle_local_position_v1', 'vehicle_local_position']"""

if OLD_IMPORT not in src:
    print(f'WARNING: expected import block not found in field_survey.py '
          f'(already patched or different version). Skipping.')
    sys.exit(0)

src = src.replace(OLD_IMPORT, NEW_IMPORT)

# ── Replace single subscription with dual subscription ──
OLD_SUB = "self.create_subscription(VLP, f'{ns}/fmu/out/{POS_TOPIC}', self._pos, qos)"
NEW_SUB  = ("for _t in POS_TOPICS:\n"
            "            self.create_subscription(VLP, f'{ns}/fmu/out/{_t}', self._pos, qos)")

if OLD_SUB not in src:
    print('WARNING: expected subscription line not found in field_survey.py. Skipping.')
    sys.exit(0)

src = src.replace(OLD_SUB, NEW_SUB)

with open(path, 'w') as f:
    f.write(src)
print('field_survey.py  ✓ patched')
PYEOF

# ── Patch swarm_drone_controller.py ───────────────────────────────────────
python3 - << 'PYEOF'
import os, sys

path = os.path.expanduser(
    '~/Desktop/FYP/ros2_ws/src/precision_pollination/precision_pollination/swarm_drone_controller.py')
with open(path) as f:
    src = f.read()

# ── Replace try/except import block (both VLP and VS) ──
OLD_IMPORT = """\
try:
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

NEW_IMPORT = """\
# PX4 v1.16 uses versioned topic NAMES with standard message TYPES.
# Subscribe to both names; whichever PX4 publishes, the callback fires.
from px4_msgs.msg import VehicleLocalPosition as VLP
from px4_msgs.msg import VehicleStatus as VS
POS_TOPICS    = ['vehicle_local_position_v1', 'vehicle_local_position']
STATUS_TOPICS = ['vehicle_status_v4',         'vehicle_status']"""

if OLD_IMPORT not in src:
    print('WARNING: expected import block not found in swarm_drone_controller.py '
          '(already patched or different version). Skipping.')
    sys.exit(0)

src = src.replace(OLD_IMPORT, NEW_IMPORT)

# ── Replace single position subscription ──
OLD_POS = ("self.create_subscription(VLP, f'{ns}/fmu/out/{POS_TOPIC}', "
           "self._pos_cb, out_qos)")
NEW_POS  = ("for _t in POS_TOPICS:\n"
            "            self.create_subscription(VLP, f'{ns}/fmu/out/{_t}', "
            "self._pos_cb, out_qos)")

if OLD_POS not in src:
    print('WARNING: expected position subscription not found. Skipping.')
    sys.exit(0)
src = src.replace(OLD_POS, NEW_POS)

# ── Replace single status subscription ──
OLD_STA = ("self.create_subscription(VS, f'{ns}/fmu/out/{STATUS_TOPIC}', "
           "self._vs_cb, out_qos)")
NEW_STA  = ("for _t in STATUS_TOPICS:\n"
            "            self.create_subscription(VS, f'{ns}/fmu/out/{_t}', "
            "self._vs_cb, out_qos)")

if OLD_STA not in src:
    print('WARNING: expected status subscription not found. Skipping.')
    sys.exit(0)
src = src.replace(OLD_STA, NEW_STA)

with open(path, 'w') as f:
    f.write(src)
print('swarm_drone_controller.py  ✓ patched')
PYEOF

echo ""
echo "Both nodes patched. Each now subscribes to:"
echo "  Position : vehicle_local_position_v1  +  vehicle_local_position"
echo "  Status   : vehicle_status_v4          +  vehicle_status"
echo ""
echo "No rebuild needed (symlink install). Just relaunch:"
echo "  bash ~/Desktop/FYP/launch_stage14.sh"
echo ""
echo "Expected first signs of life in each controller terminal:"
echo "  [D0] Position lock      ← pos topic received"
echo "  [D0] PREFLIGHT -> ARMING"
echo "  [D0] ARMING -> SYNC_HOVER  ← arming_state == 2"
echo "============================================================"
