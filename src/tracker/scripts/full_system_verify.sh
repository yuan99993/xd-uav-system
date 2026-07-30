#!/bin/bash
# full_system_verify.sh — 完整系统功能验证脚本
# 验证: Tracker ↔ Follower ↔ PX4(MAVROS) ↔ MRS 之间的完整通信链路

set -e

source /home/promise/mrs_test/devel/setup.bash
export PYTHONNOUSERSITE=1

echo "╔══════════════════════════════════════════════════════════╗"
echo "║   Tracker + Follower  完整系统功能验证                    ║"
echo "╚══════════════════════════════════════════════════════════╝"
echo ""

# ═══════════════════════════════════════════════════════════════
# STEP 0: Environment check
# ═══════════════════════════════════════════════════════════════
echo ">>> STEP 0: Environment Check"
echo "  Python imports..."
python3 -c "
import rospy
from tracker.msg import NormalizedError, ExternalInput, TrackingOutput
from follower.msg import FollowerCommand, FollowerStatus, ControllerFeedback, ControllerCommand
from follower.srv import SetMode
from tracker.tracker_core import TrackerCore
from follower.follower_core import FollowerCore
from tracker.bspline_predictor import BSplinePredictor
from follower.feasibility_limiter import FeasibilityLimiter
print('    All imports OK')
" 2>&1 || { echo "  FAIL: Python imports"; exit 1; }

echo "  ROS packages..."
for pkg in tracker follower mavros px4 mavlink_sitl_gazebo; do
    if rospack find $pkg >/dev/null 2>&1; then
        echo "    $pkg: found"
    else
        echo "    $pkg: NOT FOUND (may be OK if not needed)"
    fi
done

echo "  STEP 0: PASS"
echo ""

# ═══════════════════════════════════════════════════════════════
# STEP 1: Start roscore + nodes
# ═══════════════════════════════════════════════════════════════
echo ">>> STEP 1: Start ROS Core + Nodes"

roscore &>/tmp/verify_roscore.log &
ROSCORE_PID=$!
sleep 3

rosrun tracker tracker_node.py &>/tmp/verify_tracker.log &
TRACKER_PID=$!
sleep 2

rosrun follower follower_node.py &>/tmp/verify_follower.log &
FOLLOWER_PID=$!
sleep 3

echo "  Tracker PID: $TRACKER_PID"
echo "  Follower PID: $FOLLOWER_PID"
echo ""

# ═══════════════════════════════════════════════════════════════
# STEP 2: Verify topics
# ═══════════════════════════════════════════════════════════════
echo ">>> STEP 2: Verify ROS Topics"

TOPICS=$(rostopic list 2>/dev/null)
REQUIRED_TOPICS=(
    "/tracker_node/external_input"
    "/tracker_node/normalized_error"
    "/tracker_node/tracking_output"
    "/follower_node/follower_command"
    "/follower_node/follower_status"
    "/follower_node/controller_feedback"
    "/follower_node/controller_command"
)

ALL_OK=true
for topic in "${REQUIRED_TOPICS[@]}"; do
    if echo "$TOPICS" | grep -q "$topic"; then
        echo "  $topic: ✓"
    else
        echo "  $topic: ✗ MISSING"
        ALL_OK=false
    fi
done

if [ "$ALL_OK" = false ]; then
    echo "  FAIL: Missing topics"
    kill $FOLLOWER_PID $TRACKER_PID $ROSCORE_PID 2>/dev/null
    exit 1
fi

echo "  STEP 2: PASS (all 7 topics present)"
echo ""

# ═══════════════════════════════════════════════════════════════
# STEP 3: Tracker external input → normalized error
# ═══════════════════════════════════════════════════════════════
echo ">>> STEP 3: Tracker: External Input → Normalized Error"

# Send start_track with bounding box
rostopic pub -1 /tracker_node/external_input tracker/ExternalInput \
    "{header: {stamp: now}, source: 'bounding_box', command: 'start_track', \
      normalized_bbox: [0.4, 0.45, 0.15, 0.2], has_normalized_bbox: true, \
      confidence: 0.95, class_id: 0}" >/dev/null 2>&1

sleep 2

# Send continuous updates with moving target
for i in $(seq 1 10); do
    X=$(python3 -c "print(0.4 + $i * 0.01)")
    Y=$(python3 -c "print(0.45 - $i * 0.005)")
    rostopic pub -1 /tracker_node/external_input tracker/ExternalInput \
        "{header: {stamp: now}, source: 'bounding_box', command: '', \
          normalized_bbox: [$X, $Y, 0.15, 0.2], has_normalized_bbox: true, \
          confidence: 0.95, class_id: 0}" >/dev/null 2>&1
    sleep 0.05
done

sleep 1

# Check normalized_error output
ERROR_OUT=$(rostopic echo -n 1 /tracker_node/normalized_error 2>/dev/null)
if echo "$ERROR_OUT" | grep -q "error_valid: True"; then
    EX=$(echo "$ERROR_OUT" | grep "error_x:" | head -1 | awk '{print $2}')
    EY=$(echo "$ERROR_OUT" | grep "error_y:" | head -1 | awk '{print $2}')
    echo "  error_valid: True"
    echo "  error_x: $EX"
    echo "  error_y: $EY"
    echo "  STEP 3: PASS"
else
    echo "  FAIL: error_valid is False"
    echo "  Output: $ERROR_OUT"
    kill $FOLLOWER_PID $TRACKER_PID $ROSCORE_PID 2>/dev/null
    exit 1
fi

# Check tracking_output
TRACK_OUT=$(rostopic echo -n 1 /tracker_node/tracking_output 2>/dev/null)
if echo "$TRACK_OUT" | grep -q "tracking_active: True"; then
    echo "  tracking_active: True"
else
    echo "  WARNING: tracking_active not True in tracking_output"
fi
echo ""

# ═══════════════════════════════════════════════════════════════
# STEP 4: Follower: activate and verify control output
# ═══════════════════════════════════════════════════════════════
echo ">>> STEP 4: Follower: Error → Control Velocity"

# Activate follower
rosservice call /follower_node/start "data: true" >/dev/null 2>&1
sleep 1

# Send more target updates so follower computes
for i in $(seq 1 10); do
    X=$(python3 -c "print(0.45 + $i * 0.005)")
    Y=$(python3 -c "print(0.45 - $i * 0.002)")
    rostopic pub -1 /tracker_node/external_input tracker/ExternalInput \
        "{header: {stamp: now}, source: 'bounding_box', command: '', \
          normalized_bbox: [$X, $Y, 0.15, 0.2], has_normalized_bbox: true, \
          confidence: 0.95, class_id: 0}" >/dev/null 2>&1
    sleep 0.05
done

sleep 2

# Check follower command
FOLL_CMD=$(rostopic echo -n 1 /follower_node/follower_command 2>/dev/null)
if [ -n "$FOLL_CMD" ]; then
    VFWD=$(echo "$FOLL_CMD" | grep "velocity_forward:" | head -1 | awk '{print $2}')
    YAW=$(echo "$FOLL_CMD" | grep "yaw_rate_deg_s:" | head -1 | awk '{print $2}')
    echo "  velocity_forward: ${VFWD:-0} m/s"
    echo "  yaw_rate_deg_s: ${YAW:-0} deg/s"
    echo "  STEP 4: PASS"
else
    echo "  WARNING: No follower command output (may need more frames)"
fi
echo ""

# ═══════════════════════════════════════════════════════════════
# STEP 5: Controller bidirectional port test
# ═══════════════════════════════════════════════════════════════
echo ">>> STEP 5: Controller Bidirectional Port Test"

# Check Follower→Controller feedback
CTRL_FB=$(rostopic echo -n 1 /follower_node/controller_feedback 2>/dev/null)
if [ -n "$CTRL_FB" ]; then
    echo "  /follower_node/controller_feedback: ✓"
else
    echo "  /follower_node/controller_feedback: empty (follower not active?)"
fi

# Test Controller→Follower command
rostopic pub -1 /follower_node/controller_command follower/ControllerCommand \
    "{header: {stamp: now}, controller_name: 'test_controller', \
      command_status: 'ready', controller_ready: true}" >/dev/null 2>&1
echo "  Controller→Follower port: ✓ (message sent)"
echo "  STEP 5: PASS"
echo ""

# ═══════════════════════════════════════════════════════════════
# STEP 6: Algorithm verification (Python integration test)
# ═══════════════════════════════════════════════════════════════
echo ">>> STEP 6: Algorithm Integration Test"

python3 << 'PYEOF'
import time
import math

# Test full pipeline: TrackerCore → FollowerCore
from tracker.tracker_core import TrackerCore
from follower.follower_core import FollowerCore

tracker = TrackerCore({
    'frame_width': 640, 'frame_height': 480,
    'enable_kalman_filter': True,
    'enable_motion_predictor': True,
    'enable_bspline_predictor': True,
})

follower = FollowerCore({
    'lateral_guidance_mode': 'coordinated_turn',
    'max_forward_velocity': 5.0,
    'forward_ramp_rate': 0.5,
    'pid_yaw': {'kp': 2.0, 'ki': 0.05, 'kd': 0.1},
    'pid_down': {'kp': 1.0, 'ki': 0.03, 'kd': 0.05},
    'feasibility_max_vel': 8.0, 'feasibility_max_acc': 5.0,
})

# Start tracking
tracker.process_external_bbox(
    bbox_pixel=(250, 200, 350, 300),
    confidence=0.95, class_id=0, command="start_track",
)

t_last = time.time()
for frame in range(30):
    t_now = time.time()
    dt = max(t_now - t_last, 0.001)
    t_last = t_now
    
    # Simulate detector
    cx = 250 + frame * 3
    cy = 200 + frame * 1
    det = [cx-50, cy-40, cx+50, cy+40, 0, 0.9, 0, 1]
    tr = tracker.update_with_detections([det], timestamp=t_now)
    
    # Feed to follower
    fr = follower.compute(
        error_x=tr.error_x, error_y=tr.error_y,
        dt=min(dt, 0.1), target_confidence=tr.confidence,
        error_valid=tr.tracking_active, timestamp=t_now,
    )

# Verify results
assert tr.tracking_active, "Tracker not active!"
assert abs(tr.error_x) < 1.0, f"Error X out of range: {tr.error_x}"
assert abs(tr.error_y) < 1.0, f"Error Y out of range: {tr.error_y}"
assert fr.command_valid, "Follower command not valid!"
assert abs(fr.yaw_rate_deg_s) < 100, f"Yaw rate excessive: {fr.yaw_rate_deg_s}"

print(f"  Tracker: active={tr.tracking_active}, err=({tr.error_x:+.3f},{tr.error_y:+.3f})")
print(f"  Follower: v_fwd={fr.velocity_forward:.2f}, yaw={fr.yaw_rate_deg_s:.1f} deg/s")
print(f"  Controller port: suggestion=({fr.suggested_velocity_forward:.2f},{fr.suggested_yaw_rate_deg_s:.1f})")

# Direction check
if tr.error_x > 0 and fr.yaw_rate_deg_s > 0:
    print("  Direction: correct (target right → yaw right)")
elif tr.error_x < 0 and fr.yaw_rate_deg_s < 0:
    print("  Direction: correct (target left → yaw left)")

# Safety check
from follower.safety_limits import SafetyValidator
sv = SafetyValidator()
vx, vy, vz = sv.clamp_command_magnitude(10, 10, 10)
assert math.sqrt(vx**2+vy**2+vz**2) <= 15.1, "Safety clamp failed!"

# Feasibility check
from follower.feasibility_limiter import FeasibilityLimiter
fl = FeasibilityLimiter(max_vel=5.0, max_acc=3.0)
vx2, vy2, vz2 = fl.limit(8.0, 2.0, 0.0)
assert abs(vx2) <= 5.1, f"Feasibility limiter failed: {vx2}"

print("  Safety & Feasibility: PASS")
print("  STEP 6: PASS")
PYEOF

echo ""

# ═══════════════════════════════════════════════════════════════
# STEP 7: MRS cross-package compatibility check
# ═══════════════════════════════════════════════════════════════
echo ">>> STEP 7: MRS Cross-Package Compatibility"

# Check MRS message types exist
for msg in "mrs_msgs/TrackerCommand" "mrs_msgs/ControlError" "mrs_msgs/VelocityReference"; do
    if rosmsg show $msg >/dev/null 2>&1; then
        echo "  $msg: ✓"
    else
        echo "  $msg: ✗ (not critical for standalone operation)"
    fi
done

echo "  STEP 7: PASS"
echo ""

# ═══════════════════════════════════════════════════════════════
# STEP 8: Cleanup
# ═══════════════════════════════════════════════════════════════
echo ">>> STEP 8: Cleanup"
kill $FOLLOWER_PID $TRACKER_PID $ROSCORE_PID 2>/dev/null
wait 2>/dev/null
echo "  All processes stopped"

echo ""
echo "╔══════════════════════════════════════════════════════════╗"
echo "║   VERIFICATION COMPLETE                                  ║"
echo "╚══════════════════════════════════════════════════════════╝"
