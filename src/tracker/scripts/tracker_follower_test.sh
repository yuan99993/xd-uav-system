#!/bin/bash
# tracker_follower_test.sh — 端到端 ROS 节点通信测试脚本

set -e

source /home/promise/mrs_test/devel/setup.bash

echo "=== Step 1: Start roscore ==="
roscore &
ROSCORE_PID=$!
sleep 3
echo "roscore PID: $ROSCORE_PID"

echo "=== Step 2: Start Tracker Node ==="
rosrun tracker tracker_node.py &
TRACKER_PID=$!
sleep 2

echo "=== Step 3: Start Follower Node ==="
rosrun follower follower_node.py &
FOLLOWER_PID=$!
sleep 3

echo "=== Step 4: List Topics ==="
rostopic list | grep -E "tracker|follower" || echo "  (no topics yet)"

echo "=== Step 5: Send External Input (start_track) ==="
rostopic pub -1 /tracker_node/external_input tracker/ExternalInput \
  "{header: {stamp: now}, source: 'bounding_box', command: 'start_track', \
    normalized_bbox: [0.45, 0.45, 0.1, 0.12], has_normalized_bbox: true, \
    confidence: 0.95, class_id: 0}"

sleep 1

echo "=== Step 6: Send Bounding Box Updates ==="
for i in 1 2 3 4 5; do
  rostopic pub -1 /tracker_node/external_input tracker/ExternalInput \
    "{header: {stamp: now}, source: 'bounding_box', command: '', \
      normalized_bbox: [0.45, 0.45, 0.1, 0.12], has_normalized_bbox: true, \
      confidence: 0.95, class_id: 0}" 2>/dev/null
  sleep 0.1
done

sleep 1

echo "=== Step 7: Check Tracker Output ==="
echo "--- /tracker_node/normalized_error ---"
rostopic echo -n 1 /tracker_node/normalized_error 2>/dev/null || echo "  (no data)"

echo "--- /tracker_node/tracking_output ---"
rostopic echo -n 1 /tracker_node/tracking_output 2>/dev/null || echo "  (no data)"

echo "=== Step 8: Check Follower Output ==="
echo "--- /follower_node/follower_command ---"
rostopic echo -n 1 /follower_node/follower_command 2>/dev/null || echo "  (no data)"

echo "--- /follower_node/follower_status ---"
rostopic echo -n 1 /follower_node/follower_status 2>/dev/null || echo "  (no data)"

echo ""
echo "=== Step 9: List All Topics ==="
rostopic list | grep -E "tracker|follower"

echo ""
echo "=== Cleanup ==="
kill $FOLLOWER_PID $TRACKER_PID $ROSCORE_PID 2>/dev/null
wait 2>/dev/null

echo "=== TEST COMPLETE ==="
