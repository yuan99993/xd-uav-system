#!/bin/bash
# SEAD Quad OFFBOARD 验证脚本
set -e

echo "=========================================="
echo " SEAD OFFBOARD 模式验证"
echo "=========================================="

source ~/catkin_ws/devel/setup.bash

# 1. 启动 SEAD onboard
echo ""
echo "[1/4] 启动 SEAD onboard..."
roslaunch xd_uav_sead sead_onboard.launch &>/tmp/sead_onboard.log &
SEAD_PID=$!
sleep 15

# 检查 frame_type
echo "[2/4] 验证 frame_type 分类..."
FT=$(grep "frame_type" ~/.ros/log/latest/uav1-sead_onboard-1.log 2>/dev/null | tail -1)
echo "  $FT"

# 2. 切 OFFBOARD
echo ""
echo "[3/4] 发送 OFFBOARD 命令..."
rosrun xd_uav_sead mock_gcs.py _uav_name:=uav1 _cmd:=mode _mode:=GUIDED 2>&1
sleep 5

# 3. 检查 PX4 模式
echo ""
echo "[4/4] 验证 PX4 模式..."
STATE=$(rostopic echo /uav1/mavros/state -n 1 2>/dev/null)
MODE=$(echo "$STATE" | grep "mode:" | awk '{print $2}' | tr -d '"')
ARMED=$(echo "$STATE" | grep "armed:" | awk '{print $2}')

echo "  armed=$ARMED"
echo "  mode=$MODE"

if [ "$MODE" = "OFFBOARD" ]; then
    echo ""
    echo "=========================================="
    echo " ✅ OFFBOARD 切换成功！"
    echo "=========================================="
else
    echo ""
    echo "=========================================="
    echo " ❌ 模式=$MODE（预期 OFFBOARD）"
    echo "=========================================="
fi

# 清理
kill $SEAD_PID 2>/dev/null
