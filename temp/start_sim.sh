#!/bin/bash
# SEAD 仿真环境启动脚本
# 封装 MRS one_drone tmux session，启动后等待 Gazebo + PX4 就绪

set -e

MRS_START_SCRIPT="$HOME/catkin_ws/src/mrs_uav_gazebo_simulator/tmux/one_drone/start.sh"
SIM_READY_TOPIC="/uav1/control_manager/offboard_switch"
TIMEOUT=120

echo "=========================================="
echo " SEAD 仿真环境启动"
echo "=========================================="
echo ""
echo "步骤 1: 启动 MRS one_drone tmux session..."
echo "  (这会打开 tmux 窗格: roscore → Gazebo → PX4 → takeoff → rviz)"
echo ""

# 检查是否已有 tmux session 在跑
if tmux -L mrs has-session -t simulation 2>/dev/null; then
    echo "[WARN] MRS simulation session 已存在"
    echo "  如需重启请先运行: ~/catkin_ws/src/mrs_uav_gazebo_simulator/tmux/one_drone/kill.sh"
    echo ""
fi

# 启动
bash "$MRS_START_SCRIPT" &
STARTER_PID=$!

echo ""
echo "步骤 2: 等待仿真就绪 (最长 ${TIMEOUT}s)..."
echo "  轮询话题: $SIM_READY_TOPIC"

# 确保 xd_uav_sead workspace 已 source
if [ -f "$HOME/catkin_ws/devel/setup.bash" ]; then
    source "$HOME/catkin_ws/devel/setup.bash"
fi
echo ""

# 等待关键话题出现
ELAPSED=0
READY=0
while [ $ELAPSED -lt $TIMEOUT ]; do
    if rostopic list 2>/dev/null | grep -q "uav1"; then
        echo "  [$ELAPSED s] uav1 话题已出现"
        READY=1
        break
    fi
    sleep 2
    ELAPSED=$((ELAPSED + 2))
    if [ $((ELAPSED % 10)) -eq 0 ]; then
        echo "  [$ELAPSED s] 等待中..."
    fi
done

if [ $READY -eq 1 ]; then
    echo ""
    echo "=========================================="
    echo " 仿真已就绪！"
    echo "=========================================="
    echo ""
    echo "可用的 uav1 话题:"
    rostopic list 2>/dev/null | grep uav1 | head -20
    echo ""
    echo "下一步:"
    echo "  1. 查看 mavros 等效话题:  rostopic list | grep uav1/mavros"
    echo "  2. 启动 SEAD onboard:"
    echo "     source devel/setup.bash"
    echo "     roslaunch xd_uav_sead sead_onboard.launch"
    echo "  3. mock GCS 测试:"
    echo "     rosrun xd_uav_sead mock_gcs.py _uav_name:=uav1 _cmd:=takeoff _alt:=120"
    echo ""
else
    echo "[ERROR] 超时 ${TIMEOUT}s，仿真未就绪"
fi

wait $STARTER_PID 2>/dev/null || true
