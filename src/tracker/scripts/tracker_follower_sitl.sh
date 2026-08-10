#!/bin/bash
# tracker_follower_sitl.sh — 一键启动完整仿真环境
#
# 启动: Gazebo + PX4 SITL + MAVROS + Tracker + Follower + RViz
#
# 用法:
#   bash tracker_follower_sitl.sh              # 默认空地世界
#   bash tracker_follower_sitl.sh empty        # 空地世界
#   bash tracker_follower_sitl.sh baylands     # 湾区世界
#   bash tracker_follower_sitl.sh warehouse    # 仓库世界

set -e

WORLD=${1:-empty}

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="/home/promise/mrs_test"

# ══════════════════════════════════════════════════════════════════════════
#  Environment Setup
# ══════════════════════════════════════════════════════════════════════════

source /opt/ros/noetic/setup.bash
source ${WS_DIR}/devel/setup.bash

# CRITICAL: Prevent Python from loading user-site pip packages
# (fixes numpy version conflict with ROS Noetic)
export PYTHONNOUSERSITE=1

# Gazebo model and plugin paths
# DISABLE online model downloads — prevents black screen hang
export GAZEBO_MODEL_DATABASE_URI=""
export GAZEBO_MODEL_PATH="/opt/ros/noetic/share/px4/Tools/sitl_gazebo/models:${GAZEBO_MODEL_PATH}"
export GAZEBO_PLUGIN_PATH="/opt/ros/noetic/lib:${GAZEBO_PLUGIN_PATH}"
export GAZEBO_RESOURCE_PATH="/opt/ros/noetic/share/px4/Tools/sitl_gazebo:${GAZEBO_RESOURCE_PATH}"

# PX4 SITL
export PX4_SIM_MODEL=iris
export PX4_ESTIMATOR=ekf2

# ══════════════════════════════════════════════════════════════════════════
#  One-time setup (only needed once)
# ══════════════════════════════════════════════════════════════════════════

# Ensure mavlink_sitl_gazebo can find models and worlds
if [ ! -d /opt/ros/noetic/share/mavlink_sitl_gazebo/models ]; then
    echo "[SETUP] Creating symlink: mavlink_sitl_gazebo/models"
    sudo ln -sf /opt/ros/noetic/share/px4/Tools/sitl_gazebo/models \
        /opt/ros/noetic/share/mavlink_sitl_gazebo/models
fi
if [ ! -d /opt/ros/noetic/share/mavlink_sitl_gazebo/worlds ]; then
    echo "[SETUP] Creating symlink: mavlink_sitl_gazebo/worlds"
    sudo ln -sf /opt/ros/noetic/share/px4/Tools/sitl_gazebo/worlds \
        /opt/ros/noetic/share/mavlink_sitl_gazebo/worlds
fi

# Ensure iris.sdf exists (PX4 apt package only ships .jinja template)
IRIS_SDF=/opt/ros/noetic/share/px4/Tools/sitl_gazebo/models/iris/iris.sdf
if [ ! -f "$IRIS_SDF" ]; then
    echo "[SETUP] Copying iris.sdf from PX4 source"
    sudo cp /home/promise/PX4-Autopilot/Tools/sitl_gazebo/models/iris/iris.sdf "$IRIS_SDF"
fi

# ══════════════════════════════════════════════════════════════════════════
#  Cleanup before launch
# ══════════════════════════════════════════════════════════════════════════

echo "[CLEAN] Stopping any leftover processes..."
pkill -f "px4" 2>/dev/null || true
pkill -f "gzserver" 2>/dev/null || true
pkill -f "gzclient" 2>/dev/null || true
pkill -f "gazebo" 2>/dev/null || true
pkill -f "mavros" 2>/dev/null || true
pkill -f "roscore" 2>/dev/null || true
pkill -f "rosmaster" 2>/dev/null || true

# Clean leftover PX4 socket files (critical: prevents "Address already in use" error)
echo "[CLEAN] Removing leftover PX4 socket files..."
sudo rm -f /tmp/px4-sock-* 2>/dev/null || true
rm -f /tmp/px4-sock-* 2>/dev/null || true

sleep 2

# ══════════════════════════════════════════════════════════════════════════
#  Launch
# ══════════════════════════════════════════════════════════════════════════

echo ""
echo "=============================================="
echo "  Tracker + Follower SITL Simulation"
echo "  World: ${WORLD}"
echo "  Mode:  PX4 SITL + Gazebo + MAVROS + RViz"
echo "=============================================="
echo ""

# Determine world file
case $WORLD in
    empty|default)
        WORLD_FILE="$(rospack find mavlink_sitl_gazebo)/worlds/empty.world" ;;
    baylands)
        WORLD_FILE="$(rospack find mavlink_sitl_gazebo)/worlds/baylands.world" ;;
    warehouse)
        WORLD_FILE="$(rospack find mavlink_sitl_gazebo)/worlds/warehouse.world" ;;
    yosemite)
        WORLD_FILE="$(rospack find mavlink_sitl_gazebo)/worlds/yosemite.world" ;;
    *)
        WORLD_FILE="$(rospack find mavlink_sitl_gazebo)/worlds/empty.world" ;;
esac

echo "World file: ${WORLD_FILE}"
echo ""

# Launch everything
roslaunch tracker tracker_follower_sim.launch \
    world:="${WORLD_FILE}" \
    gui:=true \
    paused:=false \
    rviz:=true \
    use_sim_time:=true
