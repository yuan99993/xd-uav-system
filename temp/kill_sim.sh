#!/bin/bash
# 关闭 MRS one_drone 仿真 session
# 等价于: tmux ctrl-a + :kill-session

export TMUX_SOCKET_NAME=mrs
export TMUX_SESSION_NAME=simulation

# 方法1: kill-session
tmux -L $TMUX_SOCKET_NAME kill-session -t $TMUX_SESSION_NAME 2>/dev/null && \
    echo "MRS simulation session killed."

# 方法2: 如果方法1失败，用原有 kill.sh
if tmux -L $TMUX_SOCKET_NAME has-session -t $TMUX_SESSION_NAME 2>/dev/null; then
    echo "kill-session failed, trying kill.sh..."
    bash "$HOME/catkin_ws/src/mrs_uav_gazebo_simulator/tmux/one_drone/kill.sh"
fi

# 确认进程已退出
sleep 2
for p in gzserver gzclient px4 roscore rosout; do
    if pgrep -x "$p" > /dev/null 2>&1; then
        echo "[WARN] $p still running, sending SIGKILL..."
        pkill -9 -x "$p" 2>/dev/null
    fi
done

sleep 1
echo "Done."
