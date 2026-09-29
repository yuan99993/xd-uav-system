#!/bin/bash

PID_FILE=/tmp/UE_roslaunch.pids

if [ ! -f "$PID_FILE" ]; then
    echo "没有找到 UE_start.sh 启动的 roslaunch。"
    exit 0
fi

echo "正在关闭 UE_start.sh 启动的 roslaunch..."

while read -r PID; do

    [ -z "$PID" ] && continue

    if kill -0 "$PID" 2>/dev/null; then

        # 防止 PID 被系统重新分配后误杀其他程序
        CMD=$(ps -p "$PID" -o args=)

        if [[ "$CMD" == *"roslaunch"* ]]; then
            echo "关闭 PID=$PID"
            echo "  $CMD"

            # SIGINT 相当于 Ctrl+C，让 roslaunch 正常关闭其启动的节点
            kill -INT "$PID"
        else
            echo "跳过 PID=$PID：已经不是 roslaunch"
        fi

    fi

done < "$PID_FILE"

rm -f "$PID_FILE"

echo "UE roslaunch 已关闭。"