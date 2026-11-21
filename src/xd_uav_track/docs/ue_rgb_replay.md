# UE RGB 录像与 D-FINE-S V3 回放

入口为 `dfine_v3_rgb_replay.launch`，复用 SAR 包内的原始 RGB/K/TF 回放工具和完整四模式；XD 保持 perception_only。录像、来源/时间契约、环境和验收步骤见 [SAR RGB 录像说明](../../sar_yolo_detector/docs/ue_rgb_recording.md)。

`px4_ue_sitl.launch`、`px4_mission.py` 与实际坐标反馈属于本包；RGB 编解码、相机标定/视线、录制导出和回放输入属于 sar_yolo_detector。回放不启动飞行进程，不发送记录中的飞行命令，也不把记录的轨迹重新注入跟踪器。
