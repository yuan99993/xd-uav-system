# NanoTrack v2（局部跟踪候选）

来自 HonglinChu/SiamTrackers 的原始 ONNX 文件，来源提交及 SHA256 见 manifest.json，
许可证见 LICENSE。通过 OpenCV TrackerNano CPU 后端运行，不替换 D-FINE 检测器。

启动完整四模式候选测试：

```bash
roslaunch xd_uav_track dfine_v3_four_modes.launch feature_tracker_algorithm:=nano
```

UE 完整入口也接受 `feature_tracker_algorithm:=nano`；默认仍为配置中的 KCF。
`csrt`、`kcf` 可用同一个参数进行对比。指定算法不可用或初始化失败时明确返回失败，
不会把模板跟踪伪装成指定算法。深度单目标跟踪仍会漂移；必须继续使用 Fusion 的外观、
尺度、位置、检测身份和有效期门控。
