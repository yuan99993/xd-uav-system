# xd_uav_task_execute

`xd_uav_task_execute` 是每架工作机上的“到达后任务执行层”。它不负责任务分配、飞往目标、
起降或底层控制，只在工作机到达目标附近后接收一个长时 `ExecuteTask` action：

```text
task_allocate / 上级任务管理器
        -> /<uav>/task_execute/execute
        -> arrive 或 metric/visual track handler
        -> ExecuteTaskResult
```

当前已通过 `xd_uav_task_allocate/config/task_execution.yaml` 接入分配层。当前实现：

- `ARRIVE`：兼容“到达即完成”，收到 action 后立即成功；
- `TRACK`：可选切换 follower profile、可选选择本机 `local_track_id`，启动
  `xd_uav_track` 并监听 `TrackStatus`；
- `OBSERVE`：默认调用 `fw_metric_orbit`，以任务目标世界坐标进行定半径侦察盘旋；
- `INTERCEPT`：默认调用 `fw_metric_pursuit`，以任务目标世界坐标进行固定翼接近；
- 成功要求目标可见、非纯预测、目标 ID 未发生切换、请求的 follower profile 真正生效、
  控制参考已发布、估计器有效且状态为 `tracking`，连续保持配置时长；
- 指定 `local_track_id` 时会短暂重试选择，避免分配回调先于 track 回调而造成瞬时启动失败；
- 捕获超时、目标丢失、状态超时、总执行超时和 action 取消都有明确结果；
- TRACK 无论成功、失败还是取消都会先调用 `StartTracker(false)`，停止失败会覆盖原结果并报告
  `STOP_FAILED`，防止上层误以为控制权已释放。

`OBSERVE` 和 `INTERCEPT` 分别默认使用 `fw_metric_orbit` 和
`fw_metric_pursuit`；metric goal 的 `target_pose` 会在启动跟踪后发布到
`track/metric_target`，其坐标系必须是共享世界坐标系。

## 启动

先启动对应飞机的 detector、`xd_uav_track`、状态估计和控制器，再启动执行层：

```bash
source devel/setup.bash
UAV_NAME=uav3 roslaunch xd_uav_task_execute task_execute.launch
```

接口：

```text
/uav3/task_execute/execute  xd_uav_task_execute/ExecuteTaskAction
/uav3/task_execute/status   xd_uav_task_execute/TaskExecutionStatus
```

跟踪接口默认连接：

```text
/uav3/track/status
/uav3/track/start_tracker
/uav3/track/select_track
/uav3/track/set_profile
/uav3/track/metric_target   xd_uav_track/MetricTarget (shared world frame)
```

## 手工验证

到达目标附近后，可发送 TRACK goal。`local_track_id: -1` 表示沿用
`xd_uav_track` 的自动选择；多目标场景应由后续目标解析器给出明确的本机 track ID。

```bash
rostopic pub -1 /uav3/task_execute/execute/goal \
  xd_uav_task_execute/ExecuteTaskActionGoal \
  "{goal: {task_id: 1, target_id: 1, task_type: 1, worker_name: 'uav3',
  target_class_id: 0, local_track_id: -1, follower_profile: '',
  required_execution_sec: 10.0, maximum_duration_sec: 60.0}}"
```

推荐使用 action client，而不是在正式流程中直接发布内部 action goal 话题。

参数集中在 `config/task_execute.yaml`。Goal 中非零的 `required_execution_sec` 和
`maximum_duration_sec` 会覆盖 YAML 默认值。

## task_allocate 接入语义

规划器 `REACHED` 表示 `ARRIVED`，随后向被分配工作机的
`task_execute/execute` 发送 goal；只有 action 返回成功才调用 allocator 的 `complete()` 并释放
工作机。分配层配置可以保留 `mode: arrive` 的到达即完成模式，或使用
`mode: task_execute` 开启该链路；失败、取消与超时由分配层配置选择重新排队或等待操作员处理。
