# 阶段 4：QGC、UDP 与 ROS 集成审计及协议草案

更新日期：2026-08-05

本文记录阶段 4 首轮只读审计结果和设计基线。本轮没有启动或修改 QGroundControl，没有实现适配器，也没有进入飞行验证。V0–V8 的既有结论不在本轮重复验证；V9、DPGA/SimpleStrike 物理任务飞行、固定翼实飞和真实 XBee 仍是延期扩展项。

## 1. 现场与版本审计

### 1.1 运行现场

- `xd-uavsystem-test` 当前分支为 `feature/LTJ`。
- 仓库已有 3 个阶段 3 相关未提交修改：`SEAD_FULL_VALIDATION_RUNBOOK.md`、`sead_validation_visualizer.py`、`tmux/validation/commands.sh`。本轮未覆盖、暂存或回滚它们。
- 宿主机没有 tmux server；ROS master 不可达；进程列表未发现 QGC、ROS、Gazebo、PX4、MAVROS 或 SEAD 运行进程。

### 1.2 `src/Release` 产物

- Windows 产物入口是 `src/Release/QGroundControl.exe`，大小 44,352,000 bytes，修改时间为 2026-03-18 22:30:34 +08:00，SHA-256 为 `2df61d9c1536a9205c04d0a723c0e0937fe62e7b80528926ff893d3af3ca2f2a`。
- 同目录包含 Qt 6 运行库和 `lib/CustomXbee.lib`。后者大小 20,301,266 bytes，修改时间为 2026-03-18 22:23:20 +08:00，说明该产物确实编入过自定义 XBee/SEAD 模块，而不是纯上游 QGC。
- 二进制字符串中可识别 `SeadBackend`、`sendSeadMission`、`SEAD_MISSION` 和空域相关日志，进一步支持上述判断。
- 产物目录没有可可靠关联源码提交的 manifest、Git hash 或构建元数据。不能仅凭文件名或 QGC 显示版本证明它由当前源码构建。

### 1.3 `src/qgroundcontrol` 源码

- 当前分支为 `sead-dev`，HEAD 为 `6255348b95938a80434a8d2fe0ea13b7316c62d4`，`git describe` 为 `v5.0.3-622-g6255348b9`，工作区干净。
- 当前 HEAD 日期为 2026-05-16，晚于 Release 产物约两个月；关键提交 `488a558d3`（SEAD 修复）、`d1c6f78c8`（Formation）和 `84d22adac`（路由表发包逻辑）也晚于产物。结论：Release 最多对应 2026-03-18 左右的源码状态，不对应当前 `sead-dev` HEAD。
- 已有 UI 与任务模型：`FlyViewMap.qml` 可添加/插入/清除 SEAD 点，`XbeeWindow.qml` 可设置 origin、UDP/XBee 链路并触发 SEAD 下发；`SeadBackend` 保存任务点并生成 `msg_id=18` 和 `msg_id=19` 包。
- 已有 UDP：`MissionControl` 用同一个 `QUdpSocket` 绑定本地端口并向配置的 IP/同一端口发送裸业务包；UDP 和 XBee 共用 `PacketProtocol` 二进制 payload。它还接收遥测、文本和时间同步包。
- 当前裸 UDP 包没有应用层 magic/version、任务 ID、消息 ID、长度字段、通用校验、通用 ACK、重试、去重或事务超时。Airspace 有专用 ACK，但不能替代任务级可靠传输。
- 当前 QGC `SEAD_mission` 会按路由表逐 UAV 构造任务，UAV 角色由 `uav_id % 3` 推导；init 与 base 都取第一个任务点，heading 固定为 0。它不是完整、显式且可版本化的任务模型。

## 2. 当前 ROS 接口与语义

`xd_uav_sead` 没有自定义 `.msg/.srv/.action`；仿真桥使用 `std_msgs/String` JSON envelope，并在内部转换为原始二进制协议，以保持机载状态机不变。

| 方向 | ROS 接口 | 当前语义 |
|---|---|---|
| GCS → 单机 | `/<uav_name>/sead/command` | JSON：`msg_id`、`info`，可附 `ts`、`source`；bridge 按目标节点自己的 `uav_id` 生成原始包 |
| 单机 → GCS | `/<uav_name>/sead/telemetry` | JSON envelope，含 `ts/src/dst/broadcast/blob/len`；`blob` 是 base64 原始二进制包 |
| UAV ↔ UAV | `/sead/u2u` | 同类 JSON envelope；按 `src/dst/broadcast` 过滤，`blob` 是 base64 原始包 |
| 规划可视化 | `/<uav_name>/sead/planned_path` | DPGA 规划路径；不属于 UDP 控制协议 |

`mock_gcs.py` 是当前 ROS 命令语义的可执行样例：它向单机 command topic 发布 `msg_id + info`。已覆盖 Mode(1)、Arm(2)、Takeoff(3)、Waypoint(5)、频率(6)、Abort(8)、Origin(9)、SEAD mission(18)、Task Insert(19)、Airspace Clear/Zone(20/21)、Swarm(24)、Formation Point(26) 和 Info(44)。其中：

- Waypoint、Formation 和任务坐标是本地 ENU，单位 m；内部二进制缩放为 mm。
- SEAD mission 的 `targets`/`unknown_targets` 是 EN 平面点；`init_pos[2]` 和 `end[2]` 在原协议中实际是 heading（degree），不是高度。mock 参数名 `init_z/end_z` 容易误导，后续接口不能沿用这个歧义。
- `uav_type`、速度 m/s、最小转弯半径 m、航点半径 m 均由任务提供。
- 当前 bridge 忽略外层未知字段，因此可以接受适配器额外携带的追踪元数据，但不会把 `task_id/message_id` 传入 SEAD 状态机或回传为任务完成状态。
- 当前 telemetry 是原始二进制的 base64 包装，不是稳定的外部 JSON telemetry schema。适配器必须负责解析、筛选和版本化，QGC 不应直接依赖 ROS bridge 私有 envelope。

## 3. 最小 UDP 协议草案：SEAD-UDP/1

### 3.1 设计原则

- UDP 边界使用 UTF-8 JSON，便于 QGC、Python 适配器和测试器独立实现；ROS 内部仍映射到现有 command JSON，避免第一步重写 SEAD 算法。
- 一 datagram 一 envelope，禁止 IP/应用层分片；最大 datagram 建议 1200 bytes。较大目标集按 `chunk_index/chunk_count` 分块，适配器收齐后一次发布任务。
- 所有整数必须在 JSON 安全整数范围内；非有限浮点数、重复 JSON key 和未知必填枚举一律拒绝。
- UDP ACK 只表示适配器已经校验、去重并成功发布到 ROS，不表示 SEAD 已规划、飞行或完成任务。业务执行状态必须用独立 telemetry/event 表达。

### 3.2 Envelope

```json
{
  "magic": "SEAD",
  "version": 1,
  "type": "mission.submit",
  "message_id": "0194f4d0-7c1a-7e40-9a31-6bb76892ce01",
  "task_id": "0194f4d0-7c1a-7e40-9a31-6bb76892ce00",
  "seq": 1,
  "sent_at_ms": 1785897600000,
  "source": {"kind": "qgc", "id": "gcs-1"},
  "destination": {"uav_ids": [1, 2, 3]},
  "frame": {
    "name": "WGS84",
    "horizontal_unit": "degree",
    "vertical_datum": "WGS84_ELLIPSOID",
    "vertical_unit": "m"
  },
  "payload": {},
  "checksum": {"algorithm": "crc32c", "value": "7A1B2C3D"}
}
```

- `message_id`：每个逻辑发送动作的 UUIDv7；重试必须复用同一 ID。分块使用同一 `message_id` 与不同 `chunk_index`。
- `task_id`：一次 SEAD 任务的稳定 UUIDv7；修改任务必须使用新的 `message_id`，并以 `revision` 单调递增。它不等于 UAV ID 或原协议 `msg_id`。
- `seq`：同一 source 会话内单调递增的 uint64，用于诊断和检测明显回退；去重主键仍是 `(source.id, message_id, chunk_index)`。
- `sent_at_ms`：UTC Unix epoch 毫秒。适配器允许的默认时钟偏差为过去 10 s、未来 2 s；超界返回 `STALE`。本地测试可显式配置该窗口，不能静默关闭。
- `destination.uav_ids`：1..255 的非空去重列表；广播必须显式写 `"broadcast": true`，不能借用 UAV ID 0 表示多个含义。
- `checksum`：对移除 `checksum` 字段后按 RFC 8785/JCS 规范化的 UTF-8 bytes 计算 CRC-32C，输出 8 位大写十六进制。它用于检测实现/传输损坏，不提供身份认证；跨不可信网络时应另加 HMAC-SHA-256 或 DTLS，不能把 CRC 当安全机制。

### 3.3 坐标系与任务 payload

QGC 地图天然产生 WGS84 经纬度，因此外部协议以 WGS84 为权威输入；适配器根据 payload 中显式 origin 转为 SEAD 本地 ENU。这样 QGC 不复制机载 ENU 公式，ROS/SEAD 也不接收含糊的“x/y/z”。

```json
{
  "revision": 1,
  "origin": {
    "origin_id": "site-a-2026-08",
    "latitude_deg": 31.2304,
    "longitude_deg": 121.4737,
    "altitude_m": 12.4,
    "vertical_datum": "WGS84_ELLIPSOID"
  },
  "uavs": [
    {"uav_id": 1, "role": "COMBAT", "speed_mps": 20.0, "min_turn_radius_m": 35.0},
    {"uav_id": 2, "role": "SURVEILLANCE", "speed_mps": 20.0, "min_turn_radius_m": 35.0}
  ],
  "start": {"latitude_deg": 31.2305, "longitude_deg": 121.4738, "heading_deg": 0.0},
  "recovery": {"latitude_deg": 31.2304, "longitude_deg": 121.4737, "heading_deg": 180.0},
  "targets": [
    {"target_id": "t-001", "latitude_deg": 31.231, "longitude_deg": 121.474, "altitude_m": 12.4, "kind": "KNOWN"}
  ],
  "waypoint_acceptance_radius_m": 50.0
}
```

- 纬度/经度单位为 degree；heading 为真北顺时针 degree，规范化至 `[0,360)`；速度为 m/s；距离和高度为 m。
- 高度基准必须显式。v1 只接受 `WGS84_ELLIPSOID`；若 QGC 数据是 AMSL，必须先经经验证的大地水准面转换，不能改标签冒充椭球高。
- ENU 定义：x=East、y=North、z=Up，右手系，以 payload origin 为原点。SEAD mission 当前只消费目标 EN 与 start/recovery heading；高度不应伪装成 heading。
- `target_id` 在一个 task revision 内唯一且稳定。当前 SEAD 二进制任务包不携带 target ID；适配器 v1 维护有序映射并在日志/telemetry 中保留，若业务需要跨节点稳定引用，应在后续获得授权后扩展 `xd_uav_sead` 内部消息语义。
- `uavs[].role` 必须显式取 `SURVEILLANCE/COMBAT/MUNITION`，不得继续由 UAV ID 取模推导。适配器按 UAV 分别生成现有 ROS `msg_id=18` 的 `uav_type`、速度和 Rmin。

v1 首批消息类型限定为 `mission.submit`、`task.insert`、`mission.abort`、`airspace.replace`、`formation.configure`、`ack`、`telemetry.snapshot` 和 `event`。Arm、Takeoff、Land 等飞行安全指令暂不纳入 QGC UDP 最小任务接口；若未来加入，应单独设计权限、联锁和操作者确认。

### 3.4 ACK、重试、去重、超时与错误

ACK 使用同一 envelope，`type="ack"`，新建 ACK 自己的 `message_id`，并在 payload 中引用请求：

```json
{
  "request_message_id": "0194f4d0-7c1a-7e40-9a31-6bb76892ce01",
  "task_id": "0194f4d0-7c1a-7e40-9a31-6bb76892ce00",
  "status": "ACCEPTED",
  "code": "OK",
  "accepted_uav_ids": [1, 2, 3],
  "detail": "validated and published to ROS"
}
```

- QGC 初次发送后等待 500 ms；未收到 ACK 时按 500 ms、1 s、2 s、4 s 重试，共最多 5 次发送。重试内容及 `message_id` 不变。总事务超时 8 s 后显示失败，禁止无限后台重发。
- 适配器保存最近 10 分钟或 4096 个请求的去重缓存（先到者为准）。重复且内容 checksum 相同：不再次发布 ROS，重放原 ACK；同一 message ID 但 checksum 不同：返回 `CONFLICT`。
- 分块任务收集超时 3 s；缺块返回 `CHUNK_TIMEOUT`，不发布部分任务。完整任务的所有目的 UAV 发布都成功才返回 `ACCEPTED`；部分失败返回 `ROS_PUBLISH_PARTIAL` 并列出 UAV，不自动假装原子回滚。
- ROS master/目标 topic 不可用、schema/范围/坐标/origin/revision 校验失败时必须返回 `REJECTED`，并带稳定错误码。最小错误码：`BAD_MAGIC`、`UNSUPPORTED_VERSION`、`MALFORMED_JSON`、`SCHEMA_INVALID`、`CHECKSUM_MISMATCH`、`AUTH_FAILED`、`STALE`、`CONFLICT`、`CHUNK_TIMEOUT`、`UNKNOWN_UAV`、`ORIGIN_INVALID`、`COORDINATE_OUT_OF_RANGE`、`UNSUPPORTED_TYPE`、`ROS_UNAVAILABLE`、`ROS_PUBLISH_FAILED`、`ROS_PUBLISH_PARTIAL`、`INTERNAL_ERROR`。
- 错误响应不回显完整非法 datagram，避免日志放大或泄露；`detail` 限长 256 UTF-8 bytes。
- 业务事件至少区分 `TASK_ACCEPTED_BY_ADAPTER`、`TASK_SEEN_BY_SEAD`、`PLAN_READY`、`TASK_ABORTED`、`TASK_FAILED`。首个适配器版本只能可靠承诺第一项，其余需要 `xd_uav_sead` 提供 task correlation 后再启用。

## 4. 适配器位置与职责边界

建议未来新增仓库级独立目录：

```text
src/xd-uavsystem-test/tools/sead_udp_adapter/
```

它位于 `xd_uav_sead` ROS package 之外，不加入该包的 `CMakeLists.txt/package.xml`，但作为 Noetic Python 3 程序可显式 source `/opt/ros/noetic` 与工作区 overlay 后使用 `rospy`。目录应自带 README、schema、协议模型、发送器和纯单元测试；运行产物仍写 `.codex-tmp/`。

职责边界：

| 组件 | 应负责 | 不应负责 |
|---|---|---|
| QGC | 地图/UI、任务编辑、生成 task/message ID、SEAD-UDP/1 发送与 ACK 展示 | ROS 依赖、ENU 私有包格式、SEAD 分配算法、把 UDP ACK 显示成任务完成 |
| UDP→ROS 适配器 | UDP bind、schema/checksum/时效/去重/重试响应、WGS84→ENU、按 UAV fan-out、ROS command/telemetry 映射 | GA/DPGA/SimpleStrike 决策、飞控、安全状态机、修改任务内容 |
| `xd_uav_sead` | 任务语义、分配、路径/Formation/SimpleStrike、U2U、向 manager 提交期望 | 监听公网 UDP、QGC UI、底层 PX4 控制权 |
| manager/controller | OFFBOARD、控制状态、安全、姿态/推力输出 | GCS 网络协议和 SEAD 任务分配 |

适配器默认只绑定明确配置的地址，开发默认 `127.0.0.1`，建议端口 `15660/udp`；生产地址、允许的 QGC source 和认证材料必须从配置读取，禁止写死。QGC 本地接收 ACK 可使用同一 socket 的临时源端口，适配器回复 datagram 的实际 source address，不信任 payload 中回填的 IP。

## 5. 分阶段测试顺序

1. **协议纯测试**：canonical JSON、CRC-32C、schema、范围、错误码、UUID/revision、分块、超时、重复与冲突；无 ROS、无 QGC。
2. **适配器 ROS 假端测试**：本地发送器 → 适配器 → 测试 ROS subscriber；核对每个 UAV 的 command JSON、WGS84→ENU 和 ACK。仍不启动 SEAD/PX4。
3. **真实 SEAD 无飞行链**：发送器 → 适配器 → `xd_uav_sead`，只验证任务接收、分配日志与 telemetry/event 映射；不重复 V0–V8 的已有验证内容。
4. **QGC 后端测试**：在获授权后只改/测 SEAD-UDP/1 client 与任务模型，先用假适配器确认重试、错误展示和去重，不启动真实 QGC 前先完成 C++ 单元测试。
5. **QGC UI 人工测试**：获授权后启动与 Release 分离的新构建，观察地图字段、origin/高度基准、ACK 与错误提示。构建必须嵌入 Git hash，不能覆盖现有 Release。
6. **软件端到端**：QGC → 适配器 → ROS → SEAD 无飞行链。飞行验证是新的独立授权阶段，不由 UDP ACK 自动放行。

## 6. 后续最小修改文件清单（均待授权）

### 第一步：适配器，不改 QGC

- 新增 `tools/sead_udp_adapter/README.md`
- 新增 `tools/sead_udp_adapter/sead_udp_adapter.py`
- 新增 `tools/sead_udp_adapter/sead_udp_sender.py`
- 新增 `tools/sead_udp_adapter/protocol.py`
- 新增 `tools/sead_udp_adapter/schema/sead_udp_v1.schema.json`
- 新增 `tools/sead_udp_adapter/test/test_protocol.py`
- 新增 `tools/sead_udp_adapter/test/test_udp_ros_mapping.py`

若只要求“适配器已接收并发布”的 ACK，上述文件即可，不必修改 `xd_uav_sead`。若要求 `TASK_SEEN_BY_SEAD` 或任务执行状态与 `task_id` 关联，最小增量是修改 `xd_uav_sead/comms/rosbridge.py` 和相应测试，让追踪字段进入/退出 ROS bridge；不能由适配器猜测成功。

### 第二步：QGC（另行授权，且先确定源码/产物基线）

- 新增 `src/CustomXbee/SeadUdpProtocol.{h,cc}`：独立 envelope、JCS/CRC、ACK 状态机。
- 修改 `src/CustomXbee/SeadBackend.{h,cc}`：显式任务 ID/revision、UAV role、origin 与目标 ID，不再用 ID 取模推导角色。
- 修改 `src/CustomXbee/MissionControl.{h,cc}`：将 SEAD-UDP/1 与旧裸 UDP/XBee 路径分开；旧 XBee 保持兼容。
- 修改 `src/UI/CustomXbee/XbeeWindow.qml`：显示协议模式、任务 ID、ACK/错误，不把 connected/bound 等同于任务成功。
- 可能修改 `src/FlyView/FlyViewMap.qml`：只补目标 ID/任务模型所需字段，不重写地图。
- 新增对应 QGC 单元测试和构建 provenance 文件；新产物放独立版本目录，不覆盖 `src/Release/QGroundControl.exe`。

## 7. 当前阶段结论与门槛

当前 QGC 已有可复用的 SEAD UI、任务点模型和 UDP/XBee 代码，但当前 Release 与源码 HEAD 不对应，现有 UDP 是无版本裸业务包，不能直接作为稳定集成协议。正确顺序仍是：确认本草案 → 实现独立适配器和测试发送器 → 无飞行 ROS/SEAD 链 → 再申请修改和启动 QGC。

进入实现前仍需用户确认：SEAD-UDP/1 是否采用 JSON/JCS/CRC-32C；v1 是否只保证 adapter acceptance ACK；生产是否需要 HMAC；以及 adapter 的仓库级目录。未经确认不修改 QGC 或实现适配器。
