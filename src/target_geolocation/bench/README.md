# bench —— 性能取证与回归

这个目录是「先量再改」用的。定位解算本身很快（见 `cpu_profile.py`），真正吃 CPU
的是 Jetson 上的视频链路；下面这几个脚本就是用来把 CPU 精确归因到**线程**和
**函数**的，不靠猜。

## 脚本

| 文件 | 作用 | 是否改动被观测系统 |
|---|---|---|
| `cpu_profile.py` | 纯 Python 微基准：`solve_target` / `enu_to_geodetic` / 一次 tick / MAVLink 解析。不依赖 ROS，可直接跑 | 否 |
| `probe_jetson_cpu.py` | 一次取全：整机负载、目标进程的**线程级** top、X 服务器、进程环境、关键 fd | 只读 |
| `thread_cpu.py` | 读两次 `/proc/<pid>/task/*/stat` 求差，算出每线程真实 CPU%。`top -b -n 1` 的首次迭代口径不稳，这个更可靠 | 只读 |
| `sample_tid.py` | 按 **TID**（不是线程名）attach gdb 采样调用栈，拿函数级热点 | 短暂 stop-the-world |
| `verify_sync_fix.py` | A/B 验证 sink sync 改动，自动测前后并打印对比 | 会重启播放器 |
| `patch_ps_player.py` | 给 `ps_native_player.py` 打开关式补丁（默认行为不变） | 改文件，有备份 |
| `pin_env_to_launcher.py` | 把开关固化进 `start_ps_view.sh` | 改文件，有备份 |
| `fetch_remote.py` | 把远端脚本/配置拉回本地看 | 只读 |

## 两个容易踩的坑

**1. `top -b -n 1` 的 %CPU 不可全信。** 首次迭代没有上一帧做差分，口径可能是
累计平均。用 `thread_cpu.py` 的两次快照差分才对得上。

**2. 不能按线程名采样。** GStreamer 的多实例线程同名，`gdb` 的
`thread find rtpjitterbuffer` 会返回好几个编号，很容易采到正在睡眠的那个，得出
「它没在用 CPU」的错误结论。必须先用 `info threads` 里的 `(LWP <tid>)` 锁定
gdb 编号，`sample_tid.py` 就是这么做的。

## 2026-10-30 的结论（Jetson Orin NX，UE 像素流 1280×720@60fps）

进程 `ps_native_player.py` 长期占满一个核。线程级拆开是：

| 线程 | CPU | 说明 |
|---|---|---|
| `rtpjitterbuffer` | 70% | 忙等 |
| `nvv4l2decoder` | 10% | 真实解码（NVDEC 硬件） |
| 其余（nicesrc / queue / jitter） | 各 1~3% | 正常 |

判据：按 eth0 收包率 **3309 包/s**（平均包长 407 字节）算，70% 单核折合
**每包 241 微秒**，而这个元件正常只要 2~5 微秒，差 50 倍以上。包小、包多，
是这个元件在 WebRTC 场景下水土不服的直接原因。

### 踩过的三个坑（都值得记住）

**坑一：重启初期的数据不能当稳态。** 第一次 A/B 时重启后 3 秒测到 8.5%，
就宣布「改动有效」——其实那时码率才爬到 152 kbps、`shapeReqs=3`，进程根本
还没进入工作状态。稳定运行 6 分钟后再测，同一个配置回到 72%。**任何 A/B 都
必须等码率、fps 都稳定后再取样，且窗口不短于 60 秒。**

**坑二：`sync=True` 会引入成批丢帧。** 日志刷
`gst_base_sink_is_too_late ()`，`fps` 周期性掉到 0.9。原因不只是 CPU 节流——
basesink 默认 `max-lateness=20ms`，RTP 时间戳经 jitterbuffer 换算后常被判
「到得太晚」，于是整批丢弃。**只看 CPU 下降就下结论会漏掉画面质量的退化。**

**坑三：去掉外部 jitterbuffer 没用。** 以为 `webrtcbin` 与外部
`rtpjitterbuffer` 重复处理，去掉后 CPU 仍是 94.8%。因为 `webrtcbin` 内部的
rtpbin 按 SSRC/RTX 自建了多个同名线程（实测**同时存在 6 个**），外部那个只是
其中之一，真正的负担在里面，而 1.16 没暴露调它的属性。

### 四组配置实测

同一路 1280×720@60fps 流，每次测满 120~150 秒：

| 配置 | 平均单核 | 峰值 | jitterbuffer | fps | 丢帧 |
|---|---|---|---|---|---|
| `sync=False`（原始） | **94.9%** | 98.8% | ~70% | 60.0 | 无 |
| `sync=True` + 默认 20ms | 33.7% | 78.1% | 8.4% | 60 / **0.9** | **有** |
| **`sync=True` + `max-lateness=-1`** | **59.2%** | 90.9% | 27~46% | **60.0** | **无** |
| `sync=False` + 去掉外部 jb | 94.8% | 98.3% | ~70% | 60.0 | 无 |

采用第三组：**94.9% → 59.2%，省 38%，全程满帧不丢帧**。它没第二组那么低，是
因为「永不判迟到」要求 sink 必须消化每一帧，jitterbuffer 仍会少量排队——
但画面完整比那 25 个百分点重要。

开关（都已固化进 `start_ps_view.sh`）：`PS_SINK_SYNC`（默认 1）、
`PS_MAX_LATENESS`（默认 -1）、`PS_JB_LATENCY`、`PS_DEC_MAXPERF`、`PS_NO_JB`。

### 仍未解决

`rtpjitterbuffer` 27~46% 是外部那个去掉后、`webrtcbin` 内部实例的固定开销，
GStreamer 1.16 的 `webrtcbin` 没有暴露它的 `latency`。要再往下压，只能换接收
方式（例如让 UE 走 RTSP/RTMP，用可控 latency 的 `rtspsrc`），属于架构改动。

## 还剩的大户（本次未动）

- `chromium --type=gpu-process` 94%：`start_kiosk.sh` 用 `--use-angle=gl`，但
  `xorg3.conf` 里 `Disable "dri"`，:3 上大概率回落到软件光栅化，1080p 全屏合成
  纯 CPU。
- `selkies` 55%：`ximagesrc` 抓 1920×1080 全屏再 `nvh264enc` 编码；其中
  `pulsesrc`（音频采集）若用不到可以关掉。
- 架构层面：UE 流 → Jetson 解码 → 画到 X → 从 X 抓回 → 再编码 → 浏览器解码，
  同一条视频被解码两次、编码两次、跨进程搬三次。要根治得让远端浏览器直接连 UE
  的 WebRTC，Jetson 不参与转发。
