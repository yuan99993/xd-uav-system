#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
云台视觉伺服跟踪节点 (Image-Based Visual Servoing, IBVS)

功能
----
1. 订阅目标检测结果（目标在图像中的像素坐标）。
2. 控制云台，使目标始终保持在图像中心。

控制方式：速率控制 还是 绝对姿态控制？
--------------------------------------
本项目云台底层消息 GIMBAL_DEVICE_SET_ATTITUDE 同时支持：
  - 绝对姿态（四元数 q）
  - 角速率（angular_velocity_x/y/z）
core_sim 的 gimbal.cpp calcSetpoint() 对二者都做了处理（速率+角度=速率受限趋近角度，
纯速率=积分，纯角度=速率受限趋近）。

但是 mavros 提供的话题 /<uav_name>/mavros/mount_control/command (MountControl, mode=2)
只支持【绝对角度】，不支持速率（见 gimbal_control1.py 注释："MountControl在mode=2时
只支持绝对角度"）。要用速率控制只能走原始 mavlink 话题 /uav0/mavlink/to 自行打包
GIMBAL_DEVICE_SET_ATTITUDE，偏离了既有控制话题。

因此本节点采用【绝对姿态控制】，并用【图像视觉伺服】闭环：
  - 反馈量是“图像误差”（目标偏离图像中心多少像素），而不是云台绝对角度；
  - 每一帧按误差比例修正云台目标角度，闭环自然收敛到图像中心；
  - 不依赖云台角度反馈，因此没有开环积分漂移问题；
  - 仿真器内部已有速率限制 + slerp 平滑，绝对角度指令也能平滑运动，不会跳变。

话题
----
默认订阅: /<uav_name>/sar_yolo_detector/coco/tracked_detections
                                            (sar_yolo_detector/TrackedDetection2DArray)
也可切换为旧输入: /detect_multi/inference_box (vision_msgs/BoundingBox2D)
发布: /<uav_name>/mavros/mount_control/command (mavros_msgs/MountControl) 云台绝对角度

参数（可用 rosparam 或 launch 覆盖）
------------------------------------
~mode                        : 运行模式, tracking=自动跟踪(默认) / manual=手动测试
                                 / simulate=模拟运动目标(无需真实检测器, 测试连续跟踪)
~image_width / ~image_height : 图像分辨率（默认 1280x720）
~uav_name                   : UAV 命名空间（默认从 ROS_NAMESPACE 取，否则 uav0）
~detection_input             : tracked(默认，SmartTracker) / bbox2d(旧接口)
~detection_topic             : 检测输入话题；默认随 UAV 名称生成
~target_track_id             : 指定跟踪 ID，默认 -1（不筛选；兼容 track_id=-1）
~target_class_id             : 指定类别 ID，默认 -1（不筛选）
~minimum_detection_confidence: 最低置信度，默认 0
~fov_x_deg                   : 水平视场角（度，默认 45）
~kp_yaw / ~kp_pitch          : 比例增益，每帧消除误差的比例（0~1）。默认 yaw=0.09、
                                 pitch=0.06，带宽 kp×loop_hz=2.7/1.8°/s/°。延迟回路
                                 稳定判据=带宽×延迟<π/2(链路延迟实测0.4~0.5s)：全增益
                                 0.15/0.10(4.5/3°/s/°)时 4.5×0.5=2.25>1.57 → 实测±140px
                                 过冲振荡(测试9)。yaw 0.09(1.35<1.57)绕圈滞后
                                 5.7/2.7=2.1°≈57px(测试15 0.08 时 ±35~90px 晃动主体,
                                 0.09 待绕圈复验)。pitch 0.075(1.01<1.57)仍触发静态
                                 极限环(测试16: 静止目标 ±40px 来回摆周期 2.1s, 同段
                                 yaw 正常) → pitch 通道实际延迟>判据假设(UE 云台 pitch
                                 执行慢 + 检测 v 噪声起振 + 振荡频率处 ff 环路滞后≈92°
                                 变正反馈供能), 0.06 是验证过的稳定边界。注意: ff 只补
                                 瞬态(稳态画面静止时 ff=0), 稳态滞后=v_target/kp_eff
                                 全由 P 承担。检测噪声(0.07°)→指令抖动仅 ±0.2°/s。
~kd_yaw / ~kd_pitch          : 微分(阻尼)增益，单位秒，作用在滤波后角度误差变化率上
                                 （默认 0=关闭）。差分放大检测噪声(÷dt≈30倍)→画面抖动，
                                 故默认关；近距乱转靠提高 max_slew_yaw 解决。仅检测极干净
                                 且仍有延迟振荡时手动开启 0.03~0.05。
~ki_yaw / ~ki_pitch          : 条件积分增益（默认 2.0 / 1.5）。消除飞机匀速/转弯运动时
                                 纯比例无法消除的稳态偏差(目标画面角速率/kp_eff)。五重
                                 anti-windup：|err|>8° 不积分、接近限位冻结、clamp 封顶
                                 (补偿角速度最大 yaw=60°/s pitch=40°/s)、误差方向反转且
                                 |err|>0.3° 才清零(迟滞防检测微扰触发回摆；高速往返目标
                                 每 2-3s 过零一次，迟滞过大则反转后旧积分继续推挤 0.3-0.5s
                                 造成来回晃动，0.3°≈8px 即可 0.1s 内释放)、追击期(误差
                                 快速减小 d_ang<-3°/s)按帧衰减 0.9 防到位后残留推挤
                                 (测试14 静止目标: 残留 3.6°·s 推 7°/s → 过冲 42px 二次
                                 回中; 衰减后 <0.5°/s, 绕圈稳态不触发)。
~i_error_deg                 : 积分误差阈值（度，默认 8）。|err| 超阈值不积分。
                                 15→8°：初始大误差追击期(接管时 471px≈16.9°)误差
                                 降到 15° 以下即高速累积积分, 目标到位后残留积分
                                 以 8.8°/s 继续推 → 过零后冲过中心 81px 再回摆多次
                                 (实测测试13 静止目标)。8° 下追击期零累积、残留推挤
                                 减至 1/3; 稳态跟踪误差通常 <8°(绕圈峰值 6.5°)
                                 积分补偿行为不变, 无回归。
~step_alpha                  : 指令步长低通系数（0~1，默认 0.5，0=关闭）。对 P/I 合成
                                 的单帧步长做一阶平滑(时间常数≈33ms@30Hz)，抑制死区
                                 边缘反复触发与检测微扰引起的指令微摆。延迟仅 1 帧。
~ff_alpha / ~ff_dz_px        : 目标像素速度前馈：滤波后位置差分→速度低通(默认
                                 0.30, 时间常数≈0.11s)→速度死区(默认 15px/s≈0.5°/s)。
                                 前馈让云台直接以目标画面速度转动(死区内也执行)，
                                 消除反馈式控制的速度滞后与"停顿"。静止目标前馈=0。
                                 死区 15px/s 高于低通后白噪声(~12px/s)防误动, 低于
                                 原 20px/s 使 15~20px/s 慢速目标也有前馈参与(去停顿)。
                                 注意: ff 测的是【画面】速度(目标world-云台world)。
                                 [测试17 修复] 用 #285 反馈角速度(世界系, 差分+低通,
                                 ~fb_rate_alpha 默认0.15)把云台自转分量加回画面速度:
                                 ff 输入=画面速度+云台世界角速度=目标真实运动。静态
                                 目标(云台不动)ff→0 → 断开 0.45s 延迟下 ff 在振荡频点
                                 (周期≈2.1s)滞后≈92° 由阻尼变正反馈的极限环(测试16:
                                 ±40px@2.1s, 测试17 kp 回退后仍 ±13px); 绕圈稳态
                                 ff=目标速度(不再由 P 独扛稳态滞后)。反馈中断>1s 时
                                 补偿失效退化为旧行为。
~ff_gate_full_px / ~ff_gate_off_px : 前馈误差门控（默认 40/120px）。|err|≥off 时
                                 前馈权重=0，≤full 全开，之间线性过渡。根因: 大误差
                                 瞬态(接管/目标被甩)云台以反馈速率猛转, 画面速度≈云台
                                 自身转速(目标慢), 前馈把"云台转动"误当"目标运动"输出
                                 反方向角速度 → 与反馈对抗(测试15: 接管 242px 收敛
                                 2.1s + ff 低通残留推挤过冲 21px 再回调)。门控关闭后
                                 反馈全力收敛, 到位快且无 ff 推挤; 稳态跟踪误差
                                 (绕圈 ±35~50px)远小于 full, 正常跟踪不受影响。
~ff_max_yaw / ~ff_max_pitch  : 前馈角速度限幅（默认 45/35 deg/s，低于 slew 留反馈
                                 裕度）。前馈不追超过云台物理能力的速率。
~invert_yaw / ~invert_pitch  : 方向翻转（若云台转反了，翻转对应轴）
~deadzone_px                 : 像素死区，目标在中心附近时不调整（默认 4）。死区越小
                                 低速目标(<ff死区15px/s)越早触发 P 修正, 消除走走停停;
                                 检测噪声 ±2px(0.07°)远低于 4px 不会误触发。
~max_slew_yaw / ~max_slew_pitch : 指令最大变化率（deg/s，默认 yaw=60 pitch=45）。
                                 须高于目标最大方位变化率（飞机 40m/s 掠过 50m 近距目标
                                 ≈46°/s），否则云台追不上、比例项恒饱和→延迟极限环。
~use_prediction              : 是否启用运动预测补偿延迟（默认 False）。检测器噪声/
                                 振荡时预测器外推会放大误差(实测 proc 被推到 473/58)，
                                 加剧振荡；检测干净且目标快速运动时可开启补偿延迟。
                                 simulate 闭环模式无论如何强制关闭(模拟目标像素是
                                 yaw_cmd 从动函数、无独立惯性，外推会发散)。
~predict_horizon             : 预测前向时间（秒，默认 0.04）
~max_jump_px                 : 检测跳变门限（像素，默认 250）。单帧位移超过它视为
                                 误检(检测器跳到别的目标/虚警)丢弃，防野值踹偏云台。
                                 <=0 关闭门控。
~jump_reject_max             : 连续被跳变门控拒绝多少次后强制接纳重捕获（默认 5）
~filter_alpha                : 检测低通滤波系数（0~1，默认 0.3）。越小越平滑(抑制
                                 检测抖动)但滞后越大；检测干净/目标快速运动时上调 0.5。
~yaw_limit                   : yaw 限位（±度，默认 180）
~pitch_min / ~pitch_max      : pitch 限位（度，默认 -90 / 90）。pitch_cmd=-(相机世界
                                 俯仰)，+90=正下方、-90=正上方；过顶跟踪需下看到近正下方。
~target_timeout              : 连续无有效检测框多久后停止发送并释放云台控制权（秒，默认 0.5）
~loop_hz                     : 控制循环频率（Hz，默认 30）

模拟目标模式（~mode:=simulate）专用参数
---------------------------------------
~sim_pattern                 : 轨迹形状 circle(圆) / lissajous(李萨如) /
                                 sweep(水平扫) / figure8(8字) /
                                 rect(矩形周长+两条对角线, 分段线性, 检验各方向跟踪)，
                                 默认 circle
~sim_amplitude_deg           : 虚拟目标角幅度（度，默认 15）——云台会在 ±该角度
                                 范围内连续平滑跟踪。这是闭环模式下决定“动多大”
                                 的唯一参数；想动得更大就调大它（别超过 pitch 限位）。
~sim_period                  : 单圈周期（秒，默认 8.0，越大越慢越柔）
~sim_hz                      : 模拟目标生成频率（Hz，默认 30）
~sim_radius_px               : 【已弃用】旧开环实现的像素半径，保留仅为兼容，不再生效。

  说明（闭环）：旧版 simulate 把目标像素写成纯时间函数(开环)，云台怎么转都无法
  把它居中，IBVS 指令积分饱和、撞 pitch/yaw 限位、云台满速旋转 → 画面抖动。
  新版改为【闭环】：虚拟目标是一个缓慢运动的方向(角度)，按“目标方向 − 当前云台
  指向”投影成像素，云台越对准目标、目标越靠图像中心，误差收敛 → 平滑有界跟踪，
  与真实检测器闭环行为一致，用于在没有真实检测器时验证连续跟踪的平滑性与图像表现。

稳像后端（~stabilize:=true）
----------------------------
默认情况下本节点用 mavros MountControl(mode=2) 发送云台指令。但该消息
【没有稳像标志字段】，yaw 是机体相对，mavros 不会置 YAW_LOCK，云台处于
follow 模式：相机姿态 = 飞机实时姿态 × 云台机体角，会把飞机在 Hold 下的
微小振荡/振动直接传进画面 → 停止发送后看起来【抖动】。QGC 的 ROI 之所以
稳，是因为 PX4 发的 GIMBAL_DEVICE_SET_ATTITUDE 带 YAW_LOCK，相机直接用世界
系四元数固定在世界方向，拒绝飞机扰动（见 robot.cpp UpdateCameraPose）。

开启 ~stabilize:=true 后，本节点通过 /<uav_name>/mavlink/to 发送
GIMBAL_MANAGER_SET_ATTITUDE(#282) 给 PX4 gimbal_manager，显式带
ROLL_LOCK|PITCH_LOCK|YAW_LOCK，并把机体相对 yaw 用飞机罗盘航向转到世界系，
构造世界系四元数。manager 收到 #282 后统一产生唯一的
GIMBAL_DEVICE_SET_ATTITUDE(#284) 到设备，相机用世界系四元数固定方向、拒绝
飞机扰动，跟踪全程及停止后都保持稳像。

为什么经 manager 而非直发裸 #284：本机配置 MNT_MODE_IN=4 启用了 PX4
gimbal_manager，它是云台设备的唯一权威命令源。若本节点直发裸 #284，会与
manager 自身的输出（以及 QGC 的 ROI）两路同时打到设备、交替覆盖目标姿态 →
画面抖动；且设备只在 manager 发起的 GIMBAL_DEVICE_INFORMATION 握手后才激活，
直发 #284 在首次启动时到不了设备（表现为“稳像一开始不生效，QGC 点 ROI 后才
动”）。改发 #282 后：命令源唯一（消除抖动），且 #282 会唤起 manager 完成握手
（首次启动无需 QGC 点 ROI）。需要飞机航向，故订阅
/<uav_name>/mavros/global_position/compass_hdg。

抢占主控权（primary control）：PX4 gimbal v2 输入只接受 origin 等于已配置
primary control (sysid/compid) 的 #282，否则静默忽略（input_mavlink.cpp
_process_set_attitude）。primary 初值是 0/0（无人），所以光发 #282 云台纹丝
不动。收到有效检测框时才发 MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE(命令值 1001,
param1=src_sysid, param2=src_compid) 把 primary 抢到本节点身份，之后的 #282
才被采纳。连续 ~target_timeout 秒没有有效框时停止发送姿态并发 command 1001
释放本节点的 primary；下一次收到有效框再重新接管。QGC 能控制正是因为它也先
发这个命令/ROI 抢了 primary。

航向前馈（yaw 稳像）：世界 yaw = 航向 + yaw_cmd，若 yaw_cmd 不变，飞机一转向
世界 yaw 就跟着变，云台会先随飞机转、再被 IBVS 慢慢拉回（表现为“航向变化超过
一定角度后云台跟着旋转后再指回原方向”）。为此控制环每帧把航向增量从 yaw_cmd
里减掉，使 (航向 + yaw_cmd) 在目标居中时恒定，世界 yaw 只随 IBVS 修正演化、
与飞机航向解耦，实现真正的 yaw 稳像。

~stabilize                   : 是否启用稳像后端（默认 False，用 MountControl）
~gimbal_sysid                : #282 的 target_system（默认 1=自动驾驶仪，运行 gimbal_manager）
~manager_compid              : #282 的 target_component（默认 0=MAV_COMP_ID_ALL，manager 在此监听）
~gimbal_device_id            : #282 指定的云台设备 id（默认 154=PX4 gimbal 设备，
                                 0 仅在设备 ID 未知时作为兜底）
~gimbal_compid               : 【#282 路径不用】旧直发 #284 的设备 compid(默认154)，保留仅为兼容
~src_sysid / ~src_compid     : 本节点源身份（默认 255 / 190，仿 QGC）
~stabilize_pitch_negate      : 构造世界四元数时是否对 pitch 取反（默认 True，
                                 由 test4.py 实测约定得出；若俯仰方向反了改为 False）
~yaw_offset_deg              : 世界 yaw 偏置修正（度，默认 0；若指向有固定偏差用它修）
~heading_ff_max_step         : 航向前馈单帧变化上限（度，默认 12；超过视为罗盘野值
                                 跳过本帧前馈。30Hz 下 12°/帧≈360°/s，远超真实转向率）
~use_tmux_claim              : 是否允许 tmux 抢占作为兜底（默认 False）。抢占主控优先
                                 走 mavros command 服务(COMMAND_LONG + command=1001 + ACK, 真机首选)；失败时才
                                 用 tmux。真机够不到 PX4 shell, tmux 会自动失败跳过, 故真机无需关。
~px4_tmux_target             : PX4 控制台所在 tmux 目标（默认 "simulation:2.0"）。
                                 仅 use_tmux_claim=True 且 mavros command 1001 失败时, 经 tmux
                                 send-keys 发 gimbal primary-control 抢占主控(SITL兜底)。
                                 设为空字符串则跳过 tmux。
~px4_tmux_socket             : tmux socket 名（默认 "sim"，即 tmux -L sim）

地理兴趣点模式（适合已定位的地面目标）
------------------------------------
~geo_roi_enabled             : 默认 False。开启时须同时开启 ~stabilize:=true。
~geo_roi_topic               : TaskCandidateArray；候选必须与当前检测的 source_track_id
                                 相同、状态 CONFIRMED/UPDATED 且 localization_valid。
~vehicle_pose_topic          : 飞机 PoseStamped，默认 mavros/local_position/pose。
                                 两个 Pose 的 frame_id 必须完全相同，且使用同一个 ENU
                                 地图原点；仅名字相同但原点不同会指错位置。
~roi_hold_timeout            : 检测丢失后保留已锁定坐标的时间，默认 30 秒。
~roi_pose_timeout            : 飞机位姿最大接收间隔，默认 0.5 秒。
~roi_correction_gain         : 几何视线残差修正速率，默认 1/s。

本模式每帧根据飞机与目标的三维位置计算世界方位和俯仰，位置变化作为前馈，
识别框只修正小的指向误差；丢帧期间保持地图点指向。单靠二维检测框不能推断
目标距离，bearing_only 候选不启用地理模式。移动目标需要持续更新三维定位；
超过云台最大角速度、视场或机械限位时不能保证持续可见。
"""

import math
import time
import struct
import subprocess
import threading

import rospy
from vision_msgs.msg import BoundingBox2D
from mavros_msgs.msg import MountControl, Mavlink
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped

try:
    from sar_yolo_detector.msg import TrackedDetection2DArray
    from sar_yolo_detector.msg import TaskCandidateArray
except ImportError:  # Keep the legacy BoundingBox2D input usable without this package.
    TrackedDetection2DArray = None
    TaskCandidateArray = None

# mavros command 服务（发 COMMAND_LONG 并等 COMMAND_ACK）。用于以 mavros 原生方式
# 发 DO_GIMBAL_MANAGER_CONFIGURE(命令值 1001) 抢占云台主控权——这是真机上的标准做法
# (真机无 PX4 shell, tmux 不可用)。try/except 包裹：缺失时回退到 raw command 1001/tmux。
try:
    from mavros_msgs.srv import CommandLong
    _HAVE_CMD_SRV = True
except Exception:  # pragma: no cover
    CommandLong = None
    _HAVE_CMD_SRV = False

# pymavlink 仅在稳像后端(~stabilize:=true)需要，用于编码 GIMBAL_DEVICE_SET_ATTITUDE。
# 注意：必须用 v20 common 方言；mavutil.mavlink 默认是 v10 ardupilotmega，没有该消息。
# 用 try/except 包裹：未安装时默认 MountControl 模式仍可正常运行。
try:
    from pymavlink.dialects.v20 import common as mavlink2
    _HAVE_PYMAVLINK = True
except Exception:  # pragma: no cover
    mavlink2 = None
    _HAVE_PYMAVLINK = False


# GIMBAL_DEVICE_FLAGS（与 mavlinkcom 一致）
GIMBAL_DEVICE_FLAGS_ROLL_LOCK = 4
GIMBAL_DEVICE_FLAGS_PITCH_LOCK = 8
GIMBAL_DEVICE_FLAGS_YAW_LOCK = 16

# GIMBAL_MANAGER_FLAGS（GIMBAL_MANAGER_SET_ATTITUDE #282 使用）。
# ROLL_LOCK/PITCH_LOCK/YAW_LOCK 的位值与设备 flags 完全相同(4/8/16)，但属于
# manager 层语义：YAW_LOCK 表示四元数 yaw 在世界系。稳像后端经 PX4 gimbal_manager
# 下发 #282，由 manager 统一产生唯一的 GIMBAL_DEVICE_SET_ATTITUDE(#284) 到设备，
# 避免与 QGC ROI / manager 自身输出两路 #284 抢占设备导致抖动。
GIMBAL_MANAGER_FLAGS_RETRACT = 1
GIMBAL_MANAGER_FLAGS_NEUTRAL = 2
GIMBAL_MANAGER_FLAGS_ROLL_LOCK = 4
GIMBAL_MANAGER_FLAGS_PITCH_LOCK = 8
GIMBAL_MANAGER_FLAGS_YAW_LOCK = 16

# MAVLink gimbal 消息与命令常量。
# 这些值直接对应 PX4 gimbal_manager / gimbal_device 的 MAVLink 规范。
MSG_GIMBAL_MANAGER_SET_ATTITUDE = 282
MSG_GIMBAL_DEVICE_SET_ATTITUDE = 284
MSG_GIMBAL_DEVICE_ATTITUDE_STATUS = 285
# 这里的 1001 是 PX4 的命令枚举值，不是 MAVLink 消息 ID。
# 实际发送时通过 MAVLink COMMAND_LONG(#76) 携带这个 command=1001。
MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE = 1001


def _build_gimbal_claim_params(src_sysid, src_compid):
    """构造 command=1001 抢占主控所需的命令参数，避免把保留字段置为 NaN。"""
    return [float(src_sysid), float(src_compid), 0.0, 0.0, 0.0, 0.0, 0.0]


def _build_gimbal_release_params():
    """构造 command=1001 释放当前 primary、保持 secondary 不变的参数。"""
    return [-3.0, -3.0, -1.0, -1.0, 0.0, 0.0, 0.0]


def euler_to_quat_zyx(roll_rad, pitch_rad, yaw_rad):
    """ZYX Tait-Bryan 欧拉角 -> 四元数 [w, x, y, z]。

    与项目 core_sim/src/transforms/transform_utils.cpp 的 ToQuaternion 完全一致
    （roll 绕 x、pitch 绕 y、yaw 绕 z，右手系），保证构造的世界系四元数被仿真器
    按相同约定解释。
    """
    t0 = math.cos(yaw_rad * 0.5)
    t1 = math.sin(yaw_rad * 0.5)
    t2 = math.cos(roll_rad * 0.5)
    t3 = math.sin(roll_rad * 0.5)
    t4 = math.cos(pitch_rad * 0.5)
    t5 = math.sin(pitch_rad * 0.5)

    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5
    return [w, x, y, z]


# ================= 一阶低通滤波 =================
class LowPassFilter:
    """抑制检测框抖动。alpha 越大越跟随原始值，越小越平滑。"""

    def __init__(self, alpha):
        self.alpha = alpha
        self.value = None

    def update(self, x):
        if self.value is None:
            self.value = x
        else:
            self.value = self.alpha * x + (1.0 - self.alpha) * self.value
        return self.value

    def reset(self):
        self.value = None


# ================= 匀速运动预测 =================
class TargetPredictor:
    """估计目标像素速度并前向预测，补偿“检测+控制+云台响应”链路延迟，
    减小运动目标跟踪滞后。速度估计做了平滑，避免单帧跳变。"""

    def __init__(self):
        self.prev = None
        self.vel = [0.0, 0.0]
        self.t = None

    def update(self, u, v, t):
        if self.t is not None:
            dt = t - self.t
            if dt > 1e-6:
                inst_vu = (u - self.prev[0]) / dt
                inst_vv = (v - self.prev[1]) / dt
                self.vel[0] = 0.5 * inst_vu + 0.5 * self.vel[0]
                self.vel[1] = 0.5 * inst_vv + 0.5 * self.vel[1]
        self.prev = (u, v)
        self.t = t

    def predict(self, t_now, horizon):
        if self.prev is None:
            return None
        dt = (t_now - self.t) + horizon
        return (self.prev[0] + self.vel[0] * dt,
                self.prev[1] + self.vel[1] * dt)

    def reset(self):
        self.prev = None
        self.vel = [0.0, 0.0]
        self.t = None


# ================= 主节点 =================
class GimbalVisualServo:

    def __init__(self):
        # Multiple UAVs can run this controller under one ROS master.  The
        # vehicle is selected with ~uav_name, which namespaces its topics but
        # does not namespace this ROS node name.  Use a unique registration
        # name so starting the second aircraft cannot evict the first.
        rospy.init_node("gimbal_visual_servo", anonymous=True)

        ros_namespace = rospy.get_namespace().strip("/")
        default_uav_name = ros_namespace.split("/")[-1] if ros_namespace else "uav0"
        self.uav_name = str(
            rospy.get_param("~uav_name", default_uav_name)
        ).strip("/") or "uav0"
        self.vehicle_ns = "/" + self.uav_name
        self.mavros_ns = str(
            rospy.get_param("~mavros_ns", self.vehicle_ns + "/mavros")
        ).rstrip("/")
        if not self.mavros_ns.startswith("/"):
            self.mavros_ns = "/" + self.mavros_ns

        self.detection_input = str(
            rospy.get_param("~detection_input", "tracked")
        ).strip().lower()
        default_detection_topic = (
            "/{}/sar_yolo_detector/smart/selected_tracked_detections".format(self.uav_name)
            if self.detection_input in ("tracked", "tracked_detections", "smart_tracker")
            else "/detect_multi/inference_box"
        )
        self.detection_topic = str(
            rospy.get_param("~detection_topic", default_detection_topic)
        )
        self.target_track_id = int(rospy.get_param("~target_track_id", -1))
        self.target_class_id = int(rospy.get_param("~target_class_id", -1))
        self.minimum_detection_confidence = float(
            rospy.get_param("~minimum_detection_confidence", 0.0)
        )
        # 地理兴趣点仅对已有三维定位的检测生效。候选和飞机位姿必须处于同一地图系。
        self.geo_roi_enabled = bool(rospy.get_param("~geo_roi_enabled", False))
        self.geo_roi_topic = str(rospy.get_param(
            "~geo_roi_topic", self.vehicle_ns + "/sar_yolo_detector/task_candidates"))
        self.vehicle_pose_topic = str(rospy.get_param(
            "~vehicle_pose_topic", self.mavros_ns + "/local_position/pose"))
        self.roi_hold_timeout = max(0.0, float(rospy.get_param("~roi_hold_timeout", 30.0)))
        self.roi_pose_timeout = max(0.0, float(rospy.get_param("~roi_pose_timeout", 0.5)))
        self.roi_correction_gain = max(0.0, float(rospy.get_param("~roi_correction_gain", 1.0)))
        self._selected_track_id = None
        self._roi = None  # (source_track_id, frame_id, east, north, up, receive_wall_time)
        self._vehicle_pose = None  # (frame_id, east, north, up, receive_wall_time)
        self._prev_roi_angles = None
        self._roi_lock = threading.Lock()

        # ---------- 相机 / 图像参数 ----------
        self.width = int(rospy.get_param("~image_width", 1280))
        self.height = int(rospy.get_param("~image_height", 720))
        self.cx = self.width / 2.0
        self.cy = self.height / 2.0

        self.fov_x = math.radians(float(rospy.get_param("~fov_x_deg", 45.0)))
        aspect = self.width / float(self.height)
        self.fov_y = 2.0 * math.atan(math.tan(self.fov_x / 2.0) / aspect)

        # 焦距（像素），用于 atan 针孔模型把像素偏移精确换算成角度偏移
        self.fx = (self.width / 2.0) / math.tan(self.fov_x / 2.0)
        self.fy = (self.height / 2.0) / math.tan(self.fov_y / 2.0)

        # ---------- 控制参数 ----------
        # kp 是离散比例增益(每帧把角度误差乘以它累加到指令)。延迟回路的稳定性
        # 判据是【连续时间】带宽×延迟 < π/2(而非 kp×延迟帧数): kp_eff=kp×loop_hz。
        # 实测(测试9)链路延迟(渲染+检测推理+控制传输)≈0.4~0.5s, 软启动40%增益
        # (kp_eff=1.8, 1.8×0.5=0.9<1.57)误差414px平滑收敛零振荡; 1.5s后全增益
        # (kp_eff=4.5, 4.5×0.5=2.25>1.57)立即出现±140px过冲振荡(周期1.6s≈π/τ),
        # 与判据精确吻合。旧注释按 kp×延迟4帧=0.6<1 判断"稳定"是低估了延迟。
        # 0.09→kp_eff=2.7(×0.5s=1.35<1.57, τ=0.6s 时 1.62 临界, 回退点 0.08)。
        # 测试15 实测: 绕圈目标 yaw 峰值速度 5.7°/s, kp_eff=2.4 时稳态滞后
        # 5.7/2.4=2.4°≈64px → 误差 ±35~90px 波动即"画面晃动"主体(0.08→0.09
        # 滞后降至 5.7/2.7=2.1°≈57px)。ff 只能补瞬态(稳态时画面静止 ff=0,
        # 稳态滞后全由 P 承担——见下方 ff 注释), 故提高 kp 是缩滞后的唯一手段;
        # 0.10 带宽 3.0(×0.5=1.5<1.57)亦可试。检测噪声 ±2px(0.07°)→指令抖动
        # ±0.19°/s, 不可见。
        self.kp_yaw = float(rospy.get_param("~kp_yaw", 0.09))
        # 俯仰路通常比 yaw 更慢/更噪(检测 v 抖动大), 需更低增益。
        # 【0.075 已回退】静态目标实测触发 pitch 极限环(测试16: 目标飞机均静止,
        # pitch 误差 +36~-44px 来回摆, 周期≈2.1s, 持续 4s+ 不衰减, pitch_cmd 在
        # 54.6~60.1° 来回扫; 同段 yaw ±10px 正常)。带宽判据 kp_eff×τ=2.25×0.45
        # =1.01<1.57 仍振荡 → pitch 通道实际延迟大于判据假设: ①UE 云台 pitch 轴
        # 执行延迟比 yaw 长 ②检测 v 噪声(±10px)驱动起振 ③振荡周期 2.1s 时 ff 环路
        # 相位滞后≈(0.45s 执行+0.11s 低通)/2.1s×360°≈92° 由阻尼变正反馈持续供能。
        # 0.06→kp_eff=1.8(×0.45s=0.81)是测试14/15 验证过的稳定边界, 绕圈滞后
        # 3.7/1.8=2.06°≈55px 由 pitch 噪声/执行特性决定, 暂接受。
        self.kp_pitch = float(rospy.get_param("~kp_pitch", 0.06))
        # 微分(阻尼)增益，单位=秒(作用在滤波后角度误差的变化率上)。理论上 PD 微分能
        # 提供相位超前抑制"传输延迟+slew饱和"的极限环。但【默认关闭(=0)】：对滤波后误差
        # 做差分(÷dt≈33ms)会把检测框逐帧残余噪声放大约 30 倍注入指令 → 实测画面严重
        # 抖动。近距乱转的真正根因是 slew 追不上方位变化率(见 max_slew_yaw)，提高 slew
        # 即对症、无需微分。仅当检测非常干净且仍有延迟振荡时才手动 _kd_yaw:=0.03~0.05。
        self.kd_yaw = float(rospy.get_param("~kd_yaw", 0.0))
        self.kd_pitch = float(rospy.get_param("~kd_pitch", 0.0))
        # 积分增益(条件积分/anti-windup)：消除飞机/目标匀速运动时的稳态偏差。
        # 纯比例 e_ss=目标画面角速率/kp_eff，飞机 40m/s 近距掠过时方位速率可达
        # 46°/s，kp_eff=3°/s/° 需 15° 误差才追平 → 目标被拖出画面(实测 -190px)。
        # 积分补偿恒定漂移使 e_ss→0。四重保护防 windup(裸积分实测在 pitch+90
        # 限位饱和后反向抵抗、yaw 甩 130° 崩盘)：
        #   1) |err|>i_error_deg 不积分(目标跳变/逃逸不积累)。阈值 15→8°：初始
        #      大误差追击期(如接管时 471px≈16.9°)误差一降到 15° 以下积分即高速
        #      累积(0.63s 内积 4.4°·s), 目标到位后残留积分以 8.8°/s 继续推云台,
        #      过零迟滞 0.3° 期间持续错误推挤 → 过冲 81px 后回摆多次(实测测试13
        #      静止目标收敛 4.5s)。8° 使追击期(16.9°→8°)零累积, 残留推挤减至
        #      ~1/3; 稳态跟踪误差通常 <8°(测试11 绕圈峰值 6.5°), 行为不变。
        #   2) 指令接近限位时冻结(限位处不继续积分)
        #   3) clamp 封顶(ki×I_max=最大补偿角速度, 覆盖 46°/s 留裕度)
        #   4) 误差方向反转时清零(目标过中心/过冲, 旧积分方向必错, 丢弃)
        #   5) 追击期(误差快速减小 d_ang<-3°/s)积分按帧衰减 0.9(时间常数≈0.32s):
        #      追击段累积的积分在目标到位后仍会推挤 → 过冲二次回中(测试14:
        #      静止目标 264px 接管, 残留 3.6°·s 推 7°/s → 过冲 42px)。衰减后
        #      残留<0.5°/s 无推挤; 绕圈稳态 d_ang≈0 不触发, 无回归。
        self.ki_yaw = float(rospy.get_param("~ki_yaw", 2.0))
        self.ki_pitch = float(rospy.get_param("~ki_pitch", 1.5))
        self.i_error_deg = float(rospy.get_param("~i_error_deg", 8.0))
        self._i_yaw = 0.0
        self._i_pitch = 0.0
        self._i_yaw_max = 60.0     # ki_yaw×30 = 60°/s 补偿(slew 60 兜底)
        self._i_pitch_max = 40.0   # ki_pitch×26.7 = 40°/s 补偿
        # 指令步长低通系数(0~1, 默认 0.5)：对 P/I 合成的单帧步长做一阶平滑
        # (时间常数≈33ms@30Hz)，抑制死区边缘反复触发、检测微扰造成的指令微摆
        # (实测静止段指令 ±0.4° 摆动、盘旋段回摆)。延迟仅 1 帧可忽略。
        # 0=关闭。前馈不过此低通(其本身平滑)。
        self.step_alpha = float(rospy.get_param("~step_alpha", 0.5))
        self._y_step_f = 0.0
        self._p_step_f = 0.0
        # 目标像素速度前馈(运动跟随核心)：反馈式控制的速度∝误差——目标匀速/变速
        # 移动时误差先积累、云台才提速，误差进死区(±8px)后指令又冻结，表现为
        # "云台停顿、一顿一顿、速度忽快忽慢"。前馈把滤波后目标像素速度直接换算
        # 成角速度叠加进指令：目标动云台立即动(死区内也执行→消除停顿)、反馈只补
        # 残差(速度连续平滑)。历史教训：裸位置差分速度噪声(±120px/s≈±4.2°/s)
        # 注入指令→画面抖动(已回退)。三重防护：
        #   1) 用低通滤波后位置(filt_u/filt_v)差分(噪声已减半)
        #   2) 速度一阶低通 ff_alpha=0.30(时间常数≈0.11s)，白噪声衰减≈×0.39。
        #      0.30(而非 0.15)：目标方向反转/变速时前馈符号+幅值建立快一倍——反馈
        #      冻结(死区内)+前馈滞后是"停顿"根因(实测目标反向瞬间云台 0.5s 不动)。
        #      噪声代价小：位置滤波减半后再低通, 白噪声±12px/s 仍低于 20px/s 死区。
        #   3) 速度死区 ff_dz_px=15px/s(≈0.5°/s)：静止目标前馈=0，云台完全不动。
        #      15 高于低通后白噪声(~12px/s)防误动；原 20px/s 使 15~20px/s 慢速目标
        #      前馈被吞+8px 死区内 P 也不动 → 走走停停(实测测试10 目标 9px/s 时
        #      yaw_cmd 冻结 4s)。降死区后慢速目标由 P(死区 4px)+ff 共同连续驱动。
        #   4) 限幅 ff_max(低于 slew 留反馈裕度)：前馈不追超过云台物理能力的速率
        # 目标慢速移动 2°/s=57px/s >> 15px/s 死区, 正常通过。前馈不过 step 低通
        # (其本身已平滑)；invert 时与 P 项同取反；丢失/重捕获时状态清零。
        self.ff_alpha = float(rospy.get_param("~ff_alpha", 0.30))
        self.ff_dz_px = float(rospy.get_param("~ff_dz_px", 15.0))
        self.ff_max_yaw = float(rospy.get_param("~ff_max_yaw", 45.0))    # deg/s
        self.ff_max_pitch = float(rospy.get_param("~ff_max_pitch", 35.0))  # deg/s
        # 前馈误差门控(anti-fight, 见头部文档 ~ff_gate_*): |err|≥ff_gate_off_px 时
        # 前馈权重→0, ≤ff_gate_full_px 全开, 之间线性过渡。根因: 大误差瞬态(接管/
        # 目标被甩)云台以反馈速率猛转, 目标画面速度≈云台自身转速-目标慢速 → 前馈把
        # "云台自身转动"误当"目标运动", 输出与反馈反方向的角速度 → 对抗(实测测试15:
        # 接管 242px 段收敛 2.1s, ff 低通残留到位后仍推挤 → 过冲 21px 再回调)。
        # 门控关闭后反馈全力收敛, 到位快且无 ff 推挤; 稳态跟踪误差(绕圈实测
        # ±35~50px)远小于 full(40px)阈值, 正常跟踪 ff 全开不受影响。
        self.ff_gate_full_px = float(rospy.get_param("~ff_gate_full_px", 40.0))
        self.ff_gate_off_px = float(rospy.get_param("~ff_gate_off_px", 120.0))
        if self.ff_gate_off_px <= self.ff_gate_full_px:
            self.ff_gate_off_px = self.ff_gate_full_px + 1.0
        self._ff_vu = 0.0
        self._ff_vv = 0.0
        self._ff_t = None
        self._ff_pu = None
        self._ff_pv = None
        # 方向约定：图像 u 向右、v 向下；MAVLink yaw+ 向右、pitch+ 向上。
        # 目标在右(du>0) -> yaw+ ，无需翻转(yaw 全程机体相对, 无 negate)。
        # 俯仰：真实路径 pitch_cmd = -(相机世界俯仰)(stabilize_pitch_negate)，目标在上方
        # (dv<0)需相机抬高=pitch_cmd 减小，故 dv<0 -> pitch_step 负，无需翻转 -> False。
        # (simulate 投影已统一用相机实际世界俯仰，故此默认值对两模式都正确。)
        self.invert_yaw = bool(rospy.get_param("~invert_yaw", False))
        self.invert_pitch = bool(rospy.get_param("~invert_pitch", False))
        # 像素死区：目标在中心附近时不修正。默认 4px(原 8px)：目标低速漂移
        # (<ff死区 15px/s)时 8px 死区让目标先漂出 8px 才动→"走走停停"(实测测试10
        # yaw_cmd 冻结 4s)；4px 死区漂出即修, 云台连续微转。检测噪声 ±2px(0.07°)
        # 远低于 4px, 不会误触发。
        self.deadzone_px = float(rospy.get_param("~deadzone_px", 4.0))
        # 最大变化率: 必须高于目标最大方位变化率，否则云台物理上追不上(误差持续大、
        # 比例项恒饱和)。飞机 40m/s 掠过 50m 近距目标时方位变化率≈v/d≈46°/s，旧 30°/s
        # 追不上 → 比例项恒满速(bang-bang)+延迟 → 极限环(近距 yaw 来回扫)。提到 60°/s
        # 留出裕度。远目标方位变化慢、指令步长远低于此上限，提高上限不影响其稳定性。
        self.max_slew_yaw = float(rospy.get_param("~max_slew_yaw", 60.0))    # deg/s
        self.max_slew_pitch = float(rospy.get_param("~max_slew_pitch", 45.0))  # deg/s
        # 运动预测默认关闭：检测器噪声/振荡时，预测器把像素速度外推会把误差放大
        # (实测 proc 被推到 473/58 远超 raw)，加剧闭环振荡。检测干净且目标快速运动
        # 时可 _use_prediction:=true 恢复延迟补偿。
        self.use_prediction = bool(rospy.get_param("~use_prediction", False))
        self.predict_horizon = float(rospy.get_param("~predict_horizon", 0.04))  # s
        # 检测跳变门限(像素)：单帧位移超过它视为误检(检测器跳到别的目标/虚警)丢弃。
        # 250px 远大于真实可达位移(30°/s 云台 + 常见检测率)，但能剔除实测 740px 野值。
        # <=0 关闭门控。连续多次被门控则强制接纳(目标确实大位移, 重新捕获)。
        self.max_jump_px = float(rospy.get_param("~max_jump_px", 250.0))
        self._jump_reject_max = int(rospy.get_param("~jump_reject_max", 5))

        # ---------- 云台角度限位 ----------
        self.yaw_limit = float(rospy.get_param("~yaw_limit", 180.0))
        # pitch_cmd = -(相机世界俯仰)：pitch_cmd=+90 -> 相机正下方(-90°俯仰)，
        # pitch_cmd=-90 -> 正上方。飞机过顶时目标俯仰可达近 -90°(正下方)，故下限位
        # 取 +90(与上限 -90 对称全范围)；原 50° 只能下看 50°，过顶即撞限位无法居中。
        self.pitch_min = float(rospy.get_param("~pitch_min", -90.0))
        self.pitch_max = float(rospy.get_param("~pitch_max", 90.0))

        # ---------- 目标状态 ----------
        self.timeout = float(rospy.get_param("~target_timeout", 0.5))
        self.loop_hz = float(rospy.get_param("~loop_hz", 30.0))

        # ---------- 运行模式 ----------
        # tracking: 自动视觉伺服跟踪; manual: 手动输入姿态测试云台控制链路
        # simulate: 模拟运动目标(开环像素轨迹)测试自动连续跟踪
        self.mode = str(rospy.get_param("~mode", "tracking")).lower()

        # ---------- 模拟目标参数（仅 simulate 模式使用） ----------
        self.sim_pattern = str(rospy.get_param("~sim_pattern", "circle")).lower()
        # 虚拟目标角幅度(度)：云台会平滑地在这个角度范围内连续跟踪。
        # 闭环模式下这是决定“云台动多大”的唯一参数（取代旧的像素半径）。
        self.sim_amplitude_deg = float(rospy.get_param("~sim_amplitude_deg", 15.0))
        # 旧参数，仅为兼容保留；闭环 simulate 不再使用像素半径（开环像素半径会
        # 导致指令积分饱和、撞限位、画面抖动）。
        self.sim_radius_px = float(rospy.get_param("~sim_radius_px", 250.0))
        self.sim_period = float(rospy.get_param("~sim_period", 8.0))   # 单圈周期 s
        self.sim_hz = float(rospy.get_param("~sim_hz", 30.0))

        # ---------- 稳像后端参数 ----------
        # stabilize=True: 走本机 /mavlink/to 发 GIMBAL_MANAGER_SET_ATTITUDE(#282, 带
        #   YAW_LOCK)，交给 PX4 gimbal_manager 统一产生唯一的 #284 到设备(单一命令源)。
        # stabilize=False(默认): 走 mavros MountControl(mode=2)
        self.stabilize = bool(rospy.get_param("~stabilize", False))
        if self.geo_roi_enabled and not self.stabilize:
            raise ValueError("~geo_roi_enabled requires ~stabilize=true (world-locked gimbal)")
        if self.geo_roi_enabled and (self.detection_input not in
                                     ("tracked", "tracked_detections", "smart_tracker")):
            raise ValueError("~geo_roi_enabled requires tracked detections with a stable track_id")
        if self.geo_roi_enabled and TaskCandidateArray is None:
            raise RuntimeError("~geo_roi_enabled requires sar_yolo_detector/TaskCandidateArray")
        # #282 寻址 gimbal_manager：默认发给 PX4 gimbal manager 组件(154)，而不是广播到 0。
        # 你当前 PX4 的 gimbal_manager_status 也说明该组件是实际接收端。
        self.gimbal_sysid = int(rospy.get_param("~gimbal_sysid", 1))       # target_system
        self.manager_compid = int(rospy.get_param("~manager_compid", 154))
        self.gimbal_device_id = int(rospy.get_param("~gimbal_device_id", 154))
        # 保留旧参数名，仅作为兼容；若未显式指定 manager_compid，优先使用它。
        self.gimbal_compid = int(rospy.get_param("~gimbal_compid", self.manager_compid))
        self.src_sysid = int(rospy.get_param("~src_sysid", 255))          # 仿 QGC
        self.src_compid = int(rospy.get_param("~src_compid", 190))        # MAV_COMP_ID_MISSIONPLANNER
        self.stabilize_pitch_negate = bool(rospy.get_param("~stabilize_pitch_negate", True))
        self.yaw_offset_deg = float(rospy.get_param("~yaw_offset_deg", 0.0))

        # 飞机罗盘航向（世界 yaw 参考），用于把机体相对 yaw 转到世界系
        self.compass_hdg = 0.0
        self.compass_received = False
        self._compass_last_raw = None
        self._compass_lock = threading.Lock()

        # 航向前馈：记住上一帧航向，控制环里用最小角差预补偿 yaw_cmd，
        # 使 (航向 + yaw_cmd) 在目标居中时保持恒定 → 真正的 yaw 稳像，
        # 飞机转向时云台不会先跟着转再被 IBVS 拉回。
        self._prev_heading_ff = None
        # 单帧航向变化上限(度)：超过视为罗盘野值，不施加前馈也不更新基准。
        # 30Hz 下 12°/帧 ≈ 360°/s，远超真实固定翼转向率，足以剔除跳变野值。
        self._heading_ff_max_step = float(
            rospy.get_param("~heading_ff_max_step", 12.0))

        # pymavlink 编码器（仅 stabilize 需要）
        self._mav = None
        if self.stabilize:
            if not _HAVE_PYMAVLINK:
                if self.geo_roi_enabled:
                    raise RuntimeError("geo ROI requires pymavlink for world-locked commands")
                rospy.logerr(
                    "[GVS] stabilize 需要 pymavlink 但未安装, 回退到 MountControl(无稳像)。"
                    " 安装: pip install pymavlink")
                self.stabilize = False
            else:
                self._mav = mavlink2.MAVLink(
                    None, srcSystem=self.src_sysid, srcComponent=self.src_compid)
                self._mav.mavlink20 = True  # msgid 284 > 255, 必须 MAVLink v2

        # ---------- 内部绝对指令状态（机体相对） ----------
        self.yaw_cmd = 0.0
        self.pitch_cmd = 0.0
        self.lock = threading.Lock()

        # simulate 稳像闭环：虚拟目标定义在世界系，需要一个世界 yaw 锚点作为参考。
        # 首次拿到罗盘航向时捕获(= 当时的世界指向)，之后虚拟目标相对它运动，投影
        # 误差用"世界系指向 - 锚点"计算，与航向前馈解耦(见 sim_target_position)。
        self._sim_world_anchor = None
        # 最近一次实际下发给云台的世界 yaw(在 control_loop 航向前馈之后原子算出)。
        # simulate 投影必须用它而非在生成线程里用 yaw_cmd+heading 重算：快速转向时
        # 罗盘回调已更新 heading 但前馈尚未补偿 yaw_cmd，重算会读到中间态使指向突跳
        # 航向增量 → du 尖峰 → IBVS 抽动 → 画面抖动。此值与云台实际指令一致且平滑。
        self._last_world_yaw = None
        # 是否已接管云台(首次获得有效目标后置 True)。无有效目标时不发送姿态指令，
        # 稳像模式也不持有 gimbal_manager 主控权；目标超时后释放主控权。接管瞬间
        # 用 #285 反馈把 yaw_cmd/pitch_cmd 初始化到云台当前实际姿态(无跳变)。
        self._servo_engaged = False
        self._gimbal_control_claimed = False
        self._gimbal_claim_retry_after = 0.0
        self._gimbal_release_retry_after = 0.0
        # 云台姿态反馈(#285)最新值 (yaw_deg, pitch_deg, yaw_lock)。仅稳像模式订阅填充，
        # 非稳像模式恒为 None(接管时回退到 0)。在此无条件定义，_engage_servo 才能安全读取。
        self._gimbal_fb = None
        self._gimbal_fb_lock = threading.Lock()
        # #285 反馈角速度估计(世界系, 供 ff 去耦)：云台实际世界角速度 =
        # 反馈姿态差分(≈10Hz) 低通。画面速度 = 目标世界运动 - 云台世界转动,
        # ff 用它把云台自转从画面速度中分离 → 静态目标 ff→0(断开延迟正反馈环),
        # 绕圈稳态 ff=目标真实速度(消除 P 独扛的稳态滞后)。
        self.fb_rate_alpha = float(rospy.get_param("~fb_rate_alpha", 0.15))
        self._fb_rate_t = None
        self._fb_yaw_prev = None
        self._fb_pitch_prev = None
        self._fb_yaw_rate = 0.0
        self._fb_pitch_rate = 0.0
        # 接管软启动(anti-overshoot)：接管瞬间误差通常很大(目标在画面边缘)，全速
        # P+I 猛转 + 链路延迟 → 目标冲过中心来回摆(实测 ±230px 冲程、来回 3 次)。
        # 接管后 engage_ramp_s 内 P/D/I 增益从 70% 线性恢复至 100%：kp_eff×延迟<1
        # 即无过冲, 软启动期带宽 0.7×2.4=1.68°/s/° ×0.5s=0.84<1 ✓(原 40% 起步也
        # 稳定但初始收敛慢: 测试10 实测 -504px 误差到中心 2.3s, 软启动期占大半;
        # 70%+1.0s 后 ~1.4s 收敛, 全程无过冲)。前馈不过此缩放(跟踪实时性优先)。
        self.engage_ramp_s = float(rospy.get_param("~engage_ramp_s", 1.0))
        self._engage_t = None
        self._gain_scale = 1.0

        # ---------- 检测目标缓存 ----------
        self.target = None       # (u, v)
        self.last_seen = 0.0
        self._jump_reject_count = 0  # 连续被跳变门控拒绝的次数(达上限强制接纳重捕获)

        # ---------- 滤波器 / 预测器 ----------
        # alpha 越小越平滑(抑制检测抖动)但滞后越大。0.3 实测对噪声检测器平滑效果好;
        # 检测干净/目标快速运动时可上调到 0.5 减小滞后。
        filter_alpha = float(rospy.get_param("~filter_alpha", 0.3))
        self.filt_u = LowPassFilter(filter_alpha)
        self.filt_v = LowPassFilter(filter_alpha)
        self.predictor = TargetPredictor()

        # 微分(阻尼)项状态：上一帧滤波后角度误差及时间戳，用于计算 d误差/dt。
        # None 表示尚无历史(首帧/目标重捕获)，此时微分项取 0 避免尖峰。
        self._prev_ang_yaw = None
        self._prev_ang_pitch = None
        self._prev_ang_t = None

        # ---------- ROS 接口 ----------
        self.pub = rospy.Publisher(
            self.mavros_ns + "/mount_control/command", MountControl, queue_size=10)

        # 先把反馈/诊断状态初始化好，避免回调在订阅前就收到消息时触发 AttributeError。
        self._n282_mavros = 0
        self._n530 = 0
        self._n285 = 0
        self._last_fb_src = None
        self._link_log_t = 0.0
        # simulate 模式下不订阅检测 topic：外部 tracking 节点发布的检测框会
        # 覆盖模拟目标(self.target)，其像素位置无钳位 → 误差跳变 → 云台抖动。
        if self.mode != "simulate":
            if self.detection_input in ("tracked", "tracked_detections", "smart_tracker"):
                if TrackedDetection2DArray is None:
                    raise RuntimeError(
                        "detection_input=tracked requires sar_yolo_detector messages; "
                        "source the workspace devel/setup.bash before starting this script"
                    )
                self.detection_subscriber = rospy.Subscriber(
                    self.detection_topic,
                    TrackedDetection2DArray,
                    self.tracked_detections_cb,
                    queue_size=1,
                )
            elif self.detection_input in ("bbox2d", "boundingbox2d", "legacy"):
                self.detection_subscriber = rospy.Subscriber(
                    self.detection_topic,
                    BoundingBox2D,
                    self.detection_cb,
                    queue_size=1,
                )
            else:
                raise ValueError(
                    "~detection_input must be 'tracked' or 'bbox2d', got {!r}".format(
                        self.detection_input
                    )
                )
            rospy.loginfo(
                "[GVS] detection input: %s (%s)",
                self.detection_topic,
                self.detection_input,
            )
        if self.geo_roi_enabled:
            rospy.Subscriber(self.geo_roi_topic, TaskCandidateArray,
                             self._roi_candidates_cb, queue_size=1)
            rospy.Subscriber(self.vehicle_pose_topic, PoseStamped,
                             self._vehicle_pose_cb, queue_size=1)
            rospy.loginfo("[GVS] 地理兴趣点: %s, 飞机位姿: %s (要求相同 ENU 地图系)",
                          self.geo_roi_topic, self.vehicle_pose_topic)

        # 稳像后端：原始 mavlink 发送 + 罗盘航向订阅
        self.mavlink_pub = None
        if self.stabilize:
            self.mavlink_pub = rospy.Publisher(
                self.vehicle_ns + "/mavlink/to", Mavlink, queue_size=10)
            rospy.Subscriber(
                self.mavros_ns + "/global_position/compass_hdg", Float64,
                self.compass_hdg_cb, queue_size=1)
            # 云台姿态反馈 #285(GIMBAL_DEVICE_ATTITUDE_STATUS)：mavros 不解码 gimbal
            # v2，需从本机 /mavlink/from 自行解析(同 gimbal_diag.py)。首次接管
            # 云台时用它把指令初始化到云台当前实际姿态，避免第一帧跳到正前方。
            # (_gimbal_fb / _gimbal_fb_lock 已在上方无条件定义)
            self._fb_parser = mavlink2.MAVLink(None)
            self._fb_parser.robust_parsing = True
            rospy.Subscriber(
                self.vehicle_ns + "/mavlink/from", Mavlink,
                self._gimbal_fb_cb, queue_size=100)

            # mavros command 服务客户端：以 mavros 原生方式发 command 1001 抢占/释放主控(真机首选)。
            # ServiceProxy 创建不会因服务不存在而失败，调用时才报错，故在 claim 里
            # 再 wait_for_service。
            self._cmd_client = None
            if _HAVE_CMD_SRV:
                self._cmd_client = rospy.ServiceProxy(
                    self.mavros_ns + "/cmd/command", CommandLong)
            # tmux/px4 shell 抢占只作为开发调试使用，实际场景必须通过 mavros
            # 的 COMMAND_LONG(command=1001) 切换主控。默认关闭，避免在真机/实际飞控上
            # 依赖不现实的控制台命令。
            self._use_tmux_claim = bool(rospy.get_param("~use_tmux_claim", False))
            self._allow_non_mavros_claim = bool(
                rospy.get_param("~allow_non_mavros_claim", False))

        rospy.loginfo(
            "[GVS] 初始化: %dx%d, FOVx=%.1f° FOVy=%.1f°, fx=%.1f fy=%.1f, "
            "Kp(yaw=%.2f pitch=%.2f) Ki(%.1f/%.1f), invert(yaw=%s pitch=%s), "
            "slew(yaw=%.0f pitch=%.0f deg/s), ff(alpha=%.2f dz=%.0fpx max=%.0f/%.0f "
            "gate=%.0f~%.0fpx), engage_ramp=%.1fs",
            self.width, self.height, math.degrees(self.fov_x),
            math.degrees(self.fov_y), self.fx, self.fy,
            self.kp_yaw, self.kp_pitch, self.ki_yaw, self.ki_pitch,
            self.invert_yaw, self.invert_pitch,
            self.max_slew_yaw, self.max_slew_pitch,
            self.ff_alpha, self.ff_dz_px, self.ff_max_yaw, self.ff_max_pitch,
            self.ff_gate_full_px, self.ff_gate_off_px,
            self.engage_ramp_s)
        rospy.loginfo(
            "[GVS] 发送后端: %s",
            ("稳像(GIMBAL_MANAGER_SET_ATTITUDE#282+YAW_LOCK, 主链路经 %s/mavlink/to, "
             "目标 sys/comp=%d/%d, device_id=%d, 源 %d/%d)"
             % (self.vehicle_ns, self.gimbal_sysid, self.manager_compid, self.gimbal_device_id,
                self.src_sysid, self.src_compid)) if self.stabilize
            else "MountControl(mode=2, 无稳像)")

    # ================= 罗盘航向回调 =================
    def compass_hdg_cb(self, msg):
        """记录飞机罗盘航向(度, 0=北, 顺时针)。

        这里只原样保存最新航向（它本身是 [0,360) 内的合法角度）。连续化与
        野值剔除放到航向前馈里做——那里用 wrap_180 计算最小角差，能正确处理
        过北(359°->1°)的正常跳变，再按单帧幅值上限剔除真正的野值。
        """
        raw = float(msg.data)
        with self._compass_lock:
            self.compass_hdg = raw
            self.compass_received = True

    # ================= 云台姿态反馈(#285)解析 =================
    @staticmethod
    def _rebuild_frame(msg):
        """把 mavros_msgs/Mavlink 的字段拼回完整 MAVLink 帧字节(同 gimbal_diag.py)。"""
        payload = b''.join(struct.pack('<Q', w) for w in msg.payload64)
        payload = payload[:msg.len]
        msgid = msg.msgid & 0xFFFFFF
        if msg.magic == 253:  # MAVLink v2: 10 字节头
            header = struct.pack(
                '<BBBBBBB', msg.magic, msg.len, msg.incompat_flags,
                msg.compat_flags, msg.seq, msg.sysid, msg.compid)
            header += bytes([msgid & 0xFF, (msgid >> 8) & 0xFF,
                             (msgid >> 16) & 0xFF])
        else:  # MAVLink v1: 6 字节头, msgid 1 字节
            header = struct.pack(
                '<BBBBBB', msg.magic, msg.len, msg.seq, msg.sysid,
                msg.compid, msgid & 0xFF)
        frame = header + payload + struct.pack('<H', msg.checksum)
        if msg.magic == 253 and (msg.incompat_flags & 0x01) and msg.signature:
            frame += bytes(msg.signature)
        return frame

    def _gimbal_fb_cb(self, msg):
        """解析本机 /mavlink/from，提取 #285 云台姿态反馈。

        YAW_LOCK 置位时四元数在世界系(yaw 相对正北)，否则在机体系(yaw 相对机头)。
        记录 (yaw_deg, pitch_deg, yaw_lock) 供首次接管时初始化指令。
        """
        try:
            frame = self._rebuild_frame(msg)
            parsed = self._fb_parser.parse_buffer(frame)
        except Exception:
            return
        if not parsed:
            return
        for m in parsed:
            if m.get_msgId() != MSG_GIMBAL_DEVICE_ATTITUDE_STATUS:
                continue
            flags = int(m.flags)
            q = list(m.q)  # [w, x, y, z]
            norm = math.sqrt(sum(v * v for v in q))
            if norm <= 0.5:
                continue
            w, x, y, z = [v / norm for v in q]
            pitch_rad = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
            yaw_rad = math.atan2(2.0 * (w * z + x * y),
                                 1.0 - 2.0 * (y * y + z * z))
            yaw_deg = math.degrees(yaw_rad)
            pitch_deg = math.degrees(pitch_rad)
            yaw_lock = bool(flags & GIMBAL_DEVICE_FLAGS_YAW_LOCK)
            # 诊断：记录 #285 源身份与计数，证明反馈链路是否存活
            self._n285 += 1
            self._last_fb_src = (msg.sysid, msg.compid)
            rospy.loginfo_throttle(
                5.0,
                "[GVS_LINK] #285← 源sys/comp=%d/%d yaw=%.1f pitch=%.1f yaw_lock=%s "
                "(累计%d)",
                msg.sysid, msg.compid, yaw_deg, pitch_deg, yaw_lock, self._n285)
            # 世界角速度估计(差分+低通)：供 ff 去耦(分离目标运动与云台自转)。
            # yaw 经 wrap_180 防 ±180 跳变；pitch 符号(#285 抬头为正)与画面系
            # 相反, 去耦时的符号在 ff 侧处理。
            now_fb = msg.header.stamp.to_sec() if hasattr(msg.header.stamp, "to_sec") else time.time()
            if self._fb_rate_t is not None:
                dtf = now_fb - self._fb_rate_t
                if dtf > 1e-6:
                    yr = self.wrap_180(yaw_deg - self._fb_yaw_prev) / dtf
                    pr = (pitch_deg - self._fb_pitch_prev) / dtf
                    self._fb_yaw_rate += self.fb_rate_alpha * (yr - self._fb_yaw_rate)
                    self._fb_pitch_rate += self.fb_rate_alpha * (pr - self._fb_pitch_rate)
            self._fb_rate_t = now_fb
            self._fb_yaw_prev = yaw_deg
            self._fb_pitch_prev = pitch_deg
            with self._gimbal_fb_lock:
                self._gimbal_fb = (yaw_deg, pitch_deg, yaw_lock)

    def _disengage_servo(self):
        """停止跟踪并归还主控权，等待下一帧有效检测重新接管。"""
        if self.stabilize and self._gimbal_control_claimed:
            self._release_gimbal_control()

        self._servo_engaged = False
        self._engage_t = None
        self._prev_heading_ff = None
        self._prev_ang_yaw = None
        self._prev_ang_pitch = None
        self._prev_ang_t = None
        self._i_yaw = 0.0
        self._i_pitch = 0.0
        self._y_step_f = 0.0
        self._p_step_f = 0.0
        self._ff_vu = 0.0
        self._ff_vv = 0.0
        self._ff_t = None
        self._ff_pu = None
        self._ff_pv = None
        rospy.loginfo("[GVS] 检测框超时，停止发送并释放云台控制权")

    def _engage_servo(self):
        """首次获得有效目标：接管云台控制。

        先用 #285 反馈的当前姿态初始化 yaw_cmd/pitch_cmd，避免接管时把
        云台硬拉到 0°/正前方。若还没收到反馈则一直等待(告警)，绝不带着
        空白的默认值接管——错误起步基准会让首帧指令偏离云台真实指向任意
        角度，云台猛转 → 目标出画 → 检测跳变 → 乱转(实测 26°)。"""
        now = time.time()
        with self._gimbal_fb_lock:
            fb = self._gimbal_fb

        # 航向必须就绪才能接管：世界系指令 = 航向 + yaw_cmd。接管时罗盘若还是
        # 初始 0.0（订阅后 ~80ms 内首条 compass_hdg 未到），yaw_cmd 按 0 航向
        # 反解，首帧世界指令即偏离云台真实指向一个航向角（实测偏 156° → 云台
        # 甩走 → 目标甩出视场）。罗盘通常 ~50ms 到达，这里不设截止时间：未就绪
        # 就保持等待并告警，绝不冒险用 0 航向接管。
        if self.stabilize:
            with self._compass_lock:
                have_hdg = self.compass_received
            if not have_hdg:
                rospy.logwarn_throttle(
                    2.0, "[GVS] 罗盘航向未就绪, 暂缓接管云台(等 compass_hdg)")
                return False

        if self.stabilize and fb is None:
            # 没有 #285 反馈就绝不接管：起步基准必须来自云台真实姿态，否则首帧
            # 世界指令与真实指向偏差一个任意角(实测主循环被阻塞 0.35s 后 0.25s
            # 等待超时 → 26° 偏差 → 云台猛转 → 目标出画 → 检测跳变 → 强制重
            # 捕获 → 乱转)。#285 链路已实测稳定(10Hz)，若收不到说明链路异常，
            # 云台保持不动比乱转安全。
            rospy.logwarn_throttle(
                2.0, "[GVS] 等待 #285 云台反馈(未收到前不接管, 避免错误起步基准)")
            return False

        with self._compass_lock:
            hdg = self.compass_hdg

        if fb is not None:
            fb_yaw, fb_pitch, yaw_lock = fb
            if yaw_lock:
                # 世界系反馈：world_yaw = heading + yaw_cmd + offset → 反解机体相对 yaw
                self.yaw_cmd = self.wrap_180(fb_yaw - hdg - self.yaw_offset_deg)
                # pitch 反馈相对地平线，构造四元数时取反(见 _send_stabilized)
                self.pitch_cmd = -fb_pitch if self.stabilize_pitch_negate else fb_pitch
            else:
                # 机体系反馈：yaw 已是机体相对，可直接用；pitch 非地平线系，保守置 0
                self.yaw_cmd = self.wrap_180(fb_yaw)
                self.pitch_cmd = 0.0
            self.yaw_cmd = self.clamp(self.yaw_cmd, -self.yaw_limit, self.yaw_limit)
            self.pitch_cmd = self.clamp(
                self.pitch_cmd, self.pitch_min, self.pitch_max)
        else:
            # 无反馈时不要把指令硬拉回 0°，保持当前指令，避免接管瞬间出现大跳变。
            self.yaw_cmd = self.clamp(self.yaw_cmd, -self.yaw_limit, self.yaw_limit)
            self.pitch_cmd = self.clamp(self.pitch_cmd, self.pitch_min, self.pitch_max)

        self._prev_heading_ff = None  # 复位前馈基准
        # 复位微分项历史：接管后首帧 d误差/dt 取 0，避免用陈旧前值产生尖峰
        self._prev_ang_yaw = None
        self._prev_ang_pitch = None
        self._prev_ang_t = None
        # 积分从零起步：接管瞬间误差可能很大, 避免用旧积分推错方向
        self._i_yaw = 0.0
        self._i_pitch = 0.0
        # 步长低通状态清零：接管后首帧不继承旧平滑值
        self._y_step_f = 0.0
        self._p_step_f = 0.0
        # 目标速度前馈状态清零：接管后首帧前馈从零起步(无陈旧速度)
        self._ff_vu = 0.0
        self._ff_vv = 0.0
        self._ff_t = None
        self._ff_pu = None
        self._ff_pv = None
        self._servo_engaged = True
        self._engage_t = time.time()  # 软启动计时起点(P/I 增益 70%→100% 线性恢复)
        rospy.loginfo(
            "[GVS] 首次获得目标, 接管云台控制 (从当前姿态起步 yaw_cmd=%.1f° "
            "pitch_cmd=%.1f°, 反馈=%s, 软启动 %.1fs 70%%→100%%)",
            self.yaw_cmd, self.pitch_cmd,
            ("yaw_lock" if (fb and fb[2]) else ("follow" if fb else "无")),
            self.engage_ramp_s)
        return True

    # ================= 检测回调 =================
    def detection_cb(self, msg):
        self._inject_target(msg.center.x, msg.center.y)

    def tracked_detections_cb(self, msg):
        """Extract one tracked bbox center and feed the existing IBVS loop."""
        candidates = []
        for tracked in msg.detections:
            if self.target_track_id >= 0 and tracked.track_id != self.target_track_id:
                continue

            detection = tracked.detection
            scores = [
                float(result.score)
                for result in detection.results
                if self.target_class_id < 0 or int(result.id) == self.target_class_id
            ]
            if detection.results and not scores:
                continue
            confidence = max(scores) if scores else 0.0
            if (not math.isfinite(confidence) or
                    confidence < self.minimum_detection_confidence):
                continue

            center_u = float(detection.bbox.center.x)
            center_v = float(detection.bbox.center.y)
            if math.isfinite(center_u) and math.isfinite(center_v):
                candidates.append((confidence, int(tracked.track_id), center_u, center_v))

        if not candidates:
            # No detection is intentional: the existing target_timeout logic
            # handles loss. Never inject (0, 0), which would look like a target.
            rospy.logwarn_throttle(
                5.0, "[GVS] no valid SmartTracker detection on %s", self.detection_topic
            )
            return

        # 同一目标仍在视野内时保持 ID，避免其他目标置信度短暂上升导致云台切换。
        with self._roi_lock:
            selected_id = self._selected_track_id
        same_track = [item for item in candidates if item[1] == selected_id]
        confidence, track_id, center_u, center_v = max(
            same_track or candidates, key=lambda item: item[0])
        with self._roi_lock:
            if track_id != self._selected_track_id:
                self._roi = None
                self._prev_roi_angles = None
            self._selected_track_id = track_id
        self._inject_target(center_u, center_v)
        rospy.loginfo_throttle(
            5.0,
            "[GVS] SmartTracker target center u=%.1f v=%.1f px, confidence=%.3f",
            center_u,
            center_v,
            confidence,
        )

    def _vehicle_pose_cb(self, msg):
        p = msg.pose.position
        if not msg.header.frame_id or not all(math.isfinite(x) for x in (p.x, p.y, p.z)):
            return
        with self._roi_lock:
            self._vehicle_pose = (msg.header.frame_id, p.x, p.y, p.z, time.time())

    def _roi_candidates_cb(self, msg):
        """仅接纳与当前视觉目标同 ID、已确认且三维定位有效的候选。"""
        with self._roi_lock:
            selected_id = self._selected_track_id
        if selected_id is None or selected_id < 0:
            return
        for candidate in msg.candidates:
            if (not candidate.source_track_id_valid or
                    candidate.source_track_id != selected_id or
                    candidate.status not in (candidate.CONFIRMED, candidate.UPDATED) or
                    not candidate.localization_valid):
                continue
            pose = candidate.task_pose
            p = pose.pose.position
            if not pose.header.frame_id or not all(
                    math.isfinite(x) for x in (p.x, p.y, p.z)):
                continue
            with self._roi_lock:
                if self._selected_track_id == selected_id:
                    first_lock = self._roi is None
                    self._roi = (selected_id, pose.header.frame_id,
                                 p.x, p.y, p.z, time.time())
                else:
                    first_lock = False
            if first_lock:
                rospy.loginfo("[GVS] 已锁定目标 ID=%d 的地图兴趣点 frame=%s",
                              selected_id, pose.header.frame_id)
            return

    @staticmethod
    def _roi_angles(vehicle, target, heading, yaw_offset, pitch_negate):
        """同一 ENU 地图系的两点 -> 机体相对 yaw 与云台 pitch 指令(度)。"""
        east = target[2] - vehicle[1]
        north = target[3] - vehicle[2]
        up = target[4] - vehicle[3]
        horizontal = math.hypot(east, north)
        if horizontal < 0.1 and abs(up) < 0.1:
            return None
        world_yaw = math.degrees(math.atan2(east, north))
        elevation = math.degrees(math.atan2(up, horizontal))
        yaw = GimbalVisualServo.wrap_180(world_yaw - heading - yaw_offset)
        pitch = -elevation if pitch_negate else elevation
        return yaw, pitch

    def _apply_geo_roi(self, now, dt):
        """位置变化前馈加有界闭环修正；无新检测时继续看向已锁定地面点。"""
        if not self.geo_roi_enabled or not self._servo_engaged:
            return False
        with self._roi_lock:
            roi, vehicle, selected_id = self._roi, self._vehicle_pose, self._selected_track_id
        if roi is None or vehicle is None or roi[0] != selected_id:
            self._prev_roi_angles = None
            return False
        if now - roi[5] > self.roi_hold_timeout or now - vehicle[4] > self.roi_pose_timeout:
            self._prev_roi_angles = None
            rospy.logwarn_throttle(2.0, "[GVS] ROI 或飞机位姿过期，暂停地理前馈")
            return False
        if roi[1] != vehicle[0]:
            self._prev_roi_angles = None
            rospy.logwarn_throttle(2.0, "[GVS] ROI frame=%s 与飞机 frame=%s 不一致",
                                   roi[1], vehicle[0])
            return False
        with self._compass_lock:
            heading = self.compass_hdg
        desired = self._roi_angles(vehicle, roi, heading, self.yaw_offset_deg,
                                   self.stabilize_pitch_negate)
        if desired is None:
            return False
        desired_yaw, desired_pitch = desired
        desired_world_yaw = self.wrap_180(desired_yaw + heading + self.yaw_offset_deg)
        # 位置前馈补偿飞机高速平移；残余缓慢拉回几何视线，视觉误差仍可修正标定偏差。
        ff_yaw = 0.0
        ff_pitch = 0.0
        if self._prev_roi_angles is not None:
            ff_yaw = self.wrap_180(desired_world_yaw - self._prev_roi_angles[0])
            ff_pitch = desired_pitch - self._prev_roi_angles[1]
        self._prev_roi_angles = (desired_world_yaw, desired_pitch)
        err_yaw = self.wrap_180(desired_yaw - self.yaw_cmd)
        err_pitch = desired_pitch - self.pitch_cmd
        yaw_step = ff_yaw + self.roi_correction_gain * err_yaw * dt
        pitch_step = ff_pitch + self.roi_correction_gain * err_pitch * dt
        self.yaw_cmd = self.clamp(
            self.wrap_180(self.yaw_cmd + self.clamp(
                yaw_step, -self.max_slew_yaw * dt, self.max_slew_yaw * dt)),
            -self.yaw_limit, self.yaw_limit)
        self.pitch_cmd = self.clamp(
            self.pitch_cmd + self.clamp(
                pitch_step, -self.max_slew_pitch * dt, self.max_slew_pitch * dt),
            self.pitch_min, self.pitch_max)
        return True

    def _inject_target(self, u, v):
        """把一个目标像素坐标注入控制链路。真实检测回调与模拟目标生成器
        都走这里，保证 simulate 模式与 tracking 模式使用完全相同的处理流程。

        含检测跳变门控：检测器偶发误检(跳到别的目标/虚警)会产生远超物理可达
        速度的单帧位移(实测 (648,401)->(22,5) 约 740px)。直接采纳会把云台猛地
        踹偏并污染预测器速度估计 → 振荡。故位移超过 max_jump_px 的检测视为野值
        丢弃；连续丢弃达上限则判定目标确实大位移、强制接纳以重新捕获。"""
        now = time.time()

        if self.max_jump_px > 0.0:
            with self.lock:
                last = self.target
            if last is not None:
                du = u - last[0]
                dv = v - last[1]
                jump = math.hypot(du, dv)
                if jump > self.max_jump_px:
                    self._jump_reject_count += 1
                    if self._engage_t is not None:
                        # 软启动期云台正高速转向目标, 画面大位移多为甩动中的重检
                        # (或目标被甩出画面), 强制重新捕获可能咬住错误目标追丢。
                        # 只丢弃, 计数钳在 max-1 永远不触发强制接纳; 1.5s 后自动恢复。
                        self._jump_reject_count = min(
                            self._jump_reject_count, self._jump_reject_max - 1)
                        rospy.logwarn_throttle(
                            2.0,
                            "[GVS] 软启动期检测跳变 %.0fpx, 丢弃(暂不强制重新捕获)",
                            jump)
                        return
                    if self._jump_reject_count < self._jump_reject_max:
                        rospy.logwarn_throttle(
                            2.0,
                            "[GVS] 检测跳变 %.0fpx > %.0fpx, 视为误检丢弃(连续%d/%d)",
                            jump, self.max_jump_px,
                            self._jump_reject_count, self._jump_reject_max)
                        return
                    # 连续多次被拒：目标确实大位移，强制接纳重新捕获
                    rospy.logwarn(
                        "[GVS] 连续 %d 次大跳变, 判定目标大位移, 强制重新捕获",
                        self._jump_reject_count)
                    self._jump_reject_count = 0
                else:
                    self._jump_reject_count = 0

        with self.lock:
            self.target = (u, v)
            self.last_seen = now
        self.predictor.update(u, v, now)

    # ================= 像素偏移 -> 角度偏移 =================
    def pixel_to_angle(self, du, dv):
        """使用 atan 针孔模型，大偏角时比线性近似更精确。返回弧度。"""
        ang_yaw = math.atan2(du, self.fx)
        ang_pitch = math.atan2(dv, self.fy)
        return ang_yaw, ang_pitch

    @staticmethod
    def clamp(val, lo, hi):
        return max(lo, min(hi, val))

    @staticmethod
    def wrap_180(deg):
        """规范化到 [-180, 180)。"""
        return (deg + 180.0) % 360.0 - 180.0

    # ================= 航向前馈（yaw 稳像核心） =================
    def _apply_heading_feedforward(self):
        """稳像模式下，用飞机航向变化预补偿 yaw_cmd，保持世界 yaw 恒定。

        问题背景：_send_stabilized 里 世界yaw = 航向 + yaw_cmd。若 yaw_cmd 不变，
        飞机一转向，世界yaw 就跟着变 → 云台先随飞机转，随后 IBVS 才把它拉回，
        表现为“航向变化超过一定角度后云台跟着旋转后再指回原方向”。

        解法：每一帧把航向增量 Δh 从 yaw_cmd 里减掉，使 (航向 + yaw_cmd) 在
        目标居中(无 IBVS 修正)时保持恒定。这样世界yaw 只随 IBVS 修正演化，
        与飞机航向解耦 → 真正的 yaw 稳像。

        关键：无论 Δh 多大都必须施加补偿，不能“野值跳过”。因为 世界yaw = 航向
        + yaw_cmd，航向跳变 Δh 时同步把 yaw_cmd 减 Δh，世界yaw 恰好抵消不变；
        若跳过补偿，世界yaw 会跟着航向突跳 Δh → 云台指令阶跃 → 画面抖动(实测
        快速转向单帧 Δ>12° 触发跳过时云台图像抖动)。故此处恒施加补偿，仅对超大
        单帧增量限频告警(便于排查罗盘异常)，绝不改变/跳过补偿量。用 wrap_180 求
        最小角差，正确处理过北(359°->1°)。
        """
        if not self.stabilize or not self.compass_received:
            return

        with self._compass_lock:
            heading = self.compass_hdg

        if self._prev_heading_ff is None:
            self._prev_heading_ff = heading
            return

        delta = self.wrap_180(heading - self._prev_heading_ff)
        self._prev_heading_ff = heading

        if abs(delta) > self._heading_ff_max_step:
            # 仅告警，绝不跳过：跳过补偿会让世界 yaw 突跳 → 云台抖动。
            rospy.logwarn_throttle(
                2.0, "[GVS] 航向单帧变化较大(Δ%.1f°>%.1f°), 仍施加补偿以保稳像",
                delta, self._heading_ff_max_step)

        # 前馈补偿：抵消航向变化，保持世界 yaw 恒定
        self.yaw_cmd = self.wrap_180(self.yaw_cmd - delta)

    # ================= 控制主循环 =================
    def control_loop(self):
        rate = rospy.Rate(self.loop_hz)
        last_t = time.time()
        last_log = 0.0

        try:
            while not rospy.is_shutdown():
                now = time.time()
                dt = now - last_t
                last_t = now
                if dt <= 0.0 or dt > 1.0:
                    dt = 1.0 / self.loop_hz

                with self.lock:
                    target = self.target
                    age = now - self.last_seen

                target_valid = (target is not None) and (age < self.timeout)

                # 首次获得有效目标才接管云台：此前【不下发任何姿态指令】，云台保持原有
                # 姿态(不会被拉到正前方)。接管时把指令初始化到云台当前实际姿态(无跳变)。
                if (target_valid and not self._servo_engaged and
                        now >= self._gimbal_claim_retry_after):
                    # 有有效框时先请求主控；#282 会唤起 gimbal_manager/设备握手，随后
                    # 等待 #285 当前姿态反馈，再以无跳变方式开始发送跟踪指令。
                    control_ready = (not self.stabilize or self._gimbal_control_claimed)
                    if self.stabilize and not self._gimbal_control_claimed:
                        control_ready = self._claim_gimbal_control()
                        if not control_ready:
                            self._gimbal_claim_retry_after = time.time() + 2.0
                    if control_ready:
                        self._engage_servo()
                elif not target_valid and self._servo_engaged:
                    self._disengage_servo()
                elif (not target_valid and self.stabilize and
                      self._gimbal_control_claimed):
                    self._release_gimbal_control()

                # command 1001 的 ACK 等待可能耗时；接管返回后重新检查框是否仍新鲜，
                # 避免对已经超时的旧框再发送一帧控制指令。
                with self.lock:
                    target = self.target
                    age = time.time() - self.last_seen
                target_valid = (target is not None) and (age < self.timeout)
                if not target_valid:
                    if self._servo_engaged:
                        self._disengage_servo()
                    elif self.stabilize and self._gimbal_control_claimed:
                        self._release_gimbal_control()

                geo_roi_active = False
                if self._servo_engaged:
                    # 先补偿机头转向，再按当前位置指向地图目标，最后由视觉误差微调。
                    # 顺序很重要：地理目标是机体相对角，不能再被航向前馈扣一次。
                    self._apply_heading_feedforward()
                    geo_roi_active = self._apply_geo_roi(now, dt)

                if target_valid:
                    u, v = target
                    raw_u, raw_v = u, v  # 检测器直出原始像素(供调试打印)

                    # 运动预测，补偿链路延迟
                    if self.use_prediction:
                        pred = self.predictor.predict(now, self.predict_horizon)
                        if pred is not None:
                            u, v = pred

                    # 预测外推会把像素推出画面(实测 u 越界到 1525/-32), 喂出超大误差
                    # 加剧振荡。钳位到图像边界内, 保证误差有界 du∈[-cx,cx] dv∈[-cy,cy]。
                    u = self.clamp(u, 0.0, float(self.width))
                    v = self.clamp(v, 0.0, float(self.height))

                    # 低通滤波
                    u = self.filt_u.update(u)
                    v = self.filt_v.update(v)

                    # 像素误差（相对图像中心）
                    du = u - self.cx
                    dv = v - self.cy

                    # 角度误差（每帧都算，供微分项取差分；死区只门控比例指令）
                    ang_yaw, ang_pitch = self.pixel_to_angle(du, dv)

                    # 微分(阻尼)项：滤波后角度误差的变化率。用滤波值差分抑制检测噪声放大。
                    # 首帧/重捕获无历史时取 0。ddt 异常(<=0)时取 0 防除零/尖峰。
                    d_ang_yaw = 0.0
                    d_ang_pitch = 0.0
                    if self._prev_ang_t is not None:
                        ddt = now - self._prev_ang_t
                        if ddt > 1e-6:
                            d_ang_yaw = (ang_yaw - self._prev_ang_yaw) / ddt
                            d_ang_pitch = (ang_pitch - self._prev_ang_pitch) / ddt
                    self._prev_ang_yaw = ang_yaw
                    self._prev_ang_pitch = ang_pitch
                    self._prev_ang_t = now

                    # 目标像素速度前馈：滤波后位置差分→速度低通→死区→限幅(° /s)。
                    # 见 __init__ 注释(三重噪声防护)。每帧执行，死区外与反馈叠加、
                    # 死区内单独驱动云台跟随目标移动(消除"停顿")。
                    if self._ff_t is not None:
                        fdt = now - self._ff_t
                        if fdt > 1e-6:
                            inst_vu = (u - self._ff_pu) / fdt
                            inst_vv = (v - self._ff_pv) / fdt
                            # 云台自转去耦：画面速度 = 目标世界运动 - 云台世界转动。
                            # 不加区分时静态目标画面速度全来自云台自转, ff 把它当目标
                            # 运动补偿回去 → 0.45s 执行延迟 + 0.11s 低通在振荡频点
                            # (周期≈2.1s)相位滞后≈92° → 阻尼变正反馈 → 静态极限环
                            # (测试16: ±40px@2.1s; 测试17 回退 kp 后衰减到 ±13px 仍可见)。
                            # 加上 #285 反馈世界角速度后 ff 只含目标真实运动: 静态目标
                            # ff→0(环路断开), 绕圈稳态 ff=目标速度(不再由 P 独扛滞后)。
                            # yaw 加号(#285 yaw 与画面 u 同向); pitch 减号(#285 pitch
                            # 抬头为正, 画面 v 向下为正, 符号相反)。反馈中断>1s 补偿
                            # 失效, 退化为旧行为(不更糟)。
                            if self._fb_rate_t is not None and (now - self._fb_rate_t) < 1.0:
                                inst_vu = inst_vu + self._fb_yaw_rate * self.fx / 57.2958
                                inst_vv = inst_vv - self._fb_pitch_rate * self.fy / 57.2958
                            # 钳位防检测器单帧跳变(目标瞬移)产生速度尖峰
                            inst_vu = self.clamp(inst_vu, -2000.0, 2000.0)
                            inst_vv = self.clamp(inst_vv, -2000.0, 2000.0)
                            self._ff_vu += self.ff_alpha * (inst_vu - self._ff_vu)
                            self._ff_vv += self.ff_alpha * (inst_vv - self._ff_vv)
                    self._ff_t = now
                    self._ff_pu = u
                    self._ff_pv = v

                    # 速度死区：静止/微动目标(检测噪声级)前馈=0，云台完全不动
                    vu_ff = self._ff_vu if abs(self._ff_vu) > self.ff_dz_px else 0.0
                    vv_ff = self._ff_vv if abs(self._ff_vv) > self.ff_dz_px else 0.0
                    # px/s → °/s（fx 像素/弧度）。符号与 P 项一致：目标右移(du>0)
                    # → 云台右转(yaw_cmd 增)；目标下移(dv>0) → 云台低头(pitch_cmd 增)。
                    ff_yaw_rate = self.clamp(
                        vu_ff / self.fx * 57.2958, -self.ff_max_yaw, self.ff_max_yaw)
                    ff_pitch_rate = self.clamp(
                        vv_ff / self.fy * 57.2958, -self.ff_max_pitch, self.ff_max_pitch)
                    ff_yaw_step = ff_yaw_rate * dt
                    ff_pitch_step = ff_pitch_rate * dt
                    if geo_roi_active:
                        # 几何前馈已包含飞机平移造成的方位变化，像素速度再前馈会双算。
                        ff_yaw_step = 0.0
                        ff_pitch_step = 0.0
                        self._i_yaw = 0.0
                        self._i_pitch = 0.0
                    if self.invert_yaw:
                        ff_yaw_step = -ff_yaw_step
                    if self.invert_pitch:
                        ff_pitch_step = -ff_pitch_step

                    # 死区外才修正；死区内保持当前指令（目标已居中）。接管前(尚未
                    # 下发指令)绝不累积 yaw_cmd/pitch_cmd：否则指令被拉到任意值, 首帧
                    # 下发即跳变(实测测试8: 接管前主循环被抢占 0.35s, 累积偏差更大)。
                    if self._servo_engaged and (abs(du) >= self.deadzone_px
                                                or abs(dv) >= self.deadzone_px):
                        # 比例 + 微分(PD)：比例每帧消除一部分误差使闭环收敛到中心；
                        # 微分(阻尼)抵消传输延迟引起的过冲/极限环(近距快速方位变化)。
                        # 增益时间归一化(×dt×loop_hz)：kp 仍是每帧标称增益(30Hz 时
                        # dt_gain≈1.0 行为不变)，但循环被抢占变慢时(实测 tracking 下
                        # 整机 CPU 饱和, 循环从 30Hz 跌到 1-2Hz)云台角速度不再随之同比
                        # 下跌——循环率无关的连续时间控制。dt 已钳位到 (0,1]，dt_gain
                        # 封顶 30 防病态长停顿后单步跳变(饥饿 1Hz 时仍保留完整比例增益)。
                        dt_gain = self.clamp(dt * self.loop_hz, 0.0, 30.0)
                        ang_yaw_deg = math.degrees(ang_yaw)
                        ang_pitch_deg = math.degrees(ang_pitch)

                        # 软启动增益(anti-overshoot, 见 __init__ 注释)：接管初期误差大,
                        # 全速 P+I 猛转 + 链路延迟 → 冲过中心来回摆。增益 70%→100% 线性
                        # 恢复, 初始接近速度慢、跨中心动量小。期满(或已释放)恒为 1.0。
                        if self._engage_t is not None:
                            ramp_prog = (now - self._engage_t) / self.engage_ramp_s
                            if ramp_prog >= 1.0:
                                self._gain_scale = 1.0
                                self._engage_t = None
                            else:
                                self._gain_scale = 0.7 + 0.3 * ramp_prog

                        # 条件积分(anti-windup, 见 __init__ 注释的四重保护)：消除飞机
                        # 匀速/转弯运动时的稳态偏差。误差方向反转时清零(过冲/目标过中心
                        # 后旧积分方向必错)；接近限位冻结；超阈值不积分；clamp 封顶。
                        # 软启动期内积累速率同步缩小(残留积分小, 跨中心后推挤弱)。
                        if (not geo_roi_active and abs(ang_yaw_deg) < self.i_error_deg
                                and abs(self.yaw_cmd) < self.yaw_limit - 1.0):
                            self._i_yaw += ang_yaw_deg * dt * self._gain_scale
                        if (not geo_roi_active and abs(ang_pitch_deg) < self.i_error_deg
                                and self.pitch_min + 1.0 < self.pitch_cmd < self.pitch_max - 1.0):
                            self._i_pitch += ang_pitch_deg * dt * self._gain_scale
                        self._i_yaw = self.clamp(self._i_yaw, -self._i_yaw_max, self._i_yaw_max)
                        self._i_pitch = self.clamp(self._i_pitch, -self._i_pitch_max, self._i_pitch_max)
                        # 追击期积分衰减：误差快速减小(d_ang<-3°/s, 目标正被云台追近)
                        # 时, 追击段累积的积分只会造成到位后继续推挤 → 过冲后二次回中
                        # (实测测试14: 静止目标 264px 接管, 误差 -6.8° 进入积分区间后
                        # 1.06s 积 3.6°·s, 到位(-14px)时积分仍推 7°/s, P 仅 1.2°/s →
                        # 目标被推过中心 42px 才拉回)。按帧衰减 0.9(时间常数≈0.32s,
                        # 追击 1s 残留<5%); 绕圈稳态 d_ang≈0 不触发, 方向反转过零由
                        # 下方清零逻辑处理; 检测噪声帧差分 ±2°/s 低于阈值 3°/s 不误触发。
                        if d_ang_yaw < -3.0 and self._i_yaw != 0.0:
                            self._i_yaw *= 0.9
                        if d_ang_pitch < -3.0 and self._i_pitch != 0.0:
                            self._i_pitch *= 0.9

                        # 清零阈值 0.3°(迟滞)：检测微扰(±2px≈0.07°)不会触发清零→指令回摆；
                        # 仅真正的过冲/目标方向反转(|err|>0.3° 且反号)才丢弃旧积分。
                        # 0.3°(而非 1.0°)：目标高速往返/圆周运动时误差每 2-3s 过零一次，
                        # 1.0° 迟滞使反转后云台仍被旧方向积分(实测 8-16°/s)推挤 0.3-0.5s
                        # → 误差冲大再拉回, 即"来回晃动"; 0.3°≈8px 反转后 0.1s 内即释放。
                        if ang_yaw_deg * self._i_yaw < 0.0 and abs(ang_yaw_deg) > 0.3:
                            self._i_yaw = 0.0
                        if ang_pitch_deg * self._i_pitch < 0.0 and abs(ang_pitch_deg) > 0.3:
                            self._i_pitch = 0.0

                        # 比例 + 微分(PD)：比例每帧消除一部分误差使闭环收敛到中心；
                        # 微分(阻尼)抵消传输延迟引起的过冲/极限环(近距快速方位变化)。
                        # 增益时间归一化(×dt×loop_hz)：kp 仍是每帧标称增益(30Hz 时
                        # dt_gain≈1.0 行为不变)，但循环被抢占变慢时(实测 tracking 下
                        # 整机 CPU 饱和, 循环从 30Hz 跌到 1-2Hz)云台角速度不再随之同比
                        # 下跌——循环率无关的连续时间控制。dt 已钳位到 (0,1]，dt_gain
                        # 封顶 30 防病态长停顿后单步跳变(饥饿 1Hz 时仍保留完整比例增益)。
                        yaw_step = ((ang_yaw_deg * self.kp_yaw
                                     + math.degrees(d_ang_yaw) * self.kd_yaw)
                                    * dt_gain) * self._gain_scale
                        pitch_step = ((ang_pitch_deg * self.kp_pitch
                                       + math.degrees(d_ang_pitch) * self.kd_pitch)
                                      * dt_gain) * self._gain_scale

                        # 单帧步长比例限制：每帧最多消掉剩余误差的 50%。根因：tracking 下
                        # CPU 饱和把循环压到 ~5Hz 时，dt_gain 归一化虽保持角速度恒定，
                        # 但每帧步长被放大(≈×5)，配合执行+检测延迟(~0.5s)使 增益×延迟>1
                        # → 过冲自激极限环(实测目标静止时云台 pitch ±4° 来回扫, 像素
                        # 跟着 ±200px 扫动, 误差永不进死区)。0.5 限制保证反馈滞后期间
                        # 云台总转角恒小于当前误差(几何收敛)，任意循环率/延迟下稳定；
                        # 30Hz 正常时步长仅 0.003×err 远小于 0.5×err，完全不触发。
                        # 【只限制比例项】：积分项不能钳(补偿持续漂移必须能持续输出,
                        # 若被 0.5×err 钳死则永远追不上移动目标)。积分安全性由
                        # __init__ 的四重 anti-windup 保证。
                        ratio_y = 0.5 * abs(ang_yaw_deg)
                        ratio_p = 0.5 * abs(ang_pitch_deg)
                        yaw_step = self.clamp(yaw_step, -ratio_y, ratio_y)
                        pitch_step = self.clamp(pitch_step, -ratio_p, ratio_p)

                        # 叠加积分项(连续时间角速度 ki×I 再乘 dt 得本帧步长; 软启动期
                        # 同步缩放, 避免接管初期残留积分全速输出)
                        yaw_step += self.ki_yaw * self._i_yaw * dt * self._gain_scale
                        pitch_step += self.ki_pitch * self._i_pitch * dt * self._gain_scale

                        if self.invert_yaw:
                            yaw_step = -yaw_step
                        if self.invert_pitch:
                            pitch_step = -pitch_step

                        # 指令变化率限制(物理上限)，作用于叠加后的总指令
                        max_dy = self.max_slew_yaw * dt
                        max_dp = self.max_slew_pitch * dt
                        yaw_step = self.clamp(yaw_step, -max_dy, max_dy)
                        pitch_step = self.clamp(pitch_step, -max_dp, max_dp)

                        # 指令步长一阶低通：平滑死区边缘反复触发与检测微扰的传导，
                        # 消除指令微摆(见 __init__ step_alpha 注释)。前馈不过此低通。
                        if self.step_alpha > 0.0:
                            self._y_step_f = (self.step_alpha * yaw_step
                                              + (1.0 - self.step_alpha) * self._y_step_f)
                            self._p_step_f = (self.step_alpha * pitch_step
                                              + (1.0 - self.step_alpha) * self._p_step_f)
                            yaw_step = self._y_step_f
                            pitch_step = self._p_step_f

                        # 前馈误差门控：大误差瞬态(接管/目标被甩)云台以反馈速率猛转时,
                        # 目标画面速度≈云台自身转速(目标自身慢), 前馈把"云台转动"误当
                        # "目标运动"输出反方向角速度 → 与反馈对抗(实测测试15: 接管 242px
                        # 段收敛 2.1s + ff 低通残留到位后推挤过冲 21px 再回调)。|err|≥off
                        # 前馈权重=0 让反馈全力收敛, ≤full 全开(稳态跟踪误差远小于 full,
                        # 绕圈实测 ±35~50px, 正常跟踪不受影响), 之间线性过渡。
                        err_px = math.hypot(du, dv)
                        if err_px >= self.ff_gate_off_px:
                            ff_gate = 0.0
                        elif err_px <= self.ff_gate_full_px:
                            ff_gate = 1.0
                        else:
                            ff_gate = ((self.ff_gate_off_px - err_px)
                                       / (self.ff_gate_off_px - self.ff_gate_full_px))

                        # 叠加目标速度前馈(不过 step 低通, 其本身已平滑)。前馈让云台
                        # 直接以目标画面速度转动, 反馈只补误差残差 → 速度连续不滞后,
                        # 目标变速/反转时无需等误差积累(消除"一顿一顿")。
                        yaw_step += ff_yaw_step * ff_gate
                        pitch_step += ff_pitch_step * ff_gate

                        # 累积到绝对指令并限位
                        self.yaw_cmd = self.wrap_180(self.yaw_cmd + yaw_step)
                        self.yaw_cmd = self.clamp(
                            self.yaw_cmd, -self.yaw_limit, self.yaw_limit)
                        self.pitch_cmd = self.clamp(
                            self.pitch_cmd + pitch_step, self.pitch_min, self.pitch_max)

                        # 周期性打印（避免刷屏）。同时给出原始检测像素与处理后像素，
                        # 便于排查俯仰方向/饱和（raw=检测器直出, proc=预测+钳位+滤波后）。
                        if now - last_log > 0.2:
                            rospy.loginfo(
                                "[GVS] raw(%.0f,%.0f) proc(%.0f,%.0f) 误差(%.0f,%.0f)px "
                                "-> 指令 yaw=%.1f° pitch=%.1f°",
                                raw_u, raw_v, u, v, du, dv,
                                self.yaw_cmd, self.pitch_cmd)
                            last_log = now
                    elif self._servo_engaged and (abs(ff_yaw_step) > 1e-9
                                                  or abs(ff_pitch_step) > 1e-9):
                        # 死区内且目标仍在移动：反馈冻结(目标已居中, 不修正)，但前馈
                        # 继续驱动云台跟随目标 → 目标慢移时云台连续转动而非走走停停。
                        self.yaw_cmd = self.wrap_180(self.yaw_cmd + ff_yaw_step)
                        self.yaw_cmd = self.clamp(
                            self.yaw_cmd, -self.yaw_limit, self.yaw_limit)
                        self.pitch_cmd = self.clamp(
                            self.pitch_cmd + ff_pitch_step, self.pitch_min, self.pitch_max)
                else:
                    # 目标丢失：保持当前指向，复位滤波器/预测器避免下次跳变
                    self.filt_u.reset()
                    self.filt_v.reset()
                    self.predictor.reset()
                    # 复位微分项历史，避免重捕获时用陈旧前值算出巨大 d误差/dt 尖峰
                    self._prev_ang_yaw = None
                    self._prev_ang_pitch = None
                    self._prev_ang_t = None
                    # 丢失期间不积分(漂移补偿失去意义), 重捕获从零重新积累
                    self._i_yaw = 0.0
                    self._i_pitch = 0.0
                    # 步长低通状态清零, 重捕获首帧不用陈旧平滑值
                    self._y_step_f = 0.0
                    self._p_step_f = 0.0
                    # 目标速度前馈状态清零, 重捕获首帧不用陈旧速度(防前馈尖峰)
                    self._ff_vu = 0.0
                    self._ff_vv = 0.0
                    self._ff_t = None
                    self._ff_pu = None
                    self._ff_pv = None

                # 仅在有效目标仍新鲜时下发姿态；失框超时后停止发指令并释放主控权，
                # 让手动控制端可以接管。
                if self._servo_engaged and target_valid:
                    self.send_command(self.pitch_cmd, self.yaw_cmd)
                rate.sleep()
        finally:
            # 节点退出时也归还主控权，避免异常退出后 PX4 仍认为本节点是 primary。
            if self._servo_engaged or self._gimbal_control_claimed:
                self._disengage_servo()

    # ================= 发布云台指令 =================
    def send_command(self, pitch, yaw, roll=0.0):
        """发送云台目标姿态。

        pitch/yaw/roll 约定（与 MountControl 一致）：
          yaw  : 机体相对, 0=机头, 正=向右
          pitch: 相对地平线, 正=向上
          roll : 相对地平线, 正=右滚
        根据 self.stabilize 选择发送后端。
        """
        if self.stabilize:
            self._send_stabilized(pitch, yaw, roll)
        else:
            cmd = MountControl()
            cmd.header.stamp = rospy.Time.now()
            cmd.mode = 2          # MAVLINK_TARGETING（绝对角度）
            cmd.pitch = pitch     # pitch+ 向上（相对地平线）
            cmd.roll = roll       # roll+ 右滚（相对地平线）
            cmd.yaw = yaw         # yaw+ 向右（机体相对）
            self.pub.publish(cmd)

    # ================= 稳像后端：GIMBAL_MANAGER_SET_ATTITUDE(#282) =================
    def _send_stabilized(self, pitch, yaw, roll=0.0):
        """发 GIMBAL_MANAGER_SET_ATTITUDE(#282)，带 YAW_LOCK，使相机固定在世界方向(稳像)。

        命令经 PX4 gimbal_manager 下发：manager 收到 #282 后统一产生唯一的
        GIMBAL_DEVICE_SET_ATTITUDE(#284) 到设备。robot.cpp UpdateCameraPose 仅在
        yaw_lock 时用世界系四元数(拒绝飞机扰动)，故这里置 YAW_LOCK(连同
        ROLL_LOCK|PITCH_LOCK)。经 manager 而非直发裸 #284 的原因：MNT_MODE_IN=4 下
        gimbal_manager 是设备的唯一权威命令源，直发裸 #284 会与 manager(及 QGC ROI)
        两路抢占设备、交替覆盖目标姿态 → 抖动；且发 #282 会唤起 manager 完成设备
        握手，首次启动不再需要 QGC 点 ROI。
        世界 yaw = 罗盘航向 + 机体相对 yaw；pitch 按 test4.py 实测约定取反。
        """
        if self.mavlink_pub is None:
            return

        if not self.compass_received:
            # 尚无航向时无法构造世界系，先用 MountControl 兜底（至少能控制）
            cmd = MountControl()
            cmd.header.stamp = rospy.Time.now()
            cmd.mode = 2
            cmd.pitch = pitch
            cmd.roll = roll
            cmd.yaw = yaw
            self.pub.publish(cmd)
            return

        with self._compass_lock:
            heading = self.compass_hdg

        # 世界 yaw（度）：罗盘航向(0=北,顺时针) + 机体相对 yaw + 偏置修正
        world_yaw_deg = self.wrap_180(heading + yaw + self.yaw_offset_deg)
        # 记录本次实际下发的世界指向，供 simulate 投影使用(见 sim_target_position)。
        self._last_world_yaw = world_yaw_deg
        # 诊断：观察航向前馈是否保持 world_yaw 恒定（飞机转弯时 world_yaw 不应跟转）
        if not hasattr(self, "_stab_dbg_cnt"):
            self._stab_dbg_cnt = 0
        self._stab_dbg_cnt += 1
        if self._stab_dbg_cnt % 30 == 1:
            rospy.loginfo("[GVS_DBG] heading=%.1f yaw_cmd=%.1f world_yaw=%.1f",
                          heading, yaw, world_yaw_deg)
        # pitch 取反（test4.py 实测：command_pitch = -asin(world_q)）
        pitch_for_quat = -pitch if self.stabilize_pitch_negate else pitch

        q = euler_to_quat_zyx(
            0.0, math.radians(pitch_for_quat), math.radians(world_yaw_deg))

        flags = (GIMBAL_MANAGER_FLAGS_ROLL_LOCK |
                 GIMBAL_MANAGER_FLAGS_PITCH_LOCK |
                 GIMBAL_MANAGER_FLAGS_YAW_LOCK)

        ros_msg = self._pack_mavlink_msg(flags, q)
        if ros_msg is not None and self.mavlink_pub is not None:
            self.mavlink_pub.publish(ros_msg)
            self._n282_mavros += 1
            # 诊断：证明发给 /mavlink/to 的 #282 源身份(应 255/190)、目标(1/154)、flags
            rospy.loginfo_throttle(
                5.0,
                "[GVS_LINK] #282→主链路(%s/mavlink/to) 源sys/comp=%d/%d 目标=%d/%d flags=%d "
                "world_yaw=%.1f",
                self.vehicle_ns, self.src_sysid, self.src_compid,
                self.gimbal_sysid, self.manager_compid, flags, world_yaw_deg)

        # 周期性链路汇总(~2s)：对比 #282 发出量与 #285 反馈量，
        # 并打印最近一帧 #285 的源身份，判断反馈是否真的回来
        now = time.time()
        if now - self._link_log_t >= 2.0:
            self._link_log_t = now
            rospy.loginfo(
                "[GVS_LINK] 汇总: #282(mavros)=%d | cmd1001=%d(抢占/释放) | #285收到=%d 最近#285源=%s",
                self._n282_mavros, self._n530, self._n285, self._last_fb_src)

    def _pack_mavlink_msg(self, flags, q):
        """用 pymavlink 编码 GIMBAL_MANAGER_SET_ATTITUDE(#282) 并转成
        mavros_msgs/Mavlink。

        mavros convert() 直接拷贝 magic/len/.../msgid/checksum/payload64 字段
        （不重算 CRC），因此 checksum 必须用 pymavlink pack() 算好的正确值。
        """
        try:
            # GIMBAL_MANAGER_SET_ATTITUDE(#282) 字段: target_system, target_component,
            # flags, gimbal_device_id, q[4], angular_velocity_x/y/z（time_boot_ms 自动填）
            mavmsg = self._mav.gimbal_manager_set_attitude_encode(
                self.gimbal_sysid, self.manager_compid, flags,
                self.gimbal_device_id,
                q,                       # [w, x, y, z]
                0.0, 0.0, 0.0)           # 角速率不用，显式置 0 避免 PX4 误判
            buf = mavmsg.pack(self._mav)  # 完整 v2 帧: 头(10)+payload(33)+CRC(2)
        except Exception as e:
            rospy.logwarn("[GVS] 编码 GIMBAL_MANAGER_SET_ATTITUDE 失败: %s", e)
            return None
        return self._buf_to_mavlink_msg(buf)

    def _buf_to_mavlink_msg(self, buf):
        """把 pymavlink pack() 出的完整 MAVLink v2 帧字节，按 mavros
        mavlink_convert.h 的字段映射 reconstruct 成 mavros_msgs/Mavlink。
        供 #282 / command 1001 等所有经本机 /mavlink/to 发送的消息复用。"""
        if len(buf) < 12 or buf[0] != 253:  # 必须是 MAVLink v2
            rospy.logwarn("[GVS] 非预期的 MAVLink 帧(magic=%s)",
                          buf[0] if buf else "empty")
            return None

        plen = buf[1]
        payload = buf[10:10 + plen]
        checksum = struct.unpack('<H', buf[10 + plen:12 + plen])[0]

        # payload 补齐到 8 字节边界再转 uint64 数组
        pad = (8 - len(payload) % 8) % 8
        payload += b'\0' * pad

        msg = Mavlink()
        msg.header.stamp = rospy.Time.now()
        msg.framing_status = Mavlink.FRAMING_OK
        msg.magic = buf[0]
        msg.len = plen
        msg.incompat_flags = buf[2]
        msg.compat_flags = buf[3]
        msg.seq = buf[4]
        msg.sysid = buf[5]
        msg.compid = buf[6]
        msg.msgid = buf[7] | (buf[8] << 8) | (buf[9] << 16)
        msg.checksum = checksum
        msg.payload64 = list(
            struct.unpack('<%dQ' % (len(payload) // 8), payload))
        return msg

    # ================= 抢占云台主控权 =================
    def _claim_gimbal_control(self, repeats=5):
        """把云台主控权(primary control)设成本节点身份(src_sysid/src_compid)。

        PX4 gimbal v2 只接受 origin == primary 的 #282(input_mavlink.cpp
        _process_set_attitude)。primary 初值 0/0，必须先抢占，否则 #282 全被
        "Ignoring"、云台纹丝不动。

        抢占按优先级二级：
        A. mavros command 服务发 COMMAND_LONG(command=1001, DO_GIMBAL_MANAGER_CONFIGURE)
           并等 COMMAND_ACK。这是实际使用场景下唯一应当依赖的方式：由地面站/机载
           计算机通过 mavros 把云台主控切到本节点身份。
        B. 非 mavros 兜底（仅在显式开启时使用）。默认关闭，避免在真机/实际
           飞控上依赖 PX4 shell 或原始 MAVLink 直发。
        """
        # --- 方式 A: mavros command 服务 (实际使用场景首选) ---
        if self._claim_via_mavros_cmd(retries=3):
            self._gimbal_control_claimed = True
            self._gimbal_claim_retry_after = 0.0
            return True

        if self._allow_non_mavros_claim:
            # 仅在调试/特殊环境下显式启用，默认关闭。
            if self._use_tmux_claim and self._claim_via_tmux():
                self._gimbal_control_claimed = True
                self._gimbal_claim_retry_after = 0.0
                return True
            if self._claim_via_raw_530(repeats=repeats):
                self._gimbal_control_claimed = True
                self._gimbal_claim_retry_after = 0.0
                return True

        rospy.logwarn(
            "[GVS] mavros command 1001 抢占失败，且未启用非 mavros 兜底；本节点不会再使用 "
            "PX4 shell/raw MAVLink 方式切换云台主控")
        return False

    def _release_gimbal_control(self, repeats=3):
        """释放本节点持有的 gimbal_manager primary 控制权。"""
        if not self._gimbal_control_claimed:
            return True
        if time.time() < self._gimbal_release_retry_after:
            return False

        params = _build_gimbal_release_params()
        released = self._claim_via_mavros_cmd(
            retries=3, params=params, action="释放主控")
        if not released and self._use_tmux_claim and self._allow_non_mavros_claim:
            released = self._claim_via_tmux(release=True)
        if not released:
            # 释放命令只能撤销当前发送方持有的 primary，不会接管他人控制权；
            # 即使关闭了非 mavros 抢占兜底，也通过已有 MAVLink 通道尽力归还主控。
            released = self._claim_via_raw_530(
                repeats=repeats, params=params, action="释放主控")

        if released:
            self._gimbal_control_claimed = False
            self._gimbal_release_retry_after = 0.0
            rospy.loginfo("[GVS] 已释放 gimbal_manager 主控权")
        else:
            self._gimbal_release_retry_after = time.time() + 2.0
            rospy.logwarn(
                "[GVS] 释放主控权命令未确认；已停止发送姿态指令，请检查 mavros command 服务")
        return released

    def _claim_via_mavros_cmd(self, retries=3, params=None, action="抢占主控"):
        """经 mavros /uav0/mavros/cmd/command 服务发 COMMAND_LONG(command=1001) 并等 COMMAND_ACK。

        服务内部把 COMMAND_LONG 发给飞控并等待匹配的 COMMAND_ACK；成功表示
        gimbal_manager 已处理 command 1001。失败(ACK 非 ACCEPTED 或超时)则重试；
        retries 耗尽返回 False 转兜底。"""
        if self._cmd_client is None:
            return False
        try:
            self._cmd_client.wait_for_service(timeout=2.0)
        except Exception:
            rospy.logwarn(
                "[GVS] mavros cmd 服务(%s/cmd/command)不可用, 跳过%s",
                self.mavros_ns, action)
            return False
        for i in range(max(1, retries)):
            try:
                command_params = (params if params is not None else
                                  _build_gimbal_claim_params(
                                      self.src_sysid, self.src_compid))
                resp = self._cmd_client(
                    broadcast=False, command=MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE,
                    confirmation=0, param1=command_params[0], param2=command_params[1],
                    param3=command_params[2], param4=command_params[3],
                    param5=command_params[4], param6=command_params[5],
                    param7=command_params[6])
                self._n530 += 1
                if resp.success:
                    rospy.loginfo(
                        "[GVS] 经 mavros cmd 完成%s (command 1001 ACK result=%d)",
                        action, resp.result)
                    return True
                rospy.logwarn(
                    "[GVS] command 1001 %s ACK 非 ACCEPTED (result=%d), 重试 %d/%d",
                    action, resp.result, i + 1, retries)
            except rospy.ServiceException as e:
                rospy.logwarn(
                    "[GVS] command 1001 %s服务调用失败(%s), 重试 %d/%d",
                    action, e, i + 1, retries)
            rospy.sleep(0.2)
        rospy.logwarn("[GVS] mavros cmd command 1001 %s未成功(%d 次), 尝试兜底",
                      action, retries)
        return False

    def _claim_via_tmux(self, release=False):
        """SITL 开发兜底：经 tmux send-keys 发 PX4 控制台命令 `gimbal primary-control`。

        直接改 gimbal 线程内部状态(gimbal.cpp primary-control 分支)，绕开
        vehicle_command uORB，100% 可靠；但真机够不到 PX4 shell，send-keys 会失败
        并返回 False。"""
        tmux_target = str(rospy.get_param("~px4_tmux_target", "simulation:2.0"))
        tmux_socket = str(rospy.get_param("~px4_tmux_socket", "sim"))
        if not tmux_target:
            return False
        if release:
            cmd_str = "gimbal primary-control 0 0"
        else:
            cmd_str = "gimbal primary-control %d %d" % (self.src_sysid, self.src_compid)
        try:
            subprocess.check_call(
                ["tmux", "-L", tmux_socket, "send-keys", "-t", tmux_target,
                 cmd_str, "Enter"],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=5)
            rospy.loginfo(
                "[GVS] 经 PX4 控制台更新主控(SITL兜底): %s (tmux -L %s → %s)",
                cmd_str, tmux_socket, tmux_target)
            rospy.sleep(0.5)  # 等 PX4 处理
            return True
        except Exception as e:
            rospy.logwarn("[GVS] tmux 抢占失败(%s)", e)
            return False

    def _claim_via_raw_530(self, repeats=5, params=None, action="抢占主控"):
        """兜底：编码 COMMAND_LONG(command=1001) 经 /mavlink/to 发出(无 ACK 确认)。"""
        if self.mavlink_pub is None:
            return False
        command_params = (params if params is not None else
                          _build_gimbal_claim_params(self.src_sysid, self.src_compid))
        for _ in range(max(1, repeats)):
            try:
                mavmsg = self._mav.command_long_encode(
                    self.gimbal_sysid, self.manager_compid,
                    MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE, 0,
                    command_params[0], command_params[1],
                    command_params[2], command_params[3], command_params[4],
                    command_params[5], command_params[6])
                buf = mavmsg.pack(self._mav)
            except Exception as e:
                rospy.logwarn("[GVS] 编码 command 1001 失败: %s", e)
                return False
            ros_msg = self._buf_to_mavlink_msg(buf)
            if ros_msg is not None and self.mavlink_pub is not None:
                self.mavlink_pub.publish(ros_msg)
            else:
                return False
            self._n530 += 1
            rospy.sleep(0.1)
        rospy.loginfo(
            "[GVS_LINK] 已发 raw command 1001 x%d %s(兜底): 源sys/comp=%d/%d → 目标=%d/%d",
            repeats, action, self.src_sysid, self.src_compid,
            self.gimbal_sysid, self.manager_compid)
        return True

    # ================= 模拟运动目标模式 =================
    def sim_target_position(self, t):
        """闭环模拟目标：返回 t 时刻应注入的目标像素坐标 (u, v)。

        关键设计——模拟目标必须【随云台指向变化】(闭环)。若目标像素只是时间
        函数(开环)，云台无论怎么转都无法把它“居中”，IBVS 会持续积分、指令撞
        pitch/yaw 限位、云台被命令满速旋转 → 画面抖动(旧实现的 bug)。

        这里改用一个缓慢运动的【虚拟目标方向】(角度, 机体/指令系, 幅度
        sim_amplitude_deg)，按 “目标方向 - 当前云台指向” 投影成像素偏移：
        云台越对准目标，目标越靠近图像中心，误差收敛 → 平滑且有界的连续跟踪，
        与真实检测器闭环的行为一致。当前云台指向用本节点指令 yaw_cmd/pitch_cmd
        近似（仿真器云台经 slerp 紧密跟随指令，慢轨迹下滞后极小）。
        """
        w = 2.0 * math.pi / max(self.sim_period, 1e-3)  # 角频率 rad/s
        a = self.sim_amplitude_deg                       # 虚拟目标角幅度(度)

        # 虚拟目标方向(度, 指令系)：连续平滑的小幅轨迹
        if self.sim_pattern == "sweep":
            ty, tp = a * math.sin(w * t), 0.0
        elif self.sim_pattern == "lissajous":
            ty, tp = a * math.sin(2.0 * w * t), a * math.sin(w * t)
        elif self.sim_pattern == "figure8":
            ty, tp = a * math.sin(w * t), a * 0.6 * math.sin(2.0 * w * t)
        elif self.sim_pattern == "rect":
            # 矩形周长 + 两条对角线：分段线性循环。路径先画矩形四边回到起点，
            # 再画两条对角线(并经两条竖边闭合)。yaw 半幅=a, pitch 半幅=0.6a
            # (留俯仰余量)。拐点在此不平滑，由 IBVS slew 限速与仿真器指数平滑
            # 自然 rounded——用于检验云台在水平/竖直/对角各方向的跟踪是否正常。
            ay, ap = a, a * 0.6
            wps = ((-1, -1), (1, -1), (1, 1), (-1, 1), (-1, -1),
                   (1, 1), (1, -1), (-1, 1), (-1, -1))
            n_seg = len(wps) - 1
            phase = (t / max(self.sim_period, 1e-3)) % 1.0
            sf = phase * n_seg
            i = int(sf)
            if i >= n_seg:
                i = n_seg - 1
            fr = sf - i
            y0, p0 = wps[i]
            y1, p1 = wps[i + 1]
            ty = ay * (y0 + (y1 - y0) * fr)
            tp = ap * (p0 + (p1 - p0) * fr)
        else:  # circle（默认）
            ty, tp = a * math.cos(w * t), a * math.sin(w * t)

        # 当前云台指向(度)。
        # 稳像模式下必须用【世界系指向】(航向 + 机体yaw + 偏置)而非机体相对
        # yaw_cmd 来投影虚拟目标。原因：航向前馈(_apply_heading_feedforward)会
        # 持续修改 yaw_cmd 抵消航向变化以保持世界指向恒定；若目标像素仍按 yaw_cmd
        # 投影(du=(ty-yaw_cmd)*fx)，前馈一改 yaw_cmd 目标像素就反向偏移，IBVS
        # 立刻把 yaw_cmd 拉回 → 前馈与 IBVS 互相抵消，飞机转向时世界指向仍跟着
        # 航向转(实测 world_yaw 跟随 heading ~90%, 误差撞 ±80px 钳位)。改用世界系
        # 指向投影、并把虚拟目标定义在世界系(相对锚点)后，前馈保持世界指向恒定 →
        # 目标始终接近图像中心 → IBVS 不再对抗前馈 → 真正的 yaw 稳像。
        if self.stabilize and self.compass_received:
            # 用最近一次实际下发的世界 yaw(在 control_loop 航向前馈之后原子算出)。
            # 不能在本生成线程里用 heading+yaw_cmd 重算：快速转向时罗盘回调已更新
            # heading 但前馈尚未把 yaw_cmd 减掉该增量，重算会读到 pointing=世界yaw+Δ航向
            # 的中间态 → du 尖峰(实测转弯时 -40px 级) → IBVS 抽动 → 画面抖动。
            pointing_yaw = self._last_world_yaw
            if pointing_yaw is None:
                with self._compass_lock:
                    hdg = self.compass_hdg
                pointing_yaw = self.wrap_180(hdg + self.yaw_cmd + self.yaw_offset_deg)
            if self._sim_world_anchor is None:
                self._sim_world_anchor = pointing_yaw
            # 虚拟目标定义在世界系(相对锚点)，与世界系指向同参考系 → 误差解耦航向
            ty_world = self._sim_world_anchor + ty
            du = math.radians(self.wrap_180(ty_world - pointing_yaw)) * self.fx
        else:
            du = math.radians(ty - self.yaw_cmd) * self.fx
        # 相机【实际世界俯仰】(相对地平线)。真实发送/反馈路径都带 stabilize_pitch_negate：
        # pitch_cmd = -(相机世界俯仰)(见 _send_stabilized / _engage_servo)。simulate
        # 投影必须用同一个量, 否则俯仰 plant 符号与真实相反, tracking 与 simulate 会需要
        # 相反的 invert_pitch(实测 tracking 俯仰反向、撞 pitch_max 饱和)。统一后一个
        # invert_pitch 默认值即可同时正确。
        if self.stabilize and self.stabilize_pitch_negate:
            pitch_now = -self.pitch_cmd
        else:
            pitch_now = self.pitch_cmd
        # 图像 v 向下，而俯仰+ 向上，故俯仰差投影到 v 时取反。
        dv = -math.radians(tp - pitch_now) * self.fy

        # 钳位：防止 slew 限速滞后被投影放大成巨大像素误差 → IBVS 正反馈振荡。
        # ±80px 对应角修正 Kp*(80/fx*57.3)≈1.2°/帧≈36°/s，在 slew(60°/s)内。
        max_px = 80.0
        du = max(-max_px, min(max_px, du))
        dv = max(-max_px, min(max_px, dv))
        return self.cx + du, self.cy + dv

    def simulate_loop(self):
        """模拟运动目标，测试自动连续跟踪。

        后台线程按 sim_hz 生成运动目标像素并注入控制链路；主线程运行与
        tracking 模式相同的 control_loop（滤波/IBVS/限速都生效），但【运动
        预测被强制关闭】——原因见下方说明。
        """
        # 闭环模拟目标必须关闭运动预测。真实目标的像素运动有独立惯性，匀速
        # 预测器(TargetPredictor)外推其速度可补偿链路延迟。但闭环模拟目标的
        # 像素是 (目标方向 - yaw_cmd) 的从动函数，本身没有独立速度：yaw_cmd
        # 一变它立刻反向。预测器却按上一帧速度继续外推，把小幅跟踪滞后放大成
        # 数万像素的虚假误差(实测 circle ±12° 时误差冲到 9e4 px)，IBVS 去追
        # 这个假误差 → yaw_cmd 发散振荡、画面剧烈抖动。故 simulate 下强制关闭。
        if self.use_prediction:
            rospy.loginfo(
                "[GVS] 闭环模拟目标无独立运动惯性, 关闭运动预测以防指令发散")
            self.use_prediction = False
            self.predictor.reset()

        # 重置世界 yaw 锚点：本次运行首次拿到罗盘航向时重新捕获，
        # 虚拟目标相对它定义在世界系(见 sim_target_position)。
        self._sim_world_anchor = None

        sep = "=" * 58
        rospy.loginfo(sep)
        rospy.loginfo("[GVS] 模拟目标模式(闭环)")
        rospy.loginfo("[GVS] 轨迹=%s, 角幅度=±%.0f°, 周期=%.1fs, 生成频率=%.0fHz",
                      self.sim_pattern, self.sim_amplitude_deg,
                      self.sim_period, self.sim_hz)
        rospy.loginfo("[GVS] 虚拟目标闭环投影, 云台应在 ±%.0f° 内连续平滑跟踪",
                      self.sim_amplitude_deg)
        rospy.loginfo("[GVS] Ctrl+C 退出")
        rospy.loginfo(sep)

        stop = threading.Event()

        def generator():
            r = rospy.Rate(self.sim_hz)
            t0 = time.time()
            last_log = 0.0
            while not stop.is_set() and not rospy.is_shutdown():
                t = time.time() - t0
                u, v = self.sim_target_position(t)
                self._inject_target(u, v)
                if t - last_log > 1.0:
                    rospy.loginfo("[GVS] 模拟目标: u=%.0f v=%.0f (中心 %.0f,%.0f)",
                                  u, v, self.cx, self.cy)
                    last_log = t
                r.sleep()

        gen_thread = threading.Thread(target=generator, daemon=True)
        gen_thread.start()

        try:
            self.control_loop()
        finally:
            stop.set()
            gen_thread.join(timeout=1.0)

    # ================= 手动测试模式 =================
    def manual_loop(self):
        """手动输入云台目标姿态，验证云台控制链路是否正常。

        姿态约定与跟踪模式完全一致（走同一条 MountControl 发送路径）：
          yaw  : 机体相对, 0=机头方向, 正=向右, 范围 ±yaw_limit
          pitch: 相对地平线, 正=向上, 范围 [pitch_min, pitch_max]
          roll : 相对地平线, 正=右滚（可省略, 默认 0；居中跟踪不需要 roll）
        """
        sep = "=" * 58
        rospy.loginfo(sep)
        rospy.loginfo("[GVS] 手动测试模式")
        rospy.loginfo("[GVS] 输入格式:  yaw pitch [roll]    单位: 度  (绝对姿态)")
        rospy.loginfo("[GVS]   yaw  : 0=机头, 正=向右 (±%.0f)", self.yaw_limit)
        rospy.loginfo("[GVS]   pitch: 正=向上 ([%.0f, %.0f])",
                      self.pitch_min, self.pitch_max)
        rospy.loginfo("[GVS]   roll : 正=右滚 (可省略, 默认0; 居中跟踪不需要)")
        rospy.loginfo("[GVS] 相对增量: rel dyaw dpitch [droll]  在当前指令上累加(小幅微调防跳变)")
        rospy.loginfo("[GVS] 命令:  r = 回中(0,0,0)   q = 退出")
        rospy.loginfo("[GVS] 示例:  30 -20 (绝对) | rel 5 0 (向右微调5°)")
        rospy.loginfo(sep)

        state = {"yaw": 0.0, "pitch": 0.0, "roll": 0.0}
        state_lock = threading.Lock()
        stop = threading.Event()

        # 后台线程持续发布当前姿态, 使云台保持位置(否则可能退出目标控制模式)
        def publisher():
            r = rospy.Rate(self.loop_hz)
            while not stop.is_set() and not rospy.is_shutdown():
                with state_lock:
                    y, p, ro = state["yaw"], state["pitch"], state["roll"]
                self.send_command(p, y, ro)
                r.sleep()

        pub_thread = threading.Thread(target=publisher, daemon=True)
        pub_thread.start()

        try:
            while not rospy.is_shutdown():
                try:
                    line = input("[GVS] > ").strip()
                except EOFError:
                    break
                if not line:
                    continue
                low = line.lower()
                if low == "q":
                    rospy.loginfo("[GVS] 退出手动模式")
                    break
                if low == "r":
                    with state_lock:
                        state.update(yaw=0.0, pitch=0.0, roll=0.0)
                    rospy.loginfo("[GVS] 回中: yaw=0 pitch=0 roll=0")
                    continue

                parts = line.split()
                # 相对增量模式: "rel dyaw dpitch [droll]" 在当前指令上累加
                relative = False
                if parts[0].lower() in ("rel", "d"):
                    relative = True
                    parts = parts[1:]
                try:
                    v0 = float(parts[0])
                    v1 = float(parts[1]) if len(parts) > 1 else 0.0
                    v2 = float(parts[2]) if len(parts) > 2 else 0.0
                except (ValueError, IndexError):
                    rospy.logwarn(
                        "[GVS] 格式错误: 绝对 'yaw pitch [roll]' 或 相对 'rel dyaw dpitch [droll]'")
                    continue

                with state_lock:
                    if relative:
                        yaw = state["yaw"] + v0
                        pitch = state["pitch"] + v1
                        roll = state["roll"] + v2
                    else:
                        yaw, pitch, roll = v0, v1, v2
                    yaw = self.clamp(self.wrap_180(yaw), -self.yaw_limit, self.yaw_limit)
                    pitch = self.clamp(pitch, self.pitch_min, self.pitch_max)
                    roll = self.clamp(roll, -180.0, 180.0)
                    state.update(yaw=yaw, pitch=pitch, roll=roll)
                tag = "相对-> " if relative else "绝对-> "
                rospy.loginfo("[GVS] %syaw=%.1f° pitch=%.1f° roll=%.1f°",
                              tag, yaw, pitch, roll)
        finally:
            stop.set()
            pub_thread.join(timeout=1.0)

    def run(self):
        if self.mode == "manual":
            self.manual_loop()
        elif self.mode in ("simulate", "sim"):
            self.simulate_loop()
        else:
            rospy.loginfo("[GVS] 等待目标检测 (%s, type=%s) ...",
                          self.detection_topic, self.detection_input)
            self.control_loop()


# ================= 主入口 =================
if __name__ == "__main__":
    try:
        node = GimbalVisualServo()
        node.run()
    except rospy.ROSInterruptException:
        pass
