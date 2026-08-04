# xd_uav_controller

控制算法包。节点只消费统一控制状态和控制参考，输出机体角速度与归一化推力/油门；
它不调用MAVROS服务，也不直接维持OFFBOARD。

## 接口

输入：

```text
/uavX/control_manager/state          xd_uav_controller/ControlState
/uavX/control/reference/setpoint     mavros_msgs/PositionTarget
/uavX/control/reference/trajectory   trajectory_msgs/MultiDOFJointTrajectory
/move_base_simple/goal               geometry_msgs/PoseStamped
```

输出：

```text
/uavX/controller/command                    xd_uav_controller/ControlCommand
/uavX/control/reference/trajectory_path      nav_msgs/Path
```

服务：

```text
/uavX/controller/internal/command  xd_uav_controller/InternalCommand
```

这是控制器与控制管理器之间唯一的内部服务，不是操作接口。正常使用时只调用
`/uavX/control_manager/*`下的公开服务。ROS 1无法把跨节点服务从服务列表隐藏，
因此用`internal`命名空间明确标识。

外部参考使用ROS/MAVROS已有消息，不再额外定义一套模式消息：

- 所有参考保留在消息声明的语义坐标系中；轨迹采样或计算控制量之前，才根据最新TF2
  转换到当前`ControlState.header.frame_id`。控制律内部始终只处理同一个连续odom
  坐标系。
  TF2会自动沿TF树求解变换，不需要为local、map或world分别写控制律。允许的惯性
  frame由`reference_input/allowed_frames`限制；同名但属于另一架飞机命名空间的frame不会
  被接受。
- 四旋翼和固定翼的单点/流式目标统一使用`mavros_msgs/PositionTarget`。控制器直接按
  `type_mask`逐轴判断是否使用位置XYZ、速度XYZ、加速度XYZ、yaw和yaw rate；例如
  X/Y可以使用速度控制，同时Z使用位置保持。被掩码忽略的字段不会读取，因此可以是
  NaN。加速度写在`acceleration_or_force`中，原来的独立
  `/control/reference/acceleration`接口已经取消。
- `PositionTarget.header.frame_id`必须填写，`coordinate_frame`当前必须设置为
  `FRAME_LOCAL_NED`。和MAVROS的ROS侧接口一致，消息数值仍按ROS ENU惯性坐标语义
  表达，并根据`header.frame_id`通过TF转换；这里的枚举用于表明“局部惯性系”，
  不能据其名称自行交换XY或反转Z。当前不接受`FORCE`或BODY/OFFSET frame。当参考
  frame与控制odom之间存在旋转时，该旋转不能混合掩码中启用和忽略的轴；这种逐轴
  指令建议直接使用`ControlState.header.frame_id`。
- 至少一个位置轴启用、所有速度/加速度轴和yaw rate均忽略的目标视为一次性锁存目标，
  可以只发一次；含速度、加速度或yaw rate的目标属于流式控制，需要持续发布，超过
  `reference_input/timeout`后四旋翼捕获当前位置并转为悬停，固定翼进入当前位置相切的等待
  盘旋。固定翼轨迹到达最后一个时间点后也会立即建立相切等待圆，不会继续追逐最后
  一个静止轨迹点；等待圆使用`V/R`角速度前馈维持配置半径。yaw是否启用不改变锁存
  判定。
- 四旋翼纯速度模式不要求同时启用任何位置轴。每个启用的速度轴直接跟踪对应速度值；例如
  启用VX/VY/VZ并填写`[0.3, 0, 0]`就是沿+X运动并把Y、Z速度保持为0。纯速度轴内部
  带有限幅积分补偿，用于消除阻力、模型误差和悬停推力偏差造成的稳态速度误差。
  未启用的轴不参与该模式的控制。
- 四旋翼某轴只启用加速度时，控制器使用去除重力后的`ControlState.acceleration_odom`
  对该轴形成PI加速度闭环，并在进入模式时继承切换前已经稳定的加速度补偿量；
  加速度状态无效或超时时拒绝输出。如果同一轴还启用了位置或速度，加速度字段仍作为
  MPC前馈，不额外叠加加速度PI。注意`AZ=0`只保持净加速度为0，不负责把已有垂直速度
  或高度误差恢复为0。
- 常用掩码应使用消息常量按位或生成，不建议在代码中只写数字。例如：位置+yaw为
  `IGNORE_VX|IGNORE_VY|IGNORE_VZ|IGNORE_AFX|IGNORE_AFY|IGNORE_AFZ|`
  `IGNORE_YAW_RATE`（数值2552）；纯速度为
  `IGNORE_PX|IGNORE_PY|IGNORE_PZ|IGNORE_AFX|IGNORE_AFY|IGNORE_AFZ|`
  `IGNORE_YAW|IGNORE_YAW_RATE`（3527）；纯加速度为
  `IGNORE_PX|IGNORE_PY|IGNORE_PZ|IGNORE_VX|IGNORE_VY|IGNORE_VZ|`
  `IGNORE_YAW|IGNORE_YAW_RATE`（3135）。截图中常见的掩码63只忽略位置和速度，
  实际会同时启用加速度、yaw和yaw rate。
- 只启用VX使用掩码3575；只启用AY使用掩码3455。控制器并不要求同一类型的XYZ一起
  启用，每一个位置、速度和加速度掩码位都会被独立解析。
- 水平速度加高度位置保持使用
  `IGNORE_PX|IGNORE_PY|IGNORE_VZ|IGNORE_AFX|IGNORE_AFY|IGNORE_AFZ|`
  `IGNORE_YAW|IGNORE_YAW_RATE`（3555）：填写`velocity.x/y`和`position.z`，
  其余被忽略字段可以保持为0或NaN。
- 固定翼也逐轴解析同一套掩码，并把所有有效组合统一归一化为course、course rate、
  空速、高度和爬升率参考：

  | 启用字段 | 横向解释 | 纵向解释 |
  |---|---|---|
  | PXY / PXYZ | 指向水平目标点 | PZ经高度PI生成爬升率 |
  | VXY / VXYZ | 速度方向给course、模长给空速；连续方向变化自动估计course-rate前馈 | VZ直接给爬升率 |
  | VXY + PZ（可再加VZ） | 同上 | 高度反馈与VZ前馈叠加 |
  | PXY + VXY（可带PZ/VZ） | 沿速度切线建立前视点，同时闭环消除水平位置偏差 | 按PZ/VZ组合处理 |
  | PXY + VXY + AXY | AXY与VXY统一计算曲率/course-rate前馈 | 按PZ/VZ/AZ组合处理 |
  | 仅PZ、仅VZ或PZ+VZ | 保持进入该掩码模式时的course | 按PZ/VZ组合处理 |
  | 任一上述模式 + yaw_rate | 显式course-rate前馈，优先于速度方向差分估计 | 不变 |
  | 任一上述模式 + yaw | 没有有效水平位置/速度方向时作为course参考 | 不变 |

  只启用PX或PY时，未启用的另一个水平位置轴使用飞机当前位置，不会读取消息里的
  占位值。固定翼不能悬停，因此零水平速度不会命令停车，而是保持course并使用巡航
  空速。固定翼把加速度解释为前馈而不是独立控制模式：水平加速度必须同时启用
  VX、VY、AX、AY，用于计算曲率；AZ必须与PZ或VZ一起使用，形成俯仰角速度前馈。
  纯加速度掩码3135仍会被明确拒绝。
- 多点轨迹使用`trajectory_msgs/MultiDOFJointTrajectory`，当前只接受一个机体：
  每个点必须有一个transform，velocity和acceleration可以整条轨迹一致地提供或省略，
  `time_from_start`必须严格递增。轨迹在点间线性插值，yaw按最短角距离插值；轨迹结束
  后持续保持最后一个点。轨迹采样后根据最新TF转换，因此local全局轨迹会随
  `local_origin -> odom`修正持续保持在原来的全局位置。
  固定翼轨迹优先使用水平速度切线计算期望course，`velocity.angular.z`作为course-rate
  转弯前馈；若该值为0但同时提供了水平速度和加速度，则根据轨迹曲率
  `(vx*ay-vy*ax)/(vx²+vy²)`计算前馈。轨迹yaw只在没有有效水平速度切线时作为
  course参考。流式`PositionTarget`没有加速度字段时，则从相邻水平速度方向的变化率
  估计并滤波course-rate；如果消息显式启用yaw_rate（包括值为0），始终以显式值为准。
  内部起飞、盘旋和降落参考不使用该差分估计。
  每条通过校验并被控制器接受的轨迹还会原样转换为latched的`nav_msgs/Path`，发布到
  `/uavX/control/reference/trajectory_path`。这个话题只用于RViz显示，不参与控制，
  四旋翼和固定翼共用同一套可视化接口。
- 固定翼进入近地滑跑阶段后，控制器使用当前地速与已有的进近空速自动连续衰减滚转、
  俯仰和偏航角速度指令；达到已有的触地速度条件时立即输出零角速度和零油门。该逻辑
  不引入额外滑跑参数，控制管理器仍独立执行持续触地确认后再上锁。
- 启动文件默认把全局`/move_base_simple/goal`接入控制器，便于使用RViz的“2D Nav
  Goal”。RViz的Fixed Frame既可以使用`uav1/odom`，也可以使用允许且对齐有效的
  `uav1/local_origin`。
  simple goal是一次发布、持续保持的目标；默认只使用XY和yaw，并继承控制器当前的
  期望高度，因为RViz通常会把2D目标的z写成0。连续发送多个2D目标不会反复采样带有
  波动的实测高度，也不会把高度目标逐点向下带。只有显式设置
  `reference_input/simple_goal/use_message_z: true`时才使用消息中的z。适配结果会保留原始参考
  frame，并以latched方式进入两种机型共用的
  `/uavX/control/reference/setpoint`单点路径。

新到达的单点会取消当前外部轨迹，新到达的轨迹也会接管单点。起飞完成后，任一标准
外部参考一旦到达都会接管内部悬停参考。降落期间外部参考会被忽略。

新参考只有在消息、frame白名单、全局对齐和TF转换全部通过后才会替换当前目标。
错误frame或暂时找不到TF的新消息只会被拒绝，控制器继续保持最后一个有效目标。活动
local/map/world目标所依赖的TF短暂失效时，在
`reference_input/transform/failure_grace_duration`内保持最后一个odom目标；持续失效后才把
控制输出标记为无效，交给控制管理器执行安全策略。对
`reference_input/global_alignment/frames`中的frame还会检查
`/uavX/single_tf_manager/local_alignment_valid`，避免把尚未对齐时的单位TF误当成
有效全局定位。

多机同时运行时不要让所有控制器监听同一个全局simple goal，可在系统launch中为每架
飞机设置不同的`simple_goal_topic`。

固定翼完成起飞并进入等待盘旋后，可以用附带的标准消息测试节点发布连续圆轨迹：

```bash
rosrun xd_uav_controller publish_fixedwing_trajectory.py \
  _uav_name:=uav1 _radius:=100.0 _airspeed:=15.0 \
  _direction:=ccw _relative_altitude:=0.0
```

节点从`/uav1/control_manager/state`读取当前位置和course，把当前位置设置为新圆的
切入点，然后向`/uav1/control/reference/trajectory`发布带位置、速度、yaw、yaw rate
和向心加速度的`MultiDOFJointTrajectory`。默认轨迹在当前控制odom中表达；设置
`_frame_id:=uav1/local_origin`时会先通过TF把切入点转换到local坐标。节点默认每圈
续期一次，使圆轨迹持续有效；测试结束后应明确调用降落或取消OFFBOARD，不能把停止
轨迹发布节点当成降落命令。`_direction:=cw`可改为顺时针，`_repeat:=false`只发布
一次有限时长轨迹。

在RViz中添加`Path`显示项，将Topic设为
`/uav1/control/reference/trajectory_path`。Fixed Frame可以使用轨迹本身的
`uav1/odom`或`uav1/local_origin`；也可以使用TF树中能够变换到该轨迹frame的其他
惯性坐标系。

没有外部参考或起飞请求时，控制器会自动捕获当前状态：四旋翼保持当前位置和yaw，
固定翼保持当前高度、course和空速。这样系统可以先预发送控制量并在未解锁时进入
OFFBOARD。固定翼完成起飞后会在切入点建立与当前航向相切的等待圆，持续定高盘旋；
新的单点或轨迹参考会退出等待盘旋。固定翼流式PositionTarget参考超时后不会中断控制输出，
而会在当前位置重新建立相切等待圆；四旋翼流式PositionTarget超时后转为当前位置
悬停。simple goal属于一次性锁存目标，不受该超时影响。

## 控制律

- 四旋翼：每轴状态为`[position, velocity]`的有限时域线性MPC，控制量为期望加速度；
  输出再经过jerk变化率限制形成期望合力，最后由SO(3)姿态误差生成body rates和
  collective thrust。位置/速度控制中的参考加速度作为MPC前馈量使用；位置轴使用
  限幅扰动积分消除风、载重和悬停推力偏差造成的定点静差，没有位置参考的纯速度轴
  使用速度积分补偿，纯加速度轴则使用估计加速度形成PI闭环。三类积分统一根据经过
  加速度、jerk、倾角和推力限制后的可实现加速度执行clamping与平滑退回，避免执行器
  饱和后的wind-up；专用起飞/降落阶段不启用位置积分。积分和jerk变化率限制使用实测
  控制周期，控制定时器短时抖动不会改变等效积分强度。
- 固定翼：轨迹、`PositionTarget`和内部参考先经过独立适配层，统一生成只包含course、
  course-rate、空速、高度误差、爬升率和垂直加速度的`FixedwingControlTarget`；核心
  控制律不再按消息来源分别实现公式。同一组P/V/A从轨迹和`PositionTarget`进入时，
  会产生相同的固定翼控制目标和输出。随后统一目标生成roll、pitch、
  协调转弯yaw rate与throttle。控制状态使用ROS ENU/FLU约定，因此正爬升对应负pitch，
  正course变化对应负roll；发给MAVROS后再由其转换到PX4的NED/FRD约定。固定翼和
  四旋翼不共享动力学控制律。外部参考同时包含水平位置和速度时，控制器沿速度切线
  构造`fixedwing/path_guidance/lookahead_distance`指定的前视点，再从飞机当前位置指向
  前视点生成course。这样速度提供轨迹方向前馈，位置误差负责把飞机拉回发布的空间
  轨迹，不会再出现“飞出的形状正确但整条轨迹平移”的开环现象。参考同时提供速度
  和加速度时，适配层根据曲率计算转弯半径，并在
  `guidance/minimum_lookahead_distance`与`maximum_lookahead_distance`之间平滑调整
  前视距离；急弯缩短、直线增大。轨迹和`PositionTarget`共用该逻辑。
  常规外部参考的course环带有抗饱和积分；缺少AXY的流式速度方向经过差分和低通形成
  转弯率前馈；前者消除风和舵效偏差，后者避免圆弧跟踪必须先积累航向误差才滚转。
  两者在掩码模式切换、参考源切换、超时或状态失效时清零，连续发布同一掩码不会每帧
  重置。高度
  通道使用带
  抗饱和的高度PI生成爬升率修正，叠加VZ前馈；滤波后的实际爬升率形成俯仰反馈，
  AZ形成俯仰角速度前馈。爬升率还提供小幅油门前馈，使飞机
  在改变势能时不必等空速下降后才补油。高度积分在起飞、降落、轨迹重启和状态失效
  时清零，避免模式切换后保留旧偏置。

第一版MPC采用在线有限时域Riccati递推和输入/状态限幅，不依赖MRS预编译求解器，也
不是带通用不等式QP约束的完整NMPC。

## 启动

控制参数按两层组织：

- `config/common_config.yaml`：状态输入、参考输入/TF检查和home。
- `config/multirotor.yaml`：四旋翼控制律、MPC、起飞和降落参数。
- `config/fixedwing.yaml`：固定翼控制律、起飞、进近和降落参数。

四旋翼参数也按职责分层：

- `multirotor/model/*`：重力常量和悬停推力工作点。
- `multirotor/attitude_control/*`：期望姿态到body-rate的内环增益。
- `multirotor/position_control/mpc/*`：MPC预测长度和代价权重。
- `multirotor/position_control/disturbance_rejection/*`：位置、速度和纯加速度模式的低频扰动补偿及抗积分饱和。
- `multirotor/limits/*`：推力、倾角、body-rate、速度、加速度和jerk限制。

公共参数分为三组：

- `state_input/*`：控制管理器状态消息的接收超时。
- `reference_input/*`：外部参考超时、允许坐标系、TF检查、全局对齐和simple goal适配。
- `home/*`：两种机型共用的Home语义、坐标系、位置和航向。

固定翼参数按职责分层，避免把输入适配、制导、控制增益和安全限制混为一组：

- `fixedwing/reference_adapter/*`：流式参考缺少AXY/yaw_rate时的course-rate估计。
- `fixedwing/path_guidance/*`：PXY+VXY前视点和自适应前视距离。
- `fixedwing/lateral_control/*`：course PI和滚转内环。
- `fixedwing/vertical_control/*`：高度PI、爬升率反馈、AZ前馈和俯仰内环。
- `fixedwing/energy_control/*`：空速、配平油门和能量补偿。
- `fixedwing/limits/*`：姿态、爬升率、body-rate和油门硬限制。
- `fixedwing/modes/loiter/*`：等待盘旋参考生成器。

旧版公共扁平路径与`reference_frames/*`、四旋翼扁平路径与`multirotor/mpc/*`中的限制
和积分路径，以及旧版固定翼扁平路径和`fixedwing/guidance/*`，仍可在未配置新路径时
兼容读取，并在启动时输出迁移提示；新配置或launch覆盖应使用上述新路径。

`controller.launch`先加载公共配置，再加载机型配置，因此机型文件或用户传入的自定义
机型文件可以覆盖公共默认值。完整系统launch也提供`controller_common_config`参数，
可在不复制机型参数的情况下替换公共配置。

单独启动四旋翼或固定翼控制节点：

```bash
UAV_NAME=uav1 roslaunch xd_uav_controller controller.launch \
  vehicle_type:=multirotor

UAV_NAME=uav1 roslaunch xd_uav_controller controller.launch \
  vehicle_type:=fixedwing
```

完整系统入口位于`xd_uav_control_manager`：

```bash
UAV_NAME=uav1 roslaunch xd_uav_control_manager multirotor_system.launch
UAV_NAME=uav1 roslaunch xd_uav_control_manager fixedwing_system.launch
```

## 起飞

`altitude`是首次调用时相对当前高度的增量：

```bash
rosservice call /uav1/control_manager/takeoff "altitude: 2.0"
```

四旋翼会锁定当前XY和yaw并生成平滑垂直参考。固定翼会锁定当前course，先以起飞油门
加速，达到`rotate_airspeed`后给定爬升pitch；达到目标高度后建立相切等待圆并持续
定高盘旋，直到收到外部控制参考。盘旋半径、速度和方向由
`fixedwing/modes/loiter/*`配置。
固定翼OFFBOARD起飞必须先在仿真中验证跑道、舵面方向和PX4失效参数，不能直接用于
真机。

同一次运行中重复调用起飞服务不会累计目标高度，也不会重置正在执行的起飞过程。

控制管理器的组合起飞服务会先建立起飞参考，再依次请求OFFBOARD和解锁。起飞完成后
内部参考继续保持目标，OFFBOARD不会退出；四旋翼在目标点悬停，固定翼在等待圆盘旋。
外部参考一旦到达，会接管并取消内部参考。

四旋翼原地降落会保持调用服务时的XY和yaw，并通过连续移动的高度参考限速下降；进入
近地高度后使用更低的最终下降速度。返航降落会先以限速水平参考返回home，
满足位置和速度容差后再执行相同的垂直下降。控制器不会把高度目标一步跳到地面。
下降末段的参考会略低于本次降落目标的地面高度，避免飞机接地后重新回到悬停推力；
位置、高度和垂直速度满足容差时，`ControlCommand.landing_touchdown`会通知管理器
执行停桨。

降落触地判定前，控制管理器可通过内部`CANCEL_LANDING`命令撤销降落。撤销时不会
恢复此前可能已经过期的点或轨迹参考：四旋翼捕获当前三维位置和yaw悬停，固定翼以
当前位置、当前高度和当前course建立相切等待圆。两者都继续输出有效控制量，供管理器
保持解锁OFFBOARD；后续外部参考仍可正常接管。控制器一旦报告`landing_touchdown`，
取消命令会被拒绝，避免接地后恢复飞行推力。

固定翼使用独立的降落状态机，不能执行垂直原地下降。`land`会在当前course前方、
本次起飞地面高度上建立临时接地点；`land_home`把公共home配置解析出的三维位置作为
接地点。飞机先飞向接地点上游的进近点；进入捕获半径后，控制器保持进近高度，
把原来的指点航向按可实现转弯率连续过渡到最终进近直线。完成直线捕获后只开启纵向
下滑，横向参考保持连续，随后按固定下滑角下降，
近地时从“标称下滑率×高度时间常数”自动得到的过渡高度开始，连续减小下沉率和
油门并把机头平滑拉向水平；接地前仍保留一半标称下沉率，避免切换固定拉平俯仰、
瞬间断油或渐近悬在跑道上方，
最后沿着陆course滑跑。控制器只会在进入滑跑、相对接地点高度和垂直速度满足容差
且地速足够低时发布`landing_touchdown`。
进近点捕获半径会根据着陆速度和允许滚转角对应的可实现转弯半径自动放大；捕获后
控制器沿完整着陆直线闭环消除横向偏差，避免固定翼因追逐不可达的静止点而绕圈。

固定翼`land_home`执行期间会把仍保存在`home/frame`中的接地点按最新TF转换到当前
控制odom。`home/use_home_yaw: true`时使用`home/fixed_yaw`作为跑道着陆方向；
为`false`时使用调用降落服务时的course。`home/fixed_position.z`必须填写跑道地面
在`home/frame`中的高度。当前实现是面向SITL的基础直线进近，不含地形测高、跑道
占用检查、侧风补偿或复飞决策，真机前必须另行补齐并验证这些安全能力。

代码未提供参数时以`home/mode: takeoff`为回退行为：起飞时优先把当前位置记录在
`home/frame`
（默认同一飞机命名空间下的`local_origin`）中；如果当时全局对齐或TF不可用，才退化
为在当前odom中记录。`land_home`执行期间，home仍保留在这个原始坐标系中，每个控制
周期按最新TF转换到当前odom，因此后续`local_origin -> odom`修正不会改变物理返航点。

需要固定降落点时显式设置`home/mode: fixed_local`，并配置`home/frame`、
`home/fixed_position`和`home/fixed_yaw`。固定home不会被起飞、落地或内部状态复位
覆盖。默认`home/use_home_yaw: false`，返航时保持开始降落时的航向；设为`true`后才
使用home航向。`local_origin`只是坐标系名称，并不表示固定点一定是坐标原点，只有
`fixed_position: [0, 0, 0]`时才会返回该原点。仓库提供的`common_config.yaml`
当前选择`fixed_local`，固定翼使用前应把其中的位置、地面高度和跑道航向改成实际
仿真场景对应值。
