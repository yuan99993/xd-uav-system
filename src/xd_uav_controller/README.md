# xd_uav_controller

控制算法包。节点只消费统一控制状态和控制参考，输出机体角速度与归一化推力/油门；
它不调用MAVROS服务，也不直接维持OFFBOARD。

## 接口

输入：

```text
/uavX/control_manager/state          xd_uav_controller/ControlState
/uavX/control/reference/odom         nav_msgs/Odometry
/uavX/control/reference/acceleration geometry_msgs/AccelStamped
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

外部参考只使用ROS标准消息：

- 所有参考保留在消息声明的语义坐标系中；轨迹采样或计算控制量之前，才根据最新TF2
  转换到当前`ControlState.header.frame_id`。控制律内部始终只处理同一个连续odom
  坐标系。
  TF2会自动沿TF树求解变换，不需要为local、map或world分别写控制律。允许的惯性
  frame由`reference_frames/allowed`限制；同名但属于另一架飞机命名空间的frame不会
  被接受。
- 单点目标使用`nav_msgs/Odometry`。`pose`表达目标位置和姿态；控制器从姿态中提取
  yaw。`twist`遵循Odometry语义，在`child_frame_id`中表达：当
  `child_frame_id != header.frame_id`时，控制器使用目标姿态把线速度和角速度旋转到
  参考父坐标系，再通过TF旋转到控制odom。因此`header.frame_id`和
  `child_frame_id`都必须填写，后者只能是控制机体frame或与前者相同。
- 单点加速度前馈可另外发布`geometry_msgs/AccelStamped`。线加速度根据它自己的
  `header.frame_id`旋转到控制odom，不要求与Odometry参考使用同一个源frame。超时或
  单次转换失败后本周期不使用该前馈，不影响位置/速度参考。
- 多点轨迹使用`trajectory_msgs/MultiDOFJointTrajectory`，当前只接受一个机体：
  每个点必须有一个transform，velocity和acceleration可以整条轨迹一致地提供或省略，
  `time_from_start`必须严格递增。轨迹在点间线性插值，yaw按最短角距离插值；轨迹结束
  后持续保持最后一个点。轨迹采样后根据最新TF转换，因此local全局轨迹会随
  `local_origin -> odom`修正持续保持在原来的全局位置。
  每条通过校验并被控制器接受的轨迹还会原样转换为latched的`nav_msgs/Path`，发布到
  `/uavX/control/reference/trajectory_path`。这个话题只用于RViz显示，不参与控制，
  四旋翼和固定翼共用同一套可视化接口。
- 启动文件默认把全局`/move_base_simple/goal`接入控制器，便于使用RViz的“2D Nav
  Goal”。RViz的Fixed Frame既可以使用`uav1/odom`，也可以使用允许且对齐有效的
  `uav1/local_origin`。
  simple goal是一次发布、持续保持的目标；默认只使用XY和yaw，并继承控制器当前的
  期望高度，因为RViz通常会把2D目标的z写成0。连续发送多个2D目标不会反复采样带有
  波动的实测高度，也不会把高度目标逐点向下带。只有显式设置
  `simple_goal/use_message_z: true`时才使用消息中的z。适配后的
  `nav_msgs/Odometry`会保留原始参考frame，以latched方式发布到
  `/uavX/control/reference/odom`，再通过与普通单点输入相同的TF路径进入控制器。

新到达的单点会取消当前外部轨迹，新到达的轨迹也会接管单点。起飞完成后，任一标准
外部参考一旦到达都会接管内部悬停参考。降落期间外部参考会被忽略。

新参考只有在消息、frame白名单、全局对齐和TF转换全部通过后才会替换当前目标。
错误frame或暂时找不到TF的新消息只会被拒绝，控制器继续保持最后一个有效目标。活动
local/map/world目标所依赖的TF短暂失效时，在
`reference_frames/failure_grace_duration`内保持最后一个odom目标；持续失效后才把
控制输出标记为无效，交给控制管理器执行安全策略。对
`reference_frames/global_alignment_frames`中的frame还会检查
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
新的单点或轨迹参考会退出等待盘旋。流式Odometry参考超时后不会中断控制输出，而会
在当前位置重新建立相切等待圆。simple goal属于一次性锁存目标，不受该超时影响。

## 控制律

- 四旋翼：每轴状态为`[position, velocity]`的有限时域线性MPC，控制量为期望加速度；
  输出再经过jerk变化率限制形成期望合力，最后由SO(3)姿态误差生成body rates和
  collective thrust。参考加速度作为MPC前馈量使用。
- 固定翼：位置/速度参考先转换为course、高度、爬升率和空速，再生成roll、pitch、
  协调转弯yaw rate与throttle。控制状态使用ROS ENU/FLU约定，因此正爬升对应负pitch，
  正course变化对应负roll；发给MAVROS后再由其转换到PX4的NED/FRD约定。固定翼和
  四旋翼不共享动力学控制律。外部参考同时包含水平位置和速度时，控制器沿速度切线
  构造`fixedwing/guidance/lookahead_distance`指定的前视点，再从飞机当前位置指向
  前视点生成course。这样速度提供轨迹方向前馈，位置误差负责把飞机拉回发布的空间
  轨迹，不会再出现“飞出的形状正确但整条轨迹平移”的开环现象。

第一版MPC采用在线有限时域Riccati递推和输入/状态限幅，不依赖MRS预编译求解器，也
不是带通用不等式QP约束的完整NMPC。

## 启动

控制参数按两层组织：

- `config/common_config.yaml`：状态与参考超时、参考坐标系/TF检查、simple goal和home。
- `config/multirotor.yaml`：四旋翼控制律、MPC、起飞和降落参数。
- `config/fixedwing.yaml`：固定翼控制律、起飞、进近和降落参数。

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
定高盘旋，直到收到外部控制参考。盘旋半径、速度和方向由`fixedwing/loiter/*`配置。
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

固定翼使用独立的降落状态机，不能执行垂直原地下降。`land`会在当前course前方、
本次起飞地面高度上建立临时接地点；`land_home`把公共home配置解析出的三维位置作为
接地点。飞机先飞向接地点上游的进近点并对准着陆course，随后按固定下滑角下降，
近地进入拉平并收油门，最后沿着陆course滑跑。控制器只会在进入滑跑、相对接地点
高度和垂直速度满足容差且地速足够低时发布`landing_touchdown`。
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
