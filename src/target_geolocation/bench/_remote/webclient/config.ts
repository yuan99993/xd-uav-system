// ============================================================================
// 全局配置 —— 扩展时主要改这里
// ============================================================================

// 主机 IP/端口改为可配置（hostConfig），支持 URL `?host=` / localStorage / 界面修改。
// 向导出的常量保留初始值以便兼容旧 import，运行时请用动态 getter。
import {
  getHostConfig,
  buildRosbridgeUrl, buildRosbridgeHeavyUrl,
  buildWsBridgeUrl, buildSaveServerUrl,
} from './hostConfig';

const _host = getHostConfig();

/** rosbridge WebSocket 地址（初始值）。运行时动态地址用 rosbridgeUrl() */
export const ROSBRIDGE_URL = buildRosbridgeUrl(_host);
/** rosbridge 重话题地址（初始值）。运行时用 rosbridgeUrlHeavy() */
export const ROSBRIDGE_URL_HEAVY = buildRosbridgeHeavyUrl(_host);

/** C++ WebSocket 桥地址（初始值）。运行时用 wsBridgeUrl() */
export const WS_BRIDGE_URL = buildWsBridgeUrl(_host);

/** save_server.py 地址（初始值）。运行时用 saveServerUrl() */
export const SAVE_SERVER_URL = buildSaveServerUrl(_host);

export const rosbridgeUrl = () => buildRosbridgeUrl(getHostConfig());
export const rosbridgeUrlHeavy = () => buildRosbridgeHeavyUrl(getHostConfig());
export const wsBridgeUrl = () => buildWsBridgeUrl(getHostConfig());
export const saveServerUrl = () => buildSaveServerUrl(getHostConfig());

// ----------------------------------------------------------------------------
// UE 视窗（AirSim Unreal Engine Pixel Streaming）
// UE Sim 与 SignallingWebServer 运行在 UE 主机（Windows，与本网络可互通）。
// UE 主机可能不是 Jetson，地址/端口必须可按需覆盖：
//   1. 写死默认值（下面）
//   2. 前端可用 URL 参数 `?ue=<ip>:<httpPort>&uectl=<ip>:<ctlPort>` 覆盖
//   3. 运行时可在界面修改（存 localStorage）
// ----------------------------------------------------------------------------
export const UE_HOST = '192.168.5.4';          // UE 主机 IP（SignallingWebServer HTTP 页）
export const UE_HTTP_PORT = 80;              // UE SignallingWebServer 提供的 Player 页 HTTP 端口
export const UE_CONTROL_PORT = 8090;         // ue_controller.py 控制服务端口
export const UE_ORCH_PORT = 8070;            // orchestrator.py 编排服务端口（Windows 宿主直接跑）
export const UE_PLAYER_URL = (host: string = UE_HOST, port: number = UE_HTTP_PORT) =>
  // HoveringMouse=1 让 UE Pixel Streaming 处于「悬浮鼠标」模式，
  // 点击 UE 界面菜单后光标不会被 requestPointerLock 捕获消失（默认 Locked 模式点击即隐藏鼠标）。
  `http://${host}:${port}/?HoveringMouse=1`;

/** 计算给定 host/port 的控制服务基地址 */
export const ueControlBase = (host: string, port: number) => `http://${host}:${port}`;

/** Camera / 机器人 ID，对应 launch 里的 camera_id（如 alexa / alex / ...） */
export const CAMERA_ID = 'alexa';

// 地图原点（基准站 WGS84）由 /realm/{cam}/general/gnss_base 实时更新，
// 见 src/geo.ts。这里导出同一对象的引用，方便旧代码兼容。
import { origin as _origin } from './geo';
export const ORIGIN = _origin;

/** 模拟数据模式：无 rosbridge 也能看效果。连上真实 ROS 后可在界面关闭。 */
export const SIMULATION_DEFAULT = false;

// ----------------------------------------------------------------------------
// 自由模式：云台控制 + 目标识别
// ----------------------------------------------------------------------------
/**
 * MAVROS 命名空间 —— 本系统约定为 `uav1`（与 Jetson 端
 * OpenREALM_ROS1_Bridge/gimbal_control/gimbal_control.py 的 `UAV_NS_DEFAULT` 一致）。
 *
 * ⚠️ 原来这里（以及下面所有 MAVROS_* 话题）写死的是 `/uav0` —— 那是**过期默认**。
 *    实测 ROS master 上 `/uav0/` 只有一个话题：
 *      /uav0/mavros/mount_control/command
 *    而它正是本文件这个 publisher 自己注册上去的（`/uav0` 下没有任何 mavros 节点）。
 *    实际存在的是 `/uav1|uav2|uav3/mavros/*`（各有 126 个话题）。
 *    ⇒ 后果是静默失效：云台指令发进了空话题，不报错、也不动。
 *    其中「进入建图控制 → 云台回正下视」（App.tsx）就是靠这个话题的，一直没生效。
 *
 * 命名空间只在这一个常量里定义，下面所有话题都由它派生 —— 换机只需改这一处。
 */
export const UAV_NS = 'uav1';

/** 云台控制话题（mavros 挂载控制，mavros_msgs/MountControl），
 *  与 OpenREALM_ROS1_Bridge/gimbal_control/gimbal_control.py 发布的是同一话题。 */
export const MOUNT_CONTROL_TOPIC = `/${UAV_NS}/mavros/mount_control/command`;
export const MOUNT_CONTROL_TYPE = 'mavros_msgs/MountControl';
/** MountControl.mode：2 = MAV_MOUNT_MODE_MAVLINK_TARGETING（角度制） */
export const MOUNT_CONTROL_MODE = 2;
/** 角度单位换算：MAV_CMD_DO_MOUNT_CONTROL 经 mavros 透传到 PX4 后，
 *  PX4 对 param1/param2/param3 直接做 math::radians()，即这三个参数的单位是【度】，不是厘度。
 *  所以这里必须是 1：若乘 100，则 1°→100°、5°→500°，两档都会被限幅成同一个角度，
 *  正下视 -90° 也会变成 -9000°。 */
export const MOUNT_ANGLE_SCALE = 1;
/** 云台俯仰限幅(deg)：0=水平前视，-90=正下视。
 *  ⚠️ 下限 -90 是硬约束：pitch < -90 时图像会上下翻转。 */
export const GIMBAL_PITCH_MIN = -90;
export const GIMBAL_PITCH_MAX = 45;
export const GIMBAL_YAW_MAX = 90;
/** 手动/预设云台：单步角度变化(deg) */
export const GIMBAL_STEP = 5;
/** 正下视（nadir）：俯仰 -90°，偏航归零 */
export const GIMBAL_NADIR = { pitch: -90, yaw: 0 };
/** 回中平视 */
export const GIMBAL_CENTER = { pitch: 0, yaw: 0 };

/** 目标识别框话题（vision_msgs/BoundingBox2D），由 drive_yolo 发布 */
export const DETECT_BOX_TOPIC = '/detect_multi/inference_box';
export const DETECT_BOX_TYPE = 'vision_msgs/BoundingBox2D';
/** 已画框识别图像话题（sensor_msgs/Image），用于前端预览 */
export const DETECT_IMAGE_TOPIC = '/camera/image_with_detections';

/** 自由模式前视相机原始画面话题（AirSim FrontCamera），drive_yolo 从它转发原图 */
export const FRONT_CAMERA_IMAGE_TOPIC = '/Sim/SceneBasicDrone/robots/Drone1/sensors/FrontCamera/scene_camera/image';

// ---- 自由模式叠加信息：来自飞控端 mavros（/<UAV_NS>/mavros/*），不改 gimbal_control.py ----
/** 经纬度/高度（sensor_msgs/NavSatFix）~4.5Hz */
export const MAVROS_GLOBAL_TOPIC = `/${UAV_NS}/mavros/global_position/global`;
export const MAVROS_GLOBAL_TYPE = 'sensor_msgs/NavSatFix';
/** 航向（std_msgs/Float64，deg）~4.5Hz */
export const MAVROS_COMPASS_TOPIC = `/${UAV_NS}/mavros/global_position/compass_hdg`;
export const MAVROS_COMPASS_TYPE = 'std_msgs/Float64';
/** 飞行状态（mavros_msgs/State）：connected/armed/mode */
export const MAVROS_STATE_TOPIC = `/${UAV_NS}/mavros/state`;
export const MAVROS_STATE_TYPE = 'mavros_msgs/State';
/** 飞控 HUD 遥测（mavros_msgs/VFR_HUD）：地速/空速/航向/高度/爬升率 */
export const MAVROS_VFR_HUD_TOPIC = `/${UAV_NS}/mavros/vfr_hud`;
export const MAVROS_VFR_HUD_TYPE = 'mavros_msgs/VFR_HUD';
/** 机体系速度（geometry_msgs/TwistStamped） */
export const MAVROS_VEL_BODY_TOPIC = `/${UAV_NS}/mavros/local_position/velocity_body`;
export const MAVROS_VEL_BODY_TYPE = 'geometry_msgs/TwistStamped';
/** 相对高度（mavros_msgs/Altitude） */
export const MAVROS_ALTITUDE_TOPIC = `/${UAV_NS}/mavros/altitude`;
export const MAVROS_ALTITUDE_TYPE = 'mavros_msgs/Altitude';
/** IMU 姿态（sensor_msgs/Imu，orientation 四元数 → roll/pitch/yaw deg）*/
export const MAVROS_IMU_TOPIC = `/${UAV_NS}/mavros/imu/data`;
export const MAVROS_IMU_TYPE = 'sensor_msgs/Imu';

// ---- SAR-SELECT-PIXEL：视频窗口点击选点（geometry_msgs/PointStamped） ----
// 在视频图像上单击 -> 以「原始图像像素(u,v)，原点左上」发布到对应飞行器的
// sar_yolo_detector 智能跟踪选点话题。消费者：/uavN/sar_yolo_detector/smart/*。
export const sarSelectPixelTopic = (uav: string = UAV_NS) =>
  `/${uav}/sar_yolo_detector/smart/select_pixel`;
export const SAR_SELECT_PIXEL_TYPE = 'geometry_msgs/PointStamped';
/** 允许「点击选点」的图像话题 key —— 目前仅 h264 解码图像（增强/原始）。 */
export const SELECT_PIXEL_TOPIC_KEYS = new Set<string>(['h264_enh', 'h264_orig']);

// ---- SAR-MANUAL-ROI：视频窗口拖拽「手选框」（sensor_msgs/RegionOfInterest） ----
// 在视频图像上按住左键拖出一个矩形 -> 以「原始图像像素，原点左上」的
// (x_offset, y_offset, width, height) 发布到对应飞行器的手选框话题，作为
// 检测/跟踪节点的初始 ROI。
// ⚠️ 清空约定：x_offset=y_offset=width=height=0（即零尺寸框）。
//    RegionOfInterest 的四个字段都是 uint32，不能像 select_pixel 那样用 -1 表示取消。
// ⚠️ 该消息**没有 header**，别照抄 select_pixel 的写法。
export const sarManualRoiTopic = (uav: string = UAV_NS) =>
  `/${uav}/sar_yolo_detector/manual/initial_roi`;
export const SAR_MANUAL_ROI_TYPE = 'sensor_msgs/RegionOfInterest';
/** 手选框最小边长（原始图像像素）：小于此值视为误点击，不发布。 */
export const SAR_MANUAL_ROI_MIN_PX = 4;

// ----------------------------------------------------------------------------
// 话题注册表 —— 列表即界面「图层」与订阅清单的来源
// 想新增一个可视化图层，只需在这里加一项 + 在对应 View 里处理。
// ----------------------------------------------------------------------------
export type TopicKind = 'mesh' | 'pointcloud' | 'ortho' | 'traj' | 'image' | 'video' | 'base';

export interface TopicDef {
  key: string;            // 唯一 key
  label: string;          // 界面显示名
  topic: string;          // ROS 话题（{cam} 会被 CAMERA_ID 替换）
  type: string;           // ROS 消息类型
  kind: TopicKind;        // 决定由哪个 View 消费、如何渲染
  stage: string;          // 阶段名
  defaultOn: boolean;     // 默认是否显示
  throttleRate?: number;  // rosbridge 节流(ms)，0 或不设=不限速
}

export const TOPICS: TopicDef[] = [
  {
    key: 'mesh',
    label: '3D 三角网',
    topic: '/realm/{cam}/mosaicing/mesh',
    type: 'visualization_msgs/Marker',
    kind: 'mesh',
    stage: 'mosaicing',
    defaultOn: false,   // 关闭：mesh Marker 消息巨大，导致 rosbridge 内存暴涨
    throttleRate: 1000,  // 1Hz，重新开启时用更低频率
  },
  {
    key: 'pointcloud',
    label: '3D 高程点云',
    topic: '/realm/{cam}/mosaicing/pointcloud',
    type: 'sensor_msgs/PointCloud2',
    kind: 'pointcloud',
    stage: 'mosaicing',
    defaultOn: false,
  },
  {
    key: 'lidar_sim',
    label: '仿真激光点云',
    topic: '/Sim/SceneBasicDrone/robots/Drone1/sensors/Lidar/lidar',
    type: 'sensor_msgs/PointCloud2',
    kind: 'pointcloud',
    stage: 'mosaicing',
    defaultOn: false,
    // 原始 ~18000 点/帧 x 10Hz = 3.8MB/s base64，rosbridge JSON 序列化扛不住，
    // 先限到 2Hz。确认带宽后再调，或在上游降采样（推荐）。
    throttleRate: 500,
  },
  {
    key: 'ortho',
    label: '正射影像（实时）',
    topic: '/realm/{cam}/mosaicing/update/ortho',
    type: 'realm_msgs/GroundImageCompressed',
    kind: 'ortho',
    stage: 'mosaicing',
    defaultOn: true,
    // 不传 throttleRate 给 roslib（会导致消息丢失），前端用 Date.now() 节流
    // 逐幅贴图：收到一幅 update patch 就贴一幅（与 OpenREALM/RViz 一致），避免整幅替换闪烁
  },
  {
    key: 'traj_visual',
    label: '视觉解算轨迹',
    topic: '/realm/{cam}/pose_estimation/pose/visual/utm',
    type: 'geometry_msgs/PoseStamped',
    kind: 'traj',
    stage: 'pose',
    defaultOn: true,    // 默认开：默认勾选接收/显示视觉轨迹
  },
  {
    key: 'traj_gnss',
    label: 'GNSS 轨迹',
    topic: '/realm/{cam}/pose_estimation/pose/gnss/utm',
    type: 'geometry_msgs/PoseStamped',
    kind: 'traj',
    stage: 'pose',
    defaultOn: true,    // 默认开：默认勾选接收/显示 GNSS 轨迹
    // 原 gnss/traj 是 nav_msgs/Path（61k点/19MB），rosbridge JSON序列化极慢导致前端几乎收不到更新。
    // 改用 gnss/utm（PoseStamped，~100B/条），前端逐帧累积，与视觉轨迹同频。
  },
  {
    key: 'gnss_base',
    label: '基准站定位',
    topic: '/realm/{cam}/general/gnss_base',
    type: 'sensor_msgs/NavSatFix',
    kind: 'base',
    stage: 'general',
    defaultOn: true,    // 开启：以真实基准站 WGS84 驱动 setOrigin(utmNorth/utmZone)；throttleRate=10000 已限 0.1Hz，开销可忽略
    throttleRate: 10000,  // 102Hz→0.1Hz，即使开启也大幅节流
  },
  {
    key: 'rgb',
    label: '正射影像（全局）',
    topic: '/realm/{cam}/mosaicing/rgb',
    type: 'sensor_msgs/Image',
    kind: 'image',
    stage: 'mosaicing',
    defaultOn: false,  // 未实现处理逻辑，默认关闭避免浪费带宽
  },
];

/** 用 CAMERA_ID 把 {cam} 占位替换成真实话题名 */
export function resolveTopic(t: string): string {
  return t.replace('{cam}', CAMERA_ID);
}

/** 控制服务（rviz_satellite / realm_ros 提供的 stage 控制） */
export const SERVICES = {
  requestStop: { name: '/realm/{cam}/mosaicing/request_stop', type: 'std_srvs/Trigger' },
  requestResume: { name: '/realm/{cam}/mosaicing/request_resume', type: 'std_srvs/Trigger' },
  requestReset: { name: '/realm/{cam}/mosaicing/request_reset', type: 'std_srvs/Trigger' },
  requestFinish: { name: '/realm/{cam}/mosaicing/request_finish', type: 'std_srvs/Trigger' },
  requestSave: { name: '/realm/{cam}/mosaicing/request_save', type: 'std_srvs/Trigger' },
  changeParam: { name: '/realm/{cam}/mosaicing/change_param', type: 'realm_msgs/ParameterChange' },
};

// ----------------------------------------------------------------------------
// 地图底图源（卫星/街道影像）。全部 WGS-84 无坐标偏移；瓦片加载失败时视图会回退深色背景。
// ----------------------------------------------------------------------------
export interface BasemapDef {
  label: string;
  /** Cesium ArcGisMapServerImageryProvider 用的服务 url（空 = 不用于 3D） */
  cesiumUrl: string;
  /** MapLibre raster tiles 模板 */
  tiles: string[];
  tileSize: number;
  attribution: string;
  /**
   * 该源**实际提供**的最大瓦片级别。留空 = 不限制（各渲染器用自身默认）。
   *
   * 必须夹住：超出源数据范围的级别，Esri 不报错，而是返回一张写着
   * "Map data not yet available" 的占位图（HTTP 200 + image/jpeg，约 2.5KB）。
   * 现象就是整屏灰底重复这张图，看着像底图坏了，其实是请求了不存在的层级。
   * 2026-10-26 实测：现场 108.84E/37.74N，z=19 是占位图，z=18 及以下正常。
   */
  maxZoom?: number;
}

export const BASEMAPS: Record<string, BasemapDef> = {
  esriSat: {
    label: 'Esri 卫星影像',
    cesiumUrl: 'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer',
    tiles: ['https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}'],
    tileSize: 256,
    attribution: '© Esri, Maxar, Earthstar Geographics',
    maxZoom: 18,
  },
  esriStreet: {
    label: 'Esri 街道图',
    cesiumUrl: 'https://server.arcgisonline.com/ArcGIS/rest/services/World_Street_Map/MapServer',
    tiles: ['https://server.arcgisonline.com/ArcGIS/rest/services/World_Street_Map/MapServer/tile/{z}/{y}/{x}'],
    tileSize: 256,
    attribution: '© Esri',
    maxZoom: 18,
  },
  cartoDark: {
    label: 'CartoDB 暗色',
    cesiumUrl: '',
    tiles: [
      'https://a.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}.png',
      'https://b.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}.png',
      'https://c.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}.png',
    ],
    tileSize: 256,
    attribution: '© CARTO © OpenStreetMap',
  },
  osm: {
    label: 'OpenStreetMap',
    cesiumUrl: '',
    tiles: ['https://tile.openstreetmap.org/{z}/{x}/{y}.png'],
    tileSize: 256,
    attribution: '© OpenStreetMap contributors',
  },
  txRoad: {
    label: '腾讯街道',
    cesiumUrl: '',
    tiles: [
      'https://rt0.map.gtimg.com/tile?z={z}&x={x}&y={y}&styleid=1',
      'https://rt1.map.gtimg.com/tile?z={z}&x={x}&y={y}&styleid=1',
      'https://rt2.map.gtimg.com/tile?z={z}&x={x}&y={y}&styleid=1',
      'https://rt3.map.gtimg.com/tile?z={z}&x={x}&y={y}&styleid=1',
    ],
    tileSize: 256,
    attribution: '© 腾讯地图',
  },
  txSat: {
    label: '腾讯卫星',
    cesiumUrl: '',
    tiles: [
      'https://rt0.map.gtimg.com/tile?z={z}&x={x}&y={y}&styleid=3',
      'https://rt1.map.gtimg.com/tile?z={z}&x={x}&y={y}&styleid=3',
      'https://rt2.map.gtimg.com/tile?z={z}&x={x}&y={y}&styleid=3',
      'https://rt3.map.gtimg.com/tile?z={z}&x={x}&y={y}&styleid=3',
    ],
    tileSize: 256,
    attribution: '© 腾讯地图',
  },
  txSatLabel: {
    label: '腾讯卫星注记',
    cesiumUrl: '',
    tiles: [
      'https://rt0.map.gtimg.com/tile?z={z}&x={x}&y={y}&styleid=4',
      'https://rt1.map.gtimg.com/tile?z={z}&x={x}&y={y}&styleid=4',
      'https://rt2.map.gtimg.com/tile?z={z}&x={x}&y={y}&styleid=4',
      'https://rt3.map.gtimg.com/tile?z={z}&x={x}&y={y}&styleid=4',
    ],
    tileSize: 256,
    attribution: '© 腾讯地图',
  },
};

export const DEFAULT_BASEMAP = 'esriSat';

// ----------------------------------------------------------------------------
// 可选图像话题 —— 动态视频面板的话题来源
// ----------------------------------------------------------------------------
export interface ImageTopicDef {
  key: string;
  label: string;
  topic: string;
  type: string;
  throttleRate?: number;  // rosbridge 节流(ms)
}

export const IMAGE_TOPICS: ImageTopicDef[] = [
  { key: 'camera', label: '建图图像', topic: '/realm/{cam}/img', type: 'sensor_msgs/Image' },
  { key: 'tracked', label: '跟踪图像', topic: '/realm/{cam}/pose_estimation/tracked', type: 'sensor_msgs/Image' },
  { key: 'dens_img', label: '稠密化图像', topic: '/realm/{cam}/densification/img', type: 'sensor_msgs/Image' },
  { key: 'dens_depth', label: '深度图像', topic: '/realm/{cam}/densification/depth_display', type: 'sensor_msgs/Image' },
  { key: 'rectified', label: '矫正图像', topic: '/realm/{cam}/ortho_rectification/rectified', type: 'sensor_msgs/Image' },
  { key: 'undistorted', label: '去畸变图像', topic: '/realm/{cam}/ortho_rectification/undistorted', type: 'sensor_msgs/Image' },
  { key: 'h264_enh', label: '增强图像', topic: '/Drone1/Down/h264_recv/image', type: 'sensor_msgs/Image' },
  { key: 'h264_orig', label: '原始图像', topic: '/Drone1/Down/h264_recv/image_orig', type: 'sensor_msgs/Image' },
];

/** 重话题 key 集合 —— 这些话题走独立 rosbridge 实例(9091)，避免大消息序列化阻塞轻话题 */
export const HEAVY_TOPIC_KEYS = new Set([
  'ortho', 'rgb', 'pointcloud', 'lidar_sim',
  ...IMAGE_TOPICS.map((t) => t.key),
]);

