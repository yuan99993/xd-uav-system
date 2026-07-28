---
name: xd-ros-pkg-style
description: xd-uavsystem-test 代码风格规范，用于将外部代码改写为标准 ROS1 catkin 功能包。仅在此任务及 src/xd-uavsystem-test/ 目录下生效。
metadata:
  scope: src/xd-uavsystem-test/
  task: 将 SEAD 和 PixEagle 改写为 ROS 功能包
---

# xd-uavsystem-test — ROS1 Catkin 功能包代码风格规范

本规范提取自 `src/xd-uavsystem-test/` 下三个标准 ROS 包的结构、命名、配置和构建约定。
**生效范围**：仅限"将 SEAD / PixEagle 改写为 ROS 功能包"这一任务，以及 `src/xd-uavsystem-test/` 目录下的所有产出。

---

## 一、元仓库顶层结构

```
xd-uavsystem-test/                 # 元仓库根目录，同时作为 catkin workspace
├── CMakeLists.txt                 # → /opt/ros/noetic/share/catkin/cmake/toplevel.cmake（软链接）
├── README.md                      # 仓库级说明
├── .gitignore                     # 见下方模板
├── src/                           # 所有 ROS 包放在这里
│   ├── xd_uav_state_estimators/   # 包 1
│   ├── xd_uav_single_TF_manager/  # 包 2
│   └── xd_uav_world_TF_manager/   # 包 3
└── temp/                          # 过程文件、任务清单（不提交）
```

关键规则：
- 顶层 `src/CMakeLists.txt` **必须是软链接**，指向系统 catkin toplevel。
- 每个 ROS 包是 `src/` 下的独立目录，**包之间平级，不存在嵌套**。

---

## 二、单个 ROS 包标准布局

```
<包名>/
├── CMakeLists.txt          # catkin 构建（每包一个）
├── package.xml             # format="2"
├── README.md               # 包级说明（必写，中文）
├── config/                 # YAML 可配置参数（概念上等价于 launch 的 arg/default）
│   └── *.yaml
├── launch/                 # ROS .launch 文件
│   └── *.launch
├── msg/                    # （按需）自定义 ROS 消息 .msg
├── srv/                    # （按需）自定义 ROS 服务 .srv
├── src/                    # 源码：C++ 用 .cpp，Python 可在此放库模块
├── scripts/                # （按需）可执行 Python 节点脚本
└── test/                   # （按需）rostest 集成测试
```

### 2.1 必备文件清单

每个 ROS 包 **至少** 要有：
1. `package.xml` — 依赖声明
2. `CMakeLists.txt` — 构建规则
3. `README.md` — 中文文档，描述职责、TF 树/话题、配置、启动命令

### 2.2 文件放置约定

| 内容 | 位置 | 说明 |
|---|---|---|
| C++ 节点源码 | `src/<功能>_node.cpp` | 每包通常只有一个可执行节点 |
| Python 节点脚本 | `scripts/<功能>_node.py` | 只用于测试或工具；核心逻辑用 C++ |
| 可配置参数 | `config/<用途>.yaml` | 通过 `rosparam load` 加载到节点私有 NS |
| ROS 自定义消息 | `msg/<Name>.msg` | 标准格式：类型声明 + 注释 |
| ROS 自定义服务 | `srv/<Name>.srv` | `---` 分隔 request 和 response |
| 启动文件 | `launch/<功能>.launch` | XML，使用 `$(optenv UAV_NAME uav1)` 模式 |

---

## 三、命名约定

### 3.1 包名

```
xd_uav_<功能描述>
```

- 小写 + 下划线分隔
- 前缀 `xd_uav_` 表示信达无人机系统
- 例如：`xd_uav_sead`、`xd_uav_pixeagle_tracker`

### 3.2 可执行文件名

```
<功能描述>_node
```

例如：`single_tf_manager_node`、`multi_source_estimator_node`、`odometry_adapter_manager_node`

### 3.3 launch 参数命名

```xml
<arg name="UAV_NAME" default="$(optenv UAV_NAME uav1)"/>
<arg name="uav_name" default="$(arg UAV_NAME)"/>
```

- 大写 `UAV_NAME` 用于环境变量/命令行注入
- 小写 `uav_name` 用于内部传递

### 3.4 命名空间隔离

```xml
<group ns="$(arg uav_name)">
  <!-- 所有话题自动前加 /uavX/ -->
</group>
```

- 所有节点启动在 `/uavX/` 命名空间下
- 方便多机扩展

### 3.5 坐标系命名

```
uavX/local_origin
├── uavX/odom
│   └── uavX/base_link
│       ├── uavX/lidar_link
│       │   └── uavX/lidar_imu_link
│       └── ...
└── world（全局帧，只由 world_tf_manager 发布）
```

---

## 四、package.xml 模板

```xml
<?xml version="1.0"?>
<package format="2">
  <name>包名</name>
  <version>1.0.0</version>
  <description>一句话中文描述。</description>

  <maintainer email="邮箱">作者名</maintainer>
  <license>BSD-3-Clause</license>

  <buildtool_depend>catkin</buildtool_depend>

  <!-- 按需声明其他依赖 -->

  <export/>
</package>
```

### 4.1 常见依赖声明

- `rospy` — `<depend>rospy</depend>`
- `roscpp` — `<depend>roscpp</depend>`
- `std_msgs` — `<depend>std_msgs</depend>`
- `geometry_msgs` — `<depend>geometry_msgs</depend>`
- `sensor_msgs` — `<depend>sensor_msgs</depend>`
- `nav_msgs` — `<depend>nav_msgs</depend>`
- `mavros_msgs` — `<depend>mavros_msgs</depend>`
- `tf2` / `tf2_ros` — `<depend>tf2</depend>` + `<depend>tf2_ros</depend>`
- 自定义 msg/srv — 需 `<build_depend>message_generation</build_depend>` + `<exec_depend>message_runtime</exec_depend>`
- 测试 — `<test_depend>rospy</test_depend>` + `<test_depend>rostest</test_depend>`

---

## 五、CMakeLists.txt 模板

### 5.1 纯 C++ 包

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(包名)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  # 与 package.xml 中的 <depend> 保持一致
  roscpp
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(节点名_node src/节点名_node.cpp)
add_dependencies(节点名_node ${catkin_EXPORTED_TARGETS})
target_compile_features(节点名_node PRIVATE cxx_std_17)
target_link_libraries(节点名_node ${catkin_LIBRARIES})

install(TARGETS 节点名_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
```

### 5.2 含自定义 msg/srv 的包

在 5.1 基础上增加：

```cmake
add_message_files(FILES Name1.msg Name2.msg)
add_service_files(FILES Name1.srv)

generate_messages(DEPENDENCIES std_msgs)
```

### 5.3 纯 Python 包（无 C++ 编译）

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(包名)

find_package(catkin REQUIRED COMPONENTS rospy)

catkin_package(CATKIN_DEPENDS rospy)

# 安装 Python 库模块
catkin_install_python(PROGRAMS
  scripts/节点名_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY src/${PROJECT_NAME}/ config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
```

### 5.4 C++/Python 混合包

结合 5.1 和 5.3，`find_package` 中同时声明 `rospy` + `roscpp`。

### 5.5 关键编译选项

```cmake
add_compile_options(-Wall -Wextra -Wpedantic)         # 必须
target_compile_features(节点名_node PRIVATE cxx_std_17) # C++17 必须
```

---

## 六、Launch 文件约定

```xml
<?xml version="1.0"?>
<launch>
  <!-- 命名空间参数 -->
  <arg name="UAV_NAME" default="$(optenv UAV_NAME uav1)"/>
  <arg name="uav_name" default="$(arg UAV_NAME)"/>

  <!-- 配置文件路径 -->
  <arg name="config" default="$(find 包名)/config/xxx.yaml"/>

  <group ns="$(arg uav_name)">
    <node pkg="包名"
          type="可执行文件名"
          name="节点名称"
          output="screen"
          respawn="false"
          clear_params="true">
      <rosparam command="load" file="$(arg config)"/>
      <!-- launch 参数可覆盖 YAML 中的值 -->
      <param name="uav_name" value="$(arg uav_name)"/>
    </node>
  </group>
</launch>
```

规则：
- `output="screen"` 用于调试；稳定后可改 `log`
- `clear_params="true"` 防止残留参数干扰
- `$(optenv UAV_NAME uav1)` 让环境变量传入命名空间

---

## 七、YAML 配置文件约定

```yaml
# 模块名 / 用途说明（一行中文注释）
# 数值参数应有物理含义注释和单位

publish_rate: 30.0          # Hz
input_timeout: 1.0          # seconds

feature_a:
  enabled: true
  parent_frame: local_origin
  child_frame: odom

  # 布尔/可选参数注明默认值和含义
  fallback_identity: true

  # 数组/列表
  sources: [mavros, fastlio]

  # 复杂嵌套
  filter:
    initial_position_variance: 1.0
    position_process_noise: 0.05
```

规则：
- YAML 内注释用 `#`，中文或英文皆可，但参数含义必须写清
- 带单位的参数在注释中注明（Hz, seconds, meters）
- 布尔开关用 `enabled: true/false`
- 不要硬编码 frame 名——由 launch 文件传入

---

## 八、C++ 代码风格

### 8.1 头文件组织

```cpp
// 标准库
#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

// ROS
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

// tf2
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

// 本包消息/服务
#include <本包名/消息名.h>

// 第三方（Eigen 等）
#include <Eigen/Dense>
```

### 8.2 匿名命名空间

```cpp
namespace {

constexpr double kPi = 3.14159265358979323846;

double wrapAngle(const double value) {
  return std::atan2(std::sin(value), std::cos(value));
}

}  // namespace
```

- 文件内部使用的常量、辅助函数放入匿名 namespace
- 常量命名 `kXxxYyy`

### 8.3 类/节点设计

每个 ROS 包只包含一个主节点类，在构造函数中完成初始化，析构函数中清理资源。

---

## 九、测试规范

### 9.1 测试位置

```
test/
├── test_<功能>.py            # Python 测试
└── <功能>.test               # rostest 描述文件
```

### 9.2 测试框架

- 核心逻辑：Python `unittest` + `rospy` + `rostest`
- 测试类继承 `unittest.TestCase`
- 使用 `_wait_for(predicate, timeout)` 模式等待异步消息

### 9.3 启动方式

```bash
rostest 包名 测试.test
```

---

## 十、README.md 内容结构

```markdown
# 包名

一句话描述包的功能。

## 功能职责

- 负责什么
- 不负责什么

## TF / 话题

列出主要的发布/订阅话题和坐标系。

## 配置

说明 config YAML 的关键参数

## 启动

```bash
UAV_NAME=uav1 roslaunch 包名 功能.launch
```
```

---

## 十一、.gitignore（最小通用模板）

关键项：
- `build/` `devel/` `install/` `logs/` `src/CMakeLists.txt`
- `__pycache__/` `*.py[cod]`
- `.vscode/` `.idea/`
- `*.bag` `*.bag.active`

---

## 十二、与 MRS 风格的关键差异

| 方面 | MRS (`mrs_uav_*`) | xd (`xd_uav_*`) |
|---|---|---|
| 命名空间前缀 | `uav` 后不加数字 | `/uavX/` group ns |
| 多机支持 | 隐式，launch 参数传递 | 显式 `UAV_NAME` 环境变量 |
| 语言 | C++ + Python 混合 | 核心全 C++17，仅测试用 Python |
| 依赖声明 | 使用变量 `CATKIN_DEPENDENCIES` | 直接列在 `find_package` / `catkin_package` |
| gitignore | 简单 | 非常详细的分类 |
| 复杂度 | 每包多节点 + nodelet | 每包单一责任，单节点 |
| 配置 | param YAML | 更结构化的 YAML + 详细注释 |
| 仓库 | GitHub | Gitee |

---

## 十三、改写检查清单

将外部代码转为 xd 风格 ROS 包时，逐条对照：

1. [ ] 包名符合 `xd_uav_<描述>` 规范
2. [ ] `package.xml` format="2"，依赖完整
3. [ ] `CMakeLists.txt` 包含正确的 `find_package` / `catkin_package`
4. [ ] 编译选项 `-Wall -Wextra -Wpedantic` + `cxx_std_17`
5. [ ] `install()` 规则覆盖 config、launch、可执行目标、README
6. [ ] launch 文件使用 `$(optenv UAV_NAME uav1)` 模式
7. [ ] 节点放入命名空间 `<group ns="$(arg uav_name)">`
8. [ ] YAML 配置放在 `config/`，有中文注释说明参数含义
9. [ ] `README.md` 包含功能描述、话题/TF 列表、配置说明、启动命令
10. [ ] 源码依赖不溢出包边界（不引用本包之外的私有头文件/模块）
11. [ ] 若引用其他 ROS 包的消息/服务，已在 `package.xml` 中声明
