# xd_uav_sead 完善计划

> 日期：2026-07-29
> 基于今天对完整代码的审查

---

## 发现的问题（按优先级排列）

### P0: 运行时模式参数不生效 🔴

**位置**: [sead_onboard_node.py:35-46](scripts/sead_onboard_node.py#L35)

```python
simple_strike_control_mode = os.environ.get("SIMPLE_STRIKE_CONTROL_MODE", ...)
sead_control_mode = os.environ.get("SEAD_CONTROL_MODE", ...)
sead_runtime_mode_env = os.environ.get("SEAD_RUNTIME_MODE", ...)
```

这些变量在模块级（import 时）从 `os.environ` 读取。但 launch 文件通过 `<param name="sead_runtime_mode".../>` 设置的是 rospy 参数。

**后果**: `sead_strike_demo.launch`、`sead_formation_demo.launch`、`sead_waypoint_demo.launch` 中设置的 `<param name="sead_runtime_mode">` 和 `<param name="simple_strike_control_mode">` **完全无效**。

**修复**: 在 `rospy.init_node()` 后，用 `rospy.get_param("~xxxx", os.environ.get("XXXX", default))` 覆盖模块级默认值。

### P0: mock_gcs.py 未安装 🔴

**位置**: [CMakeLists.txt:28-30](CMakeLists.txt#L28)

```cmake
catkin_install_python(PROGRAMS
  scripts/sead_onboard_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

`mock_gcs.py` 不在安装列表中。

**后果**: `rosrun xd_uav_sead mock_gcs.py` 在 install 空间（非 devel）不可用。`sead_gazebo_demo.launch` 引用 mock_gcs 作为 node type 会失败。

**修复**: 把 `mock_gcs.py` 加入 `catkin_install_python`。

### P1: Wildcard imports（风格问题）🟡

**位置**: [sead_onboard_node.py:17-18](scripts/sead_onboard_node.py#L17)

```python
from xd_uav_sead.drone.drone import *
from xd_uav_sead.comms.communication_info import *
```

还有 [drone.py:17-18](drone/drone.py#L17)：
```python
from xd_uav_sead.comms.communication_info import *
from xd_uav_sead.planning.pathFollowing import *
```

**风险**: wildcard import 会污染命名空间，难以追踪依赖。

**修复**: 改为显式 import。对于 communication_info（约 30 个名字被引用），列出实际使用的名字。

### P1: 重复 import 清理 🟡

- [drone.py:17-28](drone/drone.py#L17): `from mavros_msgs.srv import SetMode` 出现 **3 次**（17行, 21行, 28行），`CommandBool` 出现 2 次
- [sead_onboard_node.py:19-27](scripts/sead_onboard_node.py#L19): `from time import time` + `import time as time_module` 并存（功能正常但风格不佳）

### P2: empty/weak error handlers 🟡

- [sead_onboard_node.py:1328-1330](scripts/sead_onboard_node.py#L1328):
  - 第一个 `except Exception` catch-all 只 `pass`，会吞掉 RuntimeError/TyperError 等真实 bug
  - 第二个 `except Exception as e` (L1332) 有 traceback 但第一个 block 已经吞掉了异常

- [sead_onboard_node.py:1403-1404](scripts/sead_onboard_node.py#L1403): 裸 `except: pass`

### P3: sead_gazebo_demo.launch 脚本化不足 🟢

mock_gcs 作为 node 运行后发布一次性命令即退出，sleep 5 秒的窗口可能不够等 mavros 就绪。

---

## 验证确认

### 二进制协议兼容性 ✅
手工逐字段验证了 rosbridge._serialize_info 和 communication_info.unpack_packet 之间的字节偏移，SEAD_mission (msg_id=18) 完全匹配。其余消息类型（takeoff/arm/waypoint/formation 等）也匹配。

### Python 语法 ✅
17/17 .py 文件通过 `py_compile`。

### catkin_make ✅
`catkin_make --pkg xd_uav_sead` 二次编译通过。

### roslaunch --ros-args ✅
`sead_onboard.launch` 通过。

---

## 执行顺序

1. **P0 fix**: mock_gcs.py 加入 CMakeLists.txt（1 行改动）
2. **P0 fix**: 运行时模式参数改为 rospy.get_param（10 行改动）
3. **P1 fix**: 清理 wildcard imports + 重复 import
4. **验证**: 全部 launch 文件 roslaunch --ros-args 检查
5. **验证**: 最终 catkin_make 编译
6. **更新**: TASK_LIST.md 进度
