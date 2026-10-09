# SAR YOLO / PixEagle 四模式融合：需求、功能、问题与交接汇总

最新选型确认（2026-10-09）：用户确定 `/home/promise/mrs_test/sar_fusion_experiment.wlpAs3/src` 的
SAR与XD两包为后续正式方案。只有SAR已作实现修改，XD保持导出基线。
历史实验称谓/旧正式目录称谓不再决定后续默认选型；完整部署和网页合同以
[正式方案指南](web_frontend_integration_guide.md)为准。正式选型不替代UE/实机验收，本次没有搬迁、覆盖或推送。

原交接记录日期：2026-10-08，Asia/Shanghai。

## 2026-10-09 更新：当前状态优先以本节和新版指南为准

已取得最新分支 `3db09b001e903f70ca56815b325dba18d40fb756`，重新导出两个包到实验工作区。
按照用户新增约束，XD 导出后未做任何改动，原 SAR 消息和服务也逐文件保持不变。
旧实验源码与构建产物已备份至 `/home/promise/mrs_test/sar_fusion_backup_20261009.CpqpkW`。

- 最新上游允许四类 `[0,1,2,3]`，检测门限 0.10；下文 0.60 / 三类描述是旧版本历史，不是当前配置。
- 最新 PixEagle 入口移到 archive/launch，原样保留。新增唯一入口 `launch/xd_smart_tracker_fusion_integration.launch`，避免同名递归冲突。
- 保留原 SmartTracker、FeatureTracker、ColorTracker，新增 opt-in FusionTracker，使用相同 train7.pt / CUDA 权重，不替换 TensorRT。
- 保留最新上游的 select_pixel、selected_tracked_detections 和云台话题参数；fusion 的 selected 流只输出同一最终框。
- 不再扩展旧 TrackedDetection2D：新增平行 FusionDetection2D/Array 和 SelectFusionROI。所有旧 msg/srv 字节及 MD5 不变。
- 桥接通过 XD 已有 measurement_source 区分 detector / visual_tracker，原 vision/tracked 路径仍可用。
- 补未知 ROI 辅助期限、弱框仅维持、不隐藏未绑定临时 ID 框、非零点击原帧/未来时间校验。
- 60 项单元/边界回归通过；3 项融合桥接、2 项旧桥接 ROS 集成测试通过；Release 和 install 构建成功。
- 真实 train7.pt / CUDA / KCF 探针通过：39 帧融合，公开 ID 不变，重复选中框为 0，selected 和原像素叠框有效。
- UE 时序准确率、非退化性能、跨相机身份和长期坦克 ReID 尚未认证，不把静态图探针等同于场景上线验收。

具体接口、配置、部署命令见 `fusion_experiment_guide.md`；数据、失败记录与限制见 `fusion_validation_20261009.md`。
以下第 1～10 节保留为 **2026-10-08 历史交接**，其暂停、未同步、旧接口 MD5 改动和旧门限状态已被上述更新替代。

## 1. 当前决定与工作边界

用户尚未取得对方最新的 `sar_yolo_detector`，本次暂停实现和测试。
收到最新包后先核对版本和接口，再继续开发；不要直接把今天的实验副本覆盖到最新包。

- 正式包 `/home/promise/mrs_test/src/sar_yolo_detector` 未修改。
- 正式包 `/home/promise/mrs_test/src/xd_uav_track` 未修改。
- 本次只导出并修改了一份独立 `sar_yolo_detector`；没有复制第二个包。
- 副本 workspace：`/home/promise/mrs_test/sar_fusion_experiment.wlpAs3`。
- 副本源码：`src/sar_yolo_detector`。
- 基线：Gitee `feature/xd-uav-construction`，提交
  `9f7d77d5a046284c7a5efa2c58e3138bdf6ea5de`。
- 本次没有向 Gitee 提交或推送，也没有保存用户密钥到代码、文档或 Git 配置。
- 本次启动的实验 roslaunch、YOLO 节点、桥接和由其启动的 ROS master 已关闭。
- 副本属于未完成验收的实验成果，不是已经完成部署认证的版本。

## 2. 用户明确要求

### 2.1 采用哪一条算法链

以指定分支的 `launch/xd_smart_tracker_integration.launch` 及其配置、实际节点为基础。
保留 PixEagle 移植的现有 YOLO / SmartTracker 路径，不用另一分支的共享 TensorRT
入口替代它，也不把另一分支的网页 detect/track/fusion 模式管理器直接套上来。

模型固定使用 `train7.pt`。本机包内实际名称是：

`models/xd_vehicle_train7/xd_vehicle_train7.pt`

SHA-256：`8f2506e1fd9585eab0d23203f504ff0036d580327e55ef15d23bd9e46fd34b54`。
实验副本与正式包的模型字节一致。按用户当前要求走 PyTorch CUDA，不转换为 engine，
不更换权重、不重新训练。KCF/CSRT 在本方案中使用 OpenCV CPU 路径，并不是 GPU KCF。

### 2.2 原有功能必须保留

| 原模式 | 必须保留的能力 |
|---|---|
| `smart_tracker` | YOLO 检测、BoT-SORT 局部 ID、PixEagle 选择/失联处理与输出 |
| `feature_tracker` | 人工初始化 ROI，KCF/CSRT/MIL/模板等已有算法选择 |
| `color_tracker` | 人工 ROI 学习颜色；配置颜色直接搜索；颜色切换参数 |

三种模式不能被融合模式替代。应保留原启动参数、模式切换、手动选择、取消和输出语义。
新模式暂为 opt-in，不在未验收前改变原默认模式。

### 2.3 新增的融合功能

新增 `fusion_tracker` / 启动参数 `FusionTracker`：

1. 同一路图像同时具备 YOLO 检测和局部特征跟踪证据。
2. 判断两个框属于同一实体后，输出一个最终融合框。
3. 框、类别、公开 ID、中心点和叠框图像对应同一原始图像坐标。
4. YOLO 短时漏检时，合格局部跟踪可维持位置，但不得伪装成新 YOLO 检测。
5. 局部跟踪漂移时，可信 YOLO 可校正位置、尺寸或重新初始化。
6. 明显冲突、相似候选歧义、完全遮挡时不能强行取平均或保持旧实线框假装锁定。
7. BoT-SORT 局部 ID 变化不应直接改变已选目标公开 ID。
8. 类别单帧波动不应直接引起 ID 变化或丢锁。

当前实验范围是“全图 YOLO + 一个选中目标的局部融合”。没有实现对每个检测目标
各建一套 KCF 的多目标融合池。收到最新包后需再次明确是否需要扩展到全目标融合；
若扩展，应有最大活跃 SOT 数、调度优先级、CPU 预算和有界身份表。

## 3. 基线源码核查结果

入口加载 `config/smart_tracker.yaml`、`config/smart_tracker_xd_vehicle.yaml`，
然后包含 `launch/smart_tracker.launch` 和 `launch/xd_detection_bridge.launch`。

### 3.1 并非现成融合

`scripts/smart_tracker_node.py` 的图像处理分支互斥：手动跟踪处理完后跳过 YOLO。
YOLO 模式也不会同时推进手动 KCF。因此新增融合必须接入真实处理路径，而不仅是
增加一个 launch 参数、输出模式名称或把两个话题简单合并。

### 3.2 配置及语义问题

- 基线 YOLO 门限 0.60，允许类别仅 `[0,1,2]`。
- train7 实际类别包含 `car=0/ar-car=1/tank=2/m142=3`；若需要 m142，必须明确更新
  类别允许列表。融合不能恢复被上游类别过滤掉的目标。
- KCF 更新成功时 `confidence=1.0` 是代码人工赋值，不是正确定位的统计概率。
- 原手动目标默认类别 2、固定 ID 900000000；该类别是配置标签，不证明 YOLO 识别为坦克。
- 原 `RegionOfInterest` 没有图像时间戳，不能安全代表延迟网页上的原帧框选。
- 原 Kalman / 失联窗口仍有固定每帧步长、按处理帧计数的部分；更换主机/FPS 后时间意义会变化。
- 模型 README 中的“已回退 train”历史说明与 launch 的 train7 默认值不一致。
- 基线不是另一分支的共享 TensorRT + 高频像素辅助生产栈，不应混淆两者的性能记录。

## 4. 推荐融合算法与身份原则

### 4.1 同帧关联后再融合

YOLO 框和局部框统一为原图像素 `(x,y,w,h)`。检查时间、边界、运动位置、尺寸、
类别父组、外观质量后再计算一致性。不能仅凭“靠近”或者“局部 ID 相同”跳过门控。
不同相机的原始像素框不能直接融合，跨相机需要标定、时间同步及共同坐标/可靠身份证据。

### 4.2 质量裁决

| 情况 | 正确行为 |
|---|---|
| 两个合格框一致 | 质量加权融合，检测负责尺寸和类别证据 |
| YOLO 漏检、局部可靠 | 有期限的视觉辅助位置，保持公开 ID |
| 局部失败、YOLO 可信唯一 | 检测恢复，必要时重初始化局部跟踪 |
| 两框严重冲突或相似候选竞争 | ambiguous，触发重检测/确认，不盲目平均 |
| 两路都失败 | 短时真实运动预测，随后 lost |

KCF 布尔成功不是概率；HSV/纹理只是弱线索，不等于经过车辆域训练的深度 ReID。
两个算法使用相同图像，其误差可能相关，不能把一致当成两份独立身份认证。
避免每帧重建局部跟踪器，也避免检测框与局部框分别发布造成同目标双框。

### 4.3 身份与类别

初始局部轨迹 ID、当前局部轨迹 ID、公开锁定 ID 在逻辑上分开。
重获必须连续确认同一个新局部候选，并比较最佳与次佳差距。
两个同时可见的不同局部轨迹默认不能拼为同一实体。
人工未知 ROI 不默认赋 tank 类别；后续得到可信识别才补充类别。
保持类别稳定不等于永远冻结第一次错误类别，后续正式版可增加类别证据累积和迟滞。

## 5. 今天已经实现的实验内容

以下是“代码已写入副本”，不代表最新完整版本均已验收：

- 新模块 `python/sar_yolo_detector/box_fusion.py`：同目标筛选、框一致性融合、
  时间差运动预测、弱外观校验、尺度漂移门控、短时视觉辅助和失联状态。
- 同一候选连续确认后更新内部局部 ID，保留公开锁定 ID。
- 在原 `smart_tracker_node.py` 接入第四模式，保留前三种处理分支。
- 未选目标时继续全图 YOLO，选中后输出最终融合框，其他目标仍输出 YOLO 框。
- 叠框图与数值输出共用最终框，保持原图分辨率和左上角原点的框中心坐标。
- 新 `SelectFusionROI.srv`：原帧 stamp/seq/frame_id 先校验，再做 ROI 匹配。
- 原图缓存最多两帧、默认 24 MiB、选框最大年龄默认 0.75 秒；粗大 ROI 优先收紧为
  包含的唯一可信检测框，多目标 ROI 拒绝。
- 辅助、实测、预测、失联/歧义分离；预测框不作为真实检测数组输出。
- `~fusion_evidence` JSON 记录来源、公开/局部 ID、真实检测时间和耗时。
- 桥接节点传递 `measurement_source`；下游不支持时拒绝把视觉辅助伪装成检测。
- 输入尺寸改变时清理融合选择/缓存及 BoT-SORT/GMC 缓存的处理已写入，待补回归。
- 实验工作区已完成单线程 Release 构建；不启用 TensorRT C++ backend。
- 专用虚拟环境补齐 OpenCV contrib 4.10.0 KCF/CSRT，正式 GPU 环境未改。
- 说明文档 `docs/fusion_experiment_guide.md` 已写入，最终还需与最新包逐项校订。

## 6. 实验中遇到的问题及状态

### 6.1 GPU 环境没有实际 KCF/CSRT

原 `.venv-sar-gpu` 的 OpenCV 5.0.0 没有这些 API，也没有 legacy 实现。
仅配置 kcf 不代表实际执行 kcf，旧兼容代码可能降为模板。

已在实验环境安装 contrib 并验证真实 KCF/CSRT API。融合默认禁止静默模板回退。
未来主机须使用实际执行 Python 的环境检查，不能只看系统 Python 的 OpenCV。

### 6.2 导出模型权限触发安全门禁

Gitee `git archive` 导出文件带 group write 权限，模型完整性检查拒绝加载。
已只收紧实验副本模型权限；模型字节及 SHA 不变，正式模型未动。

### 6.3 Numpy 坐标类型导致 OpenCV 绘图报错

融合产生的 NumPy 标量不能在部分 OpenCV 绑定中直接作为矩形坐标。
已显式转为 Python int，随后 ROS 消息边界测试通过。

### 6.4 ROS Python 路径及测试方式

把源码 python 路径放在 catkin 生成路径前，会遮蔽 `sar_yolo_detector.msg/srv`。
应 source 隔离 devel，由 catkin relay 组合源码和消息路径。
第一次通配发现还误包含了需要运行外部节点的旧 late_fusion rostest；其失败不是
新增 box_fusion 单元测试失败。后续改为明确的测试模块列表。

### 6.5 真实 ROS 图像探针未通过

启动日志已确认：train7.pt、device=cuda、fusion_tracker 节点就绪。
但两次探针分别使用 car/4/47.png 和 train4/train_batch0.jpg，在原 0.60 门限下没有
取得满足探针要求的稳定 YOLO 目标，因此没完成“选择后真实融合输出”的端到端通过记录。

确切原因尚未排清：可能涉及样本/类别过滤、阈值、稳定局部 ID 条件或探针流程。
不能据此断言模型全面失效，也不能把启动成功当作真实融合闭环通过。
未为让探针通过而降低生产门限或更换权重。

### 6.6 图像尺寸变化导致 BoT-SORT GMC 报错

两次探针图片尺寸不同，原 tracker 的光流金字塔历史不兼容，报 GMC assertion 并
回退 identity。已增加不重载模型的 tracker/GMC reset 及融合尺寸段重置代码。
暂停前这部分尚未完成重测，不能称为已验证修复。

### 6.7 独立调试脚本的配置错误

一次脚本使用不存在的 `DevicePreference.GPU`；正确值是 CUDA。
另一次独立 backend 脚本没有按完整 profile 提供受信模型路径配置，报“没有可信 SHA”。
这两次是调试脚本问题，不是 train7 模型 SHA 改变；正式实验 launch 后续已成功加载 CUDA。

### 6.8 接口兼容性风险尚须决策

实验给 `TrackedDetection2D.msg` 增加 `measurement_source`，会改变 ROS1 消息 MD5。
虽然既有字段保留，但旧订阅程序不能直接混用，必须重编译。
收到最新包后，应检查它是否已有来源字段，优先复用；如果仍没有，要决定扩展原消息
还是采用并行新消息/兼容适配器，不能未经确认覆盖对方正在使用的合同。

## 7. 测试记录：哪些能证明、哪些不能

| 检查 | 状态 |
|---|---|
| 早期融合核心测试 | 22 项通过，含真实 KCF 移动纹理目标 |
| 隔离构建 | 成功，`-j1 -l1`，依赖正式工作区 underlay |
| 随后实验 GPU 环境回归 | 50 项全部通过，包含核心、适配、旧手动、编码和 PixEagle 测试 |
| 真实模型启动 | 成功：train7.pt / CUDA / FusionTracker |
| 真实 ROS 选中后融合输出 | 未取得通过记录，两次探针未获取稳定目标 |
| 最后新增的同时可见保护、尺寸/GMC重置等 | 尚未对最终完整快照重新跑全套 |
| UE 视频/地图 A/B | 未执行，没有对方最新包与对应用例 |
| 不低于之前最佳性能基线 | 尚未证明 |

回归日志：实验 workspace 的 `logs/gpu_environment_regression.log`。
失败探针 JSON 和 launch 日志也保留，不删除失败记录。

## 8. 拿到最新包后需要补充的信息

1. 最新源码或 Gitee 明确提交号、实际启动 launch、最终加载的全部 YAML。
2. UE 摄像机原始 topic、编码、宽高、FPS；CameraInfo、frame_id、时间戳和 `/clock`。
3. 目标主机 GPU、显存、驱动、CUDA、Torch、Ultralytics、OpenCV 的运行时版本。
4. 单相机还是双相机，YOLO 与 KCF 是否同进程/同图像，手动框选是网页还是本机界面。
5. 必须保持兼容的输出消息、网页接口与下游节点版本。
6. 一段可重复的 UE 视频/rosbag，最好有目标框和 ID 真值，包含转弯、遮挡和相似车辆。
7. “融合只针对选中目标”还是“所有检测目标都需要融合”的明确范围。

## 9. 后续执行顺序与完成门槛

### 阶段一：版本和部署合同对齐

以最新包另建隔离副本，对比今天的修改，保留对方已有接口与三模式功能。
核对 train7 字节、类别合同、Python解释器、图像尺寸/时间和真实 GPU 执行。
启动模式参数必须尊重配置文件，不应因新默认参数意外覆盖原 Default_Tracker。

完成条件：最新 launch 能正确解析、模型可启动、三原模式独立回归通过、包路径明确。

### 阶段二：完整融合与身份门控

补齐同帧关联、质量权重、观测来源、公开 ID 连续、重获确认、粗 ROI 收紧和真失联语义。
排清真实 ROS 探针不能选到目标的原因，补尺寸/GMC 重置与最终源代码回归。

完成条件：真实 train7/PT + 实际 KCF 输出一个融合框，框坐标与画面一致，辅助不刷新
真实检测时间，拒绝歧义和错误目标；原三模式仍通过。

### 阶段三：时间与资源优化

必要时分离 latest-only 图像/局部跟踪与 YOLO 推理，避免 YOLO 的低节奏限制每帧跟踪。
这是待实现项；今天仍在原图像处理/锁内同步执行 YOLO 与局部跟踪。
保持周期全图搜索，缓存/队列/目标数有界，不重复加载模型，不盲目增加算法。

完成条件：相同主机/输入/权重下，预热后 FPS、端到端 P95、CPU/GPU、内存符合约定，
没有用更大队列把掉帧换成高延迟，也没有为了更平滑引入错误身份。

### 阶段四：UE 场景非退化验收

同一输入、同权重、同阈值比较原 YOLO、独立 KCF和融合。使用真值框/ID计算定位、
ID switch、误关联和漏框空窗，分场景报告。无真值时只称“连续性代理指标”，不冒充准确率。

完成条件：定位/身份不劣于基线，空窗改善有记录，延迟和资源不越预算；尤其不能靠
长时间保留错误框或旧框换取表面存活率。未通过前不替换正式包、不改全局默认、不上线认证。

## 10. 明确不能承诺的事项

- YOLO 从未识别到的目标不会因框融合自动得到正确类别。
- 完全遮挡、出画、错误框选无法保证框永远准确。
- 相似车辆长遮挡仅凭颜色和近邻不能证明实体相同。
- 飞行/云台剧烈运动的稳定性需要时序、标定、运动补偿和专门场景测试。
- 今天的静态/合成测试不等于 UE 实拍时序验收，不等于实机飞行验收。

结论：方向确定、实验核心已有实现，但应等最新包后做接口对齐和真正的真实场景验收，
不能把今天的中间成果当作最终可交付融合版本。
