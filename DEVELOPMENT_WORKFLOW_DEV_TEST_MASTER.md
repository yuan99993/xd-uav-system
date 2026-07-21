# xd-uavsystem 开发者与管理者工作流程

本文档规定本项目使用 Git + Gitee + ROS1 的协作方式。

当前采用三条长期分支：

~~~text
feature/* → dev → test → master
~~~

含义：

- feature/*：开发者编写功能。
- dev：开发集成分支，接收已经完成自测的功能。
- test：测试、联调和验收分支。
- master：稳定版本和正式发布分支。

---

## 一、Git 的整体结构

一次代码修改会经过以下几个区域：

~~~text
工作区 Working Tree
    ↓ git add
暂存区 Staging Area / Index
    ↓ git commit
本地 Git 仓库 Local Repository
    ↓ git push
远程 Gitee 仓库 Remote Repository
~~~

### 1. 工作区

工作区就是你实际看到和修改的文件，例如：

~~~text
src/xd_demo_cpp/src/heartbeat_node.cpp
~~~

此时修改还没有形成 Git 版本。

### 2. 暂存区

执行 git add 后，文件进入暂存区。

暂存区表示：

> 本次提交准备包含哪些修改。

因此不建议一开始无条件执行 git add .，应先使用 git status 和 git diff 检查内容。

### 3. 本地仓库

执行 git commit 后，修改被保存为本地提交。

本地提交只存在于当前电脑，还没有上传 Gitee。

### 4. 远程仓库

执行 git push 后，本地提交才会上传到 Gitee。

远程仓库通常命名为 origin：

~~~text
origin = Gitee 远程仓库地址
~~~

查看远程仓库：

~~~bash
git remote -v
~~~

### 5. 分支

分支本质上是指向某个提交的可移动指针。

~~~text
master
   ↑
某个稳定提交

dev
   ↑
开发集成提交

feature/xxx
   ↑
开发者自己的功能提交
~~~

每个分支可以独立开发，最后通过 Pull Request 合并。

### 6. HEAD

HEAD 表示当前正在工作的分支或提交。

查看当前分支：

~~~bash
git status --short --branch
~~~

输出中的当前分支就是 HEAD 所在的分支。

### 7. Pull Request

Pull Request 不是普通的 git pull 命令。

Pull Request 表示：

> 请求把一个分支的代码审核并合并到另一个分支。

本项目中的 Pull Request 方向：

~~~text
feature/xxx → dev
dev → test
test → master
~~~

---

## 二、仓库目录结构

推荐的 ROS 工作空间结构：

~~~text
xd-uavsystem-test/
├── .git/                         Git 本地仓库数据，不手动修改
├── .gitee/                       Gitee Issue 和 PR 模板
├── .gitignore                    Git 忽略规则
├── README.md                     项目说明
├── DEVELOPMENT.md                开发流程说明
├── src/                          ROS 功能包源码
│   ├── xd_demo_cpp/              C++ 示例功能包
│   │   ├── package.xml
│   │   ├── CMakeLists.txt
│   │   ├── src/
│   │   ├── launch/
│   │   ├── config/
│   │   └── README.md
│   ├── xd_msgs/                   自定义消息、服务和动作
│   ├── xd_core/                   基础库和公共接口
│   ├── xd_estimation/             状态估计
│   ├── xd_control/                飞行控制
│   ├── xd_planner/                路径规划
│   └── xd_bringup/                启动文件和整机参数
├── build/                         catkin 编译生成，不提交
├── devel/                         catkin 环境生成，不提交
└── install/                       安装空间生成，不提交
~~~

build、devel 和 install 是编译生成目录，必须通过 .gitignore 排除。

---

## 三、分支职责

| 分支 | 作用 | 谁可以推送 | 谁负责合并 |
|---|---|---|---|
| feature/* | 单个功能开发 | 对应开发者 | 管理者审核后合并到 dev |
| bugfix/* | 普通问题修复 | 对应开发者 | 管理者审核后合并到 dev |
| dev | 开发集成 | 管理者通过 PR 合并 | 管理者 |
| test | 测试和联调 | 管理者通过 PR 合并 | 管理者 |
| master | 稳定发布 | 管理者通过 PR 合并 | 管理者 |
| hotfix/* | 正式版本紧急修复 | 对应开发者或管理员 | 管理者 |

建议 dev、test、master 都设置为保护分支：

- 禁止开发者直接推送。
- 只能通过 Pull Request 合并。
- 禁止强制推送。
- 禁止普通成员删除分支。
- master 合并前必须有测试结果。
- test 合并到 master 前必须完成验收。

---

## 四、管理员账号的职责

管理员账号负责：

1. 添加仓库成员。
2. 设置开发者角色。
3. 配置分支保护。
4. 审核 Pull Request。
5. 合并 feature/* 到 dev。
6. 合并 dev 到 test。
7. 合并 test 到 master。
8. 创建版本标签。
9. 处理紧急修复。
10. 配置 CI、代码审查和仓库规则。

管理员账号不建议用于日常功能开发。

这样可以保证：

~~~text
开发者负责写代码
管理员负责审核和合并
测试人员负责验证
~~~

---

## 五、开发者第一次配置

### 1. 克隆仓库

如果还没有工作目录：

~~~bash
git clone -b dev <仓库地址> ~/xd-uavsystem-test
cd ~/xd-uavsystem-test
~~~

例如：

~~~bash
git clone -b dev https://gitee.com/xd-uavsystem/xd-uavsystem-test.git ~/xd-uavsystem-test
~~~

git clone 的作用：

- 下载远程仓库的代码。
- 创建本地 Git 仓库。
- 自动配置远程地址 origin。
- 默认切换到指定分支dev。

### 2. 查看远程地址

~~~bash
git remote -v
~~~

作用：确认当前目录连接的是正确的 Gitee 仓库。

### 3. 设置提交信息

~~~bash
git config user.name "开发者账号名称"
git config user.email "开发者账号邮箱"
~~~

作用：设置提交记录中的作者信息。

建议使用仓库级配置，不要随意修改全局配置：

~~~bash
git config --local user.name "开发者账号名称"
git config --local user.email "开发者账号邮箱"
~~~

### 4. 检查当前状态

~~~bash
git status --short --branch
~~~

作用：

- 查看当前分支。
- 查看是否有未提交修改。
- 查看新增、修改和删除的文件。

---

## 六、开发者日常开发流程

### 第一步：同步最新 dev

~~~bash
cd ~/xd-uavsystem-test
git fetch origin
git switch dev
git pull --ff-only origin dev
~~~

命令解释：

- git fetch origin：只获取远程最新信息，不修改当前文件。
- git switch dev：切换到 dev 分支。
- git pull --ff-only origin dev：更新本地 dev，也会同步更新那个分支的文件过来，只有能够直接快进时才合并。

如果本地还没有 dev：

~~~bash
git switch -c dev --track origin/dev
git switch       切换分支
-c dev           创建一个名为 dev 的本地分支
--track          建立跟踪关系
origin/dev       以远程 dev 分支作为创建起点
~~~

### 第二步：创建功能分支（做新功能要创建新的分支）

~~~bash
git switch -c feature/xd-demo-cpp origin/dev
~~~

作用：

- 从最新的 origin/dev 创建功能分支。
- 新分支名为 feature/xd-demo-cpp。
- 自动切换到新分支。

分支命名建议：

~~~text
feature/lidar-driver
feature/estimation-manager
feature/xd-demo-cpp
bugfix/imu-time-sync
bugfix/planner-crash
~~~

不要直接在 dev、test、master 上修改代码。

### 第三步：编写代码

ROS 功能包放在 src 目录：

~~~text
src/xd_demo_cpp/
├── package.xml
├── CMakeLists.txt
├── src/
├── launch/
├── config/
└── README.md
~~~

一个功能包应当做到：

- package.xml 依赖声明完整。
- CMakeLists.txt 能够独立编译。
- launch 文件能够启动节点。
- README 中有编译和运行方法。
- 不依赖开发者个人目录。
- 不把密码、Token 或本地路径写进仓库。

### 第四步：编译 ROS 工作空间

在仓库根目录执行：

~~~bash
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
~~~

命令解释：

- source /opt/ros/noetic/setup.bash：加载 ROS Noetic 环境。
- rosdep install：根据 package.xml 安装缺少的 ROS 依赖。
- catkin_make：编译 src 下的所有 catkin 功能包。
- source devel/setup.bash：让当前终端找到本工作空间生成的 ROS 包和节点。

注意：catkin_make 必须在包含 src 目录的工作空间根目录执行。

### 第五步：运行功能测试

以 xd_demo_cpp 为例：

~~~bash
roslaunch xd_demo_cpp heartbeat.launch
~~~

另开一个终端：

~~~bash
source /opt/ros/noetic/setup.bash
source ~/xd-uavsystem-test/devel/setup.bash
rostopic echo /xd_demo_cpp/heartbeat
~~~

开发者至少需要记录：

- 使用的 ROS 版本。
- 编译命令。
- 启动命令。
- 话题、服务或动作测试结果。
- 硬件或仿真环境。
- 已知问题。

### 第六步：查看修改

~~~bash
git status
git diff
~~~

git status 查看修改文件。

git diff 查看代码具体变化。

确认只包含本次功能的修改，不要把以下内容加入提交：

~~~text
build/
devel/
install/
*.bag
*.log
.env
私钥
Token
个人配置
~~~

### 第七步：添加暂存文件

推荐只添加指定功能包：

~~~bash
git add src/xd_demo_cpp
~~~

如果同时修改了文档：

~~~bash
git add DEVELOPMENT.md
~~~

查看暂存区：

~~~bash
git diff --cached
~~~

git diff --cached 用于确认下一次提交真正包含的内容。

### 第八步：创建本地提交

~~~bash
git commit -m "feat: add ROS heartbeat demo package"
~~~

git commit 的作用：

- 把暂存区内容保存为一个本地版本。
- 生成一个提交编号。
- 不会自动上传 Gitee。

提交类型建议：

~~~text
feat: 新增功能
fix: 修复问题
docs: 修改文档
refactor: 重构代码
test: 增加测试
build: 修改编译配置
chore: 其他维护工作
~~~

### 第九步：推送功能分支

~~~bash
git push -u origin feature/xd-demo-cpp
~~~

作用：

- 将本地功能分支推送到远程 Gitee。
- -u 建立本地分支和远程分支的跟踪关系，否则后续每次推送需要指明分支。
- 后续在同一分支继续提交时，只需要执行 git push。

注意：

~~~text
git push feature 分支
不会自动合并到 dev、test 或 master。
~~~

### 第十步：创建 Pull Request 到 dev

在 Gitee 创建：

~~~text
源分支：feature/xd-demo-cpp
目标分支：dev
~~~

PR 描述建议包含：

~~~text
## 修改内容
- 增加 heartbeat_node
- 增加 launch 和参数文件

## 测试结果
- catkin_make：通过
- roslaunch：通过
- rostopic echo：可以收到消息

## 影响范围
- 新增功能包，不影响已有模块
~~~

---

## 七、开发者处理评审意见

管理员提出修改意见后，开发者继续在原功能分支修改,可以在已关闭的PR内查询自己的PR查看评论意见：

~~~bash
git status
git add <修改文件>
git commit -m "fix: address review comments"
git push
~~~

原 Pull Request 会自动更新，不需要创建新的 PR。


---

## 八、管理员合并 feature 到 dev

管理员检查 PR：

1. 源分支是否为正确的 feature 或 bugfix 分支。
2. 目标分支是否为 dev。
3. 是否通过编译。
4. 是否有测试说明。
5. 是否包含敏感信息。
6. package.xml 和 CMakeLists.txt 是否完整。
7. 是否影响其他功能包。
8. 是否需要补充文档或测试。

通过后在 Gitee 合并：

~~~text
feature/xd-demo-cpp → dev
~~~

如果评审不通过，管理员在 PR 中提出修改意见，不直接修改开发者代码。

---

## 九、管理员将 dev 交给测试

当 dev 中积累了一个完整的可测试版本后，管理员创建：

~~~text
源分支：dev
目标分支：test
~~~

这个 PR 表示：

> 开发集成版本准备进入测试和验收阶段。

合并后，测试人员或管理员更新本地 test：

~~~bash
git fetch origin
git switch test
git pull --ff-only origin test
~~~

如果本地没有 test：

~~~bash
git switch -c test --track origin/test
~~~

然后编译：

~~~bash
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
~~~

再执行完整测试：

~~~bash
roslaunch <功能包名> <启动文件>.launch
~~~

测试报告至少记录：

- 测试时间。
- 测试人员。
- 代码提交编号。
- ROS 版本。
- 硬件或仿真环境。
- 执行的命令。
- 测试结果。
- 失败日志。
- 已知问题。

---

## 十、测试通过后合并到 master

测试全部通过后，管理员创建：

~~~text
源分支：test
目标分支：master
~~~

管理员合并后同步本地 master：

~~~bash
git fetch origin
git switch master
git pull --ff-only origin master
~~~

创建稳定版本标签：

~~~bash
git tag -a v0.1.0 -m "first runnable version"
git push origin v0.1.0
~~~

标签的作用：

- 给某个稳定提交命名。
- 便于回溯和下载指定版本。
- 不会修改代码。
- 不会自动创建 Release，是否创建 Release 由管理员决定。

版本号建议：

~~~text
v0.1.0：初始可运行版本
v0.2.0：新增兼容功能
v0.2.1：问题修复
v1.0.0：第一个正式版本
~~~

---

## 十一、测试人员或管理员更新本地代码

已经存在本地仓库时，不需要反复 clone。

更新 dev：

~~~bash
git fetch origin
git switch dev
git pull --ff-only origin dev
~~~

更新 test：

~~~bash
git fetch origin
git switch test
git pull --ff-only origin test
~~~

更新 master：

~~~bash
git fetch origin
git switch master
git pull --ff-only origin master
~~~

只有在需要验证一个全新的干净环境时，才重新 clone：

~~~bash
git clone -b test <仓库地址> ~/xd-uavsystem-test-clean
cd ~/xd-uavsystem-test-clean
source /opt/ros/noetic/setup.bash
catkin_make
~~~

---

## 十二、紧急修复流程

正式版本出现严重问题时，从 master 创建 hotfix：

~~~bash
git fetch origin
git switch -c hotfix/fix-critical-bug origin/master
~~~

修复并测试：

~~~bash
git add <修改文件>
git commit -m "fix: resolve critical bug"
git push -u origin hotfix/fix-critical-bug
~~~

创建 PR：

~~~text
hotfix/fix-critical-bug → master
~~~

管理员合并并发布补丁版本后，还需要把修复同步到 dev 和 test。

推荐顺序：

~~~text
hotfix/* → master
master → test
test → dev
~~~

这些同步操作仍然通过 PR 完成。

---

## 十三、冲突处理

如果开发期间 dev 已经发生变化，可以将最新 dev 合并到自己的功能分支：

~~~bash
git fetch origin
git switch feature/xd-demo-cpp
git merge origin/dev
~~~

如果出现冲突：

1. 打开冲突文件。
2. 手动保留正确代码。
3. 删除冲突标记。
4. 重新检查代码。
5. 添加已解决文件。

~~~bash
git add <已解决的文件>
git commit -m "merge: sync latest dev"
git push
~~~

不要在共享分支上使用强制推送：

~~~text
git push --force
~~~

---

## 十四、禁止事项

- 不直接向 dev、test、master 推送。
- 不使用 git push --force 修改共享分支。
- 不提交密码、Token、私钥和个人配置。
- 不提交 build、devel、install 和 ROS 日志。
- 不提交 rosbag、地图、模型等大文件，除非项目明确规定。
- 一个 PR 不混入多个无关功能。
- 没有自测结果时不创建正式 PR。
- 没有测试通过记录时不合并 test 到 master。
- 不随意删除远程分支。
- 不直接在管理员账号中进行日常开发。

---

## 十五、完整操作速查

### 开发者

~~~bash
cd ~/xd-uavsystem-test

获取最新文件
git fetch origin
git switch dev
git pull --ff-only origin dev 获取最新文件

切换开发分支feature/my-feature
git switch -c feature/my-feature origin/dev

自己电脑测试效果
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash

推送到开发分支feature/my-feature
git status
git diff
git add <功能文件>
git diff --cached
git commit -m "feat: add my feature"
git push -u origin feature/my-feature
~~~

然后创建：

~~~text
PR推送feature/my-feature到仓库dev分支
feature/my-feature → dev
~~~

### 管理员进入测试

~~~text
dev → test
~~~

测试环境更新：

~~~bash
git fetch origin
git switch test
git pull --ff-only origin test
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
~~~

### 管理员发布稳定版本

~~~text
test → master
~~~

发布：

~~~bash
git switch master
git pull --ff-only origin master
git tag -a v0.1.0 -m "first runnable version"
git push origin v0.1.0
~~~

最终原则：

~~~text
开发者提交功能
管理员审核代码
dev 做集成
test 做验证
master 做发布
~~~

