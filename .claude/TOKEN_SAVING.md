# 省 tokens 铁律

1. 不做无意义的重复验证 — 编译/语法/roslaunch 各测一次
2. Bash > Edit — 批量 sed 替代逐个文件的 Edit
3. grep > Read — 定位优先，只在必须理解逻辑时 Read
4. 方案确认后再动手 — 避免反复修反复测
5. 工具退出码即验证 — 不 echo "✅" 确认
6. 只读需要的行 — `Read offset= limit=` 而非整个文件
7. TodoWrite 只在大阶段切换时更新

# 何时重启

**当回复开始变慢、出现循环输出、或你意识到上下文超过 ~100k tokens 时：**

1. 在 `temp/` 写状态文件
2. 告诉用户新开对话
3. 新对话第一条：读 CLAUDE.md + temp 状态文件

