# navigation2 局部规则

- 本仓库直接位于 `nav2_ws/src/navigation2/`，同时是唯一源码与工作空间构建入口；不存在额外构建副本，只改任务涉及文件。
- 修改源码、同步、构建或测试前读取 `/home/byd/Documents/zpy_ws/project/nav2_demo/agent_rules/ros-source-workspace.md`；修改 `.cpp` 前再读取 `/home/byd/Documents/zpy_ws/project/nav2_demo/agent_rules/cpp-layout.md`。
- 提交前检查暂存路径：路径中任一级目录（包括仓库根目录下的目录）以 `.` 开头时，该目录内文件不得暂存或提交，即使已被 Git 跟踪；普通点文件（如 `.gitignore`）不受此条限制。误暂存时只从暂存区撤下，保留工作树；不得据此删除历史文件或改写提交历史。
- 用户说“git提交”或“上传nav2到我的git”时，只操作本仓库；提交／推送前读取 `/home/byd/Documents/zpy_ws/project/nav2_demo/agent_rules/git-workflows.md`。普通 push 已授权，force-push、改历史、tag、release 仍须确认；不得操作外层、`learningnav2/` 或工作空间其他目录。
