# 开发记录归档

此目录保留项目开发过程中的环境、模型、参数与验收记录。文件名中的 M1～M8 是历史阶段标识，不是当前文档的功能分类。部分记录包含当时的工作目录、已关闭进程或已被后续配置替代的结论。

当前安装与运行方式请从 [主题文档](../README.md) 开始。原始数据目录和采集编号保持原样，便于核对哈希、实验脚本与修复前后证据。

| 主题 | 记录 |
|---|---|
| 环境与参考项目 | [环境验证](development/ROS2-Nav2-M1环境搭建与验收.md)、[参考项目接入](development/BCR-Bot接入记录.md)、[开源项目选型](development/开源项目选型.md) |
| 模型与仿真 | [模型说明](development/ROS2-Nav2-M3自建模型与验收.md)、[仿真验证](development/ROS2-Nav2-M4自建小车仿真与验收.md) |
| 建图与导航基线 | [建图记录](development/ROS2-Nav2-M5自建小车建图与验收.md)、[定位与控制器比较](development/ROS2-Nav2-M6自建小车定位导航与验收.md) |
| A* 实现 | [实现与初始验收](development/ROS2-Nav2-M7自研AStar规划器与验收.md)、[合成场景对照](development/ROS2-Nav2-M7针对性场景对照.md) |
| 跟踪与门控 | [跟踪排查](development/ROS2-Nav2-M7跟踪卡顿排查与修复.md)、[反向通行修复](development/ROS2-Nav2-M7反向通行卡顿修复与连续验收.md)、[通道与门控](development/ROS2-Nav2-M7通道宽度与任务门控验收.md) |
| 定位与对照 | [定位校准](development/ROS2-Nav2-M7累计定位偏差排查与校准.md)、[闭环对照](development/ROS2-Nav2-M7校准配置回归与规划器对照.md) |
| 交付验证 | [原始评测与交付记录](development/ROS2-Nav2-M8评测与项目交付.md) |

归档中的运行日志可能只包含于对应版本的完整发布包，不随 Git 源码仓库提供。阶段汇总不替代原始测量，不将不同地图、控制器或定位配置的样本合并为同一组对照。
