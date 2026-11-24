# 常见问题

## RViz 或 Gazebo 未正常显示

Gazebo 负责物理与传感器仿真，RViz 展示 ROS 中的地图、模型、TF 和路径。一个窗口打开不代表另一侧的数据已经正常。

启动脚本默认使用软件渲染。虚拟机负载过高时先用 `gui:=false rviz:=false` 验证数据链，再单独打开可视界面。

## 话题存在但收不到数据

单独的检查终端需要加载同一环境：

```bash
source /opt/ros/humble/setup.bash
source ~/nav_ws/install/setup.bash
source ~/tf_fix_ws/install/setup.bash
export ROS_DOMAIN_ID=25 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 topic info /scan --verbose
timeout 10s ros2 topic echo /scan sensor_msgs/msg/LaserScan --once --field header --qos-reliability best_effort
```

发布者数量不等于已经产生传感器数据。还需检查仿真是否推进、Gazebo 世界名与桥接话题是否一致，以及仿真端是否有激光消息。

## 仿真看似停止

检查 `/clock` 是否连续更新，而不是只读取一次时钟值。不要同时运行两套使用同一 partition 和 ROS domain 的仿真；不要在导航期间重置仿真时间。

## 定位或地图异常

建图模式与定位模式应分开运行，避免 SLAM 和 AMCL 同时发布 `map → odom`。正常 TF 链包含 `map → odom → base_footprint → laser_link`。

配置启用了仿真时间；单个节点使用系统时间可能导致 TF 消息过滤失败。偶发启动等待与持续缺失变换需要区分。

## tf2 库搜索路径提示

当前依赖包含系统 tf2 与独立修复覆盖库，CMake 可能提示搜索路径约束。先按 [安装指南](installation.md) 加载覆盖环境，再检查实际进程加载的库，而不是仅凭目录存在判断修复生效。补丁来源见 [第三方说明](../THIRD_PARTY_NOTICES.md)。

## 目标被拒绝

带预检入口会拒绝非空闲状态、缺失新鲜里程计或没有合规路径的任务。窄通道被额外间距策略拒绝可能是预期结果；降低权重也不会解除硬间距约束。

该入口假设单调用者，不阻止另一个 ROS 节点直接发送速度或导航请求。

## 旧文档与当前命令不同

`docs/archive/` 保存历史配置和排查过程。主题文档描述当前公开入口，归档记录用于解释变更和追溯实验，不应将不同轮次参数混合使用。提交 Issue 时请提供版本、地图、参数文件、启动命令和报错日志。
