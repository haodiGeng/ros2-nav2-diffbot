# 使用指南

以下命令在完成 [安装](installation.md) 的 Ubuntu 环境执行。导航演示、建图和自动实验应分别运行，避免同时启动多套仿真。

## 导航演示

终端 A：

```bash
source /opt/ros/humble/setup.bash
source ~/nav_ws/install/setup.bash
ros2 run my_robot_navigation start_calibrated_navigation.sh gui:=true
```

该入口加载解析示例地图、仿真校准参数、Gazebo 和 RViz，初始机器人静止。启动脚本会设置 domain25、Cyclone DDS 和独立 Gazebo partition。

需要节省显示资源时：

```bash
ros2 run my_robot_navigation start_calibrated_navigation.sh gui:=false rviz:=false
```

## 发送带预检的导航目标

等待导航系统激活后，在终端 B 执行：

```bash
source /opt/ros/humble/setup.bash
source ~/nav_ws/install/setup.bash
ros2 run my_robot_navigation start_safe_navigation.sh 0 2.6
```

命令格式为 `start_safe_navigation.sh X Y [YAW]`，位置单位为米，朝向单位为弧度，省略时为 0。成功后可执行：

```bash
ros2 run my_robot_navigation start_safe_navigation.sh 0 0
```

入口检查机器人空闲、里程计新鲜和路径可达性，然后使用带额外净间距的规划器及无运动恢复行为树执行任务。其作用范围是该入口内的单调用者任务，不是全系统急停或多客户端仲裁器。

| 退出码 | 含义 |
|---|---|
| 0 | 导航成功 |
| 2 | 路径预检拒绝，未发送导航目标 |
| 1 | 已发送的导航未成功，或运行错误 |

RViz 的普通 Nav2 Goal 不等同于带预检入口。测试期间只选择一个运动指令来源，不同时使用目标按钮、其他 action 客户端和键盘遥控。

## 建图

结束已有导航后，启动 SLAM 模式：

```bash
source /opt/ros/humble/setup.bash
source ~/nav_ws/install/setup.bash
ros2 run my_robot_navigation start_mapping.sh gui:=true
```

该入口使用 SLAM Toolbox，由 SLAM 发布 `map → odom`；导航模式由 AMCL 发布这一变换，两者不应同时工作。建图还需要运动输入与充分场景覆盖，本命令不会自动完成地图采集。保存地图、原始验证条件和已知边界见 [建图开发记录](archive/development/ROS2-Nav2-M5自建小车建图与验收.md)。

仓库保留的实测建图地图存在局部墙线间断；导航演示默认使用与仿真几何一致的解析地图。不能将解析地图上的评测结果直接当作实测地图质量的证明。

## 配置选择

| 配置文件 | 用途 |
|---|---|
| `nav2_amcl_sim_calibrated.yaml` | 示例场景中已测试的仿真校准配置 |
| `nav2_astar_params.yaml` | 自研 A*、Rotation Shim+DWB 与间距门控配置 |
| `nav2_navfn_comparison.yaml` | NavFn 对照配置 |
| `nav2_params.yaml` | 保留的 NavFn+DWB 默认基线 |

配置位于 `src/my_robot_navigation/config/`。校准入口还设置驱动有效轮距比例为 0.995，AMCL alpha1～alpha4 为 0.001；这些参数针对当前仿真，未作为实物校准结果验证。

## 停止运行

任务到点停车后，在启动仿真的终端 A 按 Ctrl+C，等待进程退出。需要中断终端 B 中的任务时，先在该终端按 Ctrl+C，再退出仿真。不要在导航过程中拖动机器人、重置仿真时间，或同时启动另一套定位节点。

GUI、时间和话题异常见 [常见问题](troubleshooting.md)。
