# ROS 2 / Nav2 自主导航项目 M1 环境搭建与验收

M1 已验收。ROS 2 发布订阅正常，Gazebo 图形界面能启动且仿真时钟能进入 ROS，RViz2 主窗口正常显示，`~/nav_ws` 空工作空间构建成功。

## 环境与选型

| 项目 | 实际使用情况 |
|---|---|
| 宿主与虚拟机 | Windows 上的 VMware，虚拟机配置为 4 核 vCPU、8 GB 内存、NAT、启用 3D 加速 |
| 来宾系统 | Ubuntu 22.04.5 LTS，初检时根分区剩余 15 GB |
| ROS 2 | 已有 Humble 安装，位于 `/opt/ros/humble` |
| 仿真与可视化 | Gazebo Fortress、`ros_gz_bridge`、RViz2 |
| 工作空间 | Ubuntu 用户目录下的 `~/nav_ws` |

沿用了已有的 Ubuntu 和 Humble。仿真采用 Fortress，因此附件中面向 Gazebo Classic 的 `gazebo_ros` 启动命令和驱动插件配置，要换成与 Fortress 兼容的方案。[Gazebo 与 ROS 官方兼容表](https://gazebosim.org/docs/harmonic/ros_installation/)

## 操作记录与用途

| 工作 | 实际操作与结果 | 为后续做的准备 |
|---|---|---|
| 检查系统和工具 | 核对 Ubuntu 版本、磁盘空间和 Humble 安装目录。确认 `python3-colcon-common-extensions` 已安装 | 确定后续安装包和接口版本，建立构建环境 |
| 检查软件源并补依赖 | 最初筛选关键 ROS 包时只看到 `rviz2`。用 `apt-cache policy` 确认四个待装包在清华镜像的 `jammy` 源有候选版本，随后补装 | 为仿真桥接、SLAM 建图和 Nav2 启动准备依赖 |
| 启动空世界并桥接时钟 | Gazebo 窗口打开，ROS 端 `/clock` 返回 `sec` 和 `nanosec` | M2 的 SLAM、定位、导航和 RViz2 可通过 `use_sim_time` 使用同一仿真时间 |
| 验证 ROS 2 通信 | `demo_nodes_cpp` 的 listener 收到 `Hello World: 15` | 确认虚拟机内节点发现和发布订阅正常，后续可据此排查小车话题 |
| 初始化工作空间 | 创建 `~/nav_ws/src` 并运行 `colcon build --symlink-install`，输出 `Summary: 0 packages finished [0.39s]` | 为机器人描述、启动配置和 C++ 规划器建立工作空间。加入首个实际包后再验证编译与安装 |
| 打开 RViz2 | 用户确认主窗口正常显示 | M2 将用它检查激光、TF、地图和路径，并设置初始位姿和导航目标 |
| 检查 SSH | Ubuntu SSH 服务为 `active`，Windows 能连接端口，账户登录授权尚未验证 | 可选的远程操作方式。M1 的验收命令在 Ubuntu 终端手动执行 |

## 复现命令

以下命令在 Ubuntu 执行。每个新终端先加载 Humble 环境。

```bash
source /opt/ros/humble/setup.bash
```

新环境补装依赖时执行：

```bash
sudo apt update && sudo apt install \
  ros-humble-ros-gz ros-humble-navigation2 \
  ros-humble-nav2-bringup ros-humble-slam-toolbox \
  python3-colcon-common-extensions
```

仿真时钟检查使用三个终端。`[` 指定由 Gazebo 向 ROS 单向桥接。

```bash
# 终端 A：启动并运行空世界
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="-r empty.sdf"

# 终端 B：桥接时间
ros2 run ros_gz_bridge parameter_bridge \
  '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'

# 终端 C：读取一条时间消息
timeout 10s ros2 topic echo /clock --once
```

发布订阅检查使用两个终端。看到 listener 日志后，用 `Ctrl+C` 停止两个节点。

```bash
# 终端 A
ros2 run demo_nodes_cpp talker

# 终端 B
ros2 run demo_nodes_cpp listener
```

构建与图形界面检查：

```bash
mkdir -p ~/nav_ws/src
cd ~/nav_ws
colcon build --symlink-install && source install/setup.bash
rviz2
```

## M2 接续顺序

1. 启动兼容 Fortress 的参考差速小车，确认遥控命令能驱动小车，ROS 端能收到 `/scan` 和 `/odom`，并检查底盘到雷达的 TF。
2. 启动 SLAM Toolbox，遥控走完整个场景，检查 RViz2 中的地图并保存地图文件。
3. 退出建图模式，用 AMCL 和 Nav2 加载保存的地图，设置初始位姿并发送目标，完成一次自主绕障到点。
