# ROS 2 Nav2 项目 M3 自建模型与验收

2026-11-22，M3 第一版自建模型完成。描述包编译及 xacro 展开成功，结构、惯性和几何接地关系检查通过，关节状态与雷达 TF 正常，用户确认 RViz 中外观显示正常。

本阶段对应原指南的自建 URDF/xacro，不包含 Gazebo 差速驱动和激光仿真插件。当前模型可在 RViz 查看，不会自行运动，也不会发布激光扫描；物理稳定性和传感器仿真留待 M4 验证。

## 文件与环境

- Windows 源码：`D:\Codex\ROS2-Nav2自主导航小车\src\my_robot_description`。
- Ubuntu 源码：`/home/geng/nav_ws/src/my_robot_description`。
- Ubuntu 构建结果：`/home/geng/nav_ws/install`。
- Ubuntu 验收记录：`/home/geng/nav_ws/artifacts/m3`；Windows 副本为 `artifacts/m3`。
- 使用 ROS_DOMAIN_ID=24 和系统时间，独立于 M2 的通信域 23。模型查看不依赖 Gazebo 时钟。

Windows 和 Ubuntu 是两份独立文件，修改后需要同步；目前在 Ubuntu 编译和运行。BCR 的 `/home/geng/bcr_ws` 和已验收的 M2 配置保留。

## 模型设计

采用 x 向前、y 向左、z 向上的坐标约定。蓝色长方体为底盘，黑色圆柱为驱动轮，灰色球体为后支撑轮，红色圆柱为雷达。

| 项目 | 第一版数值 |
|---|---|
| 底盘长、宽、高 | 0.45、0.35、0.12 m |
| 底盘质量 | 5 kg |
| 驱动轮半径、宽度 | 0.075、0.04 m |
| 左右轮中心距 | 0.40 m |
| 驱动轮轴相对 base_link 的 x 位置 | 0.08 m |
| 后支撑轮半径 | 0.025 m |
| 后支撑轮相对 base_link 的 x 位置 | -0.18 m |
| 雷达相对 base_footprint 的位置 | (0.10, 0, 0.225) m |

base_footprint 位于地面，base_link 位于距地面 0.075 m 的高度。车体视觉、碰撞和惯性原点相互一致；轮子圆柱沿 Y 轴布置，惯性张量采用对应轴向。几何计算表明双轮与支撑轮最低点均为地面，雷达底面与底盘顶面齐平。几何成立不等同于已经通过物理仿真。

驱动轮轴略前置，后部支撑轮与两驱动轮构成支撑三角形。后续 M4 仍需检查实际质量分布、接触与摩擦参数。

## 文件分工

| 文件 | 作用 |
|---|---|
| `package.xml`、`CMakeLists.txt` | ROS 包依赖、资源安装 |
| `urdf/my_robot.urdf.xacro` | 参数、颜色、模型组合入口 |
| `urdf/robot_base.xacro` | 底盘、驱动轮和后支撑轮 |
| `urdf/robot_laser.xacro` | 雷达外形与固定安装关节 |
| `launch/display.launch.py` | robot_state_publisher、joint_state_publisher 和可选 RViz |
| `rviz/model.rviz` | 模型查看配置，固定坐标系为 base_footprint |

长宽、轮半径、轮距在主 xacro 中提供参数入口。当前 display.launch.py 使用默认尺寸。Gazebo 插件文件待 M4 按 Fortress 兼容接口添加，未照搬指南中的 Gazebo Classic 插件。

## 坐标关系

```text
base_footprint
└── base_link
    ├── left_wheel_link
    ├── right_wheel_link
    ├── caster_link
    └── laser_link
```

轮关节为 continuous，其余为 fixed。模型显示时由 joint_state_publisher 给出静止轮角度；这些不是电机控制输出。M4 接入底盘驱动后，关节状态应来自仿真运动，不能保留两套冲突发布源。

## 编译与打开方法

以下命令在 Ubuntu 执行，先确保没有重复的 M3 显示实例。

```bash
source /opt/ros/humble/setup.bash
cd ~/nav_ws
colcon build --symlink-install --packages-select my_robot_description
source install/setup.bash
source ~/tf_fix_ws/install/setup.bash
export ROS_DOMAIN_ID=24
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export LIBGL_ALWAYS_SOFTWARE=1
export QT_QPA_PLATFORM=xcb
ros2 launch my_robot_description display.launch.py
```

本轮编译输出 `1 package finished`。不打开 RViz 时，可在最后一条命令追加 `rviz:=false`。在启动终端按 Ctrl+C 可结束显示节点；如果是通过 SSH 后台启动的实例，应先停止对应启动进程再重开，避免重复节点。

另一个 Ubuntu 终端可检查消息和 TF：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=24
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
timeout 5s ros2 topic echo /joint_states --once
timeout 5s ros2 run tf2_ros tf2_echo base_footprint laser_link
```

TF 查询初始可能短暂等待发现坐标系；本轮随后持续返回平移 `[0.100, 0.000, 0.225]` 和零旋转，满足预期。timeout 到期结束持续查询不代表变换失败。

## 验收与后续

| 检查 | 结果 |
|---|---|
| 描述包构建与安装 | 通过 |
| xacro 展开 | 通过 |
| 6 个 link、5 个 joint、唯一根及子节点 | 通过 |
| 正质量、正对角惯性及惯性三角关系 | 通过 |
| 驱动轮轴向、默认尺寸下接地和雷达安装高度 | 几何检查通过 |
| 两个轮关节状态 | 收到，初始角度为 0 |
| base_footprint 到 laser_link | 返回正确变换 |
| RViz 外观 | 用户确认显示正常 |

M3 描述模型基线已经建立。下一阶段 M4 为自建模型添加 Fortress 的差速驱动与雷达仿真、必要桥接和独立世界入口，验证接地稳定、遥控运动、里程计、时钟、扫描及动态 TF，再进入自建模型建图和导航。M2 保存的地图及导航参数不应未经适配直接当成新模型验收结果。
