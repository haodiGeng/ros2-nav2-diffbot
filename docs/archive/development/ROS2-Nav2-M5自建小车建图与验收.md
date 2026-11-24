# ROS 2 Nav2 项目 M5 自建小车建图与验收

2026-11-22，自建小车的 SLAM 建图、地图及位姿图保存、SLAM 恢复和 Nav2 地图文件加载验证已完成。地图可作为 M6 定位导航的初始输入。地图边缘的部分墙线仍有间断，本轮不将其认定为完整闭合或高精度地图。

## 1. 本阶段做了什么

M4 验证了小车能运动并产生传感器数据；M5 将 `/scan`、轮式 `/odom` 和 TF 接入 SLAM Toolbox，让它估计 map→odom，并将沿途扫描拼接为地图。

本轮没有启动 AMCL 或 Nav2 导航，也没有用仿真真值替代 SLAM 输入。补扫运动来自针对固定场景的低速预设路线，是数据采集辅助，不是自主探索或避障规划器。

| 工作 | 作用与结果 |
|---|---|
| 接入 my_robot_navigation 包 | 保存建图参数、入口、RViz 和补扫脚本 |
| 从新仿真实例开始 | 统一起点与朝向，避免前期遥控状态影响建图 |
| 在线异步建图 | 初始扫描即生成 117×117 格地图 |
| 七个补扫点 | 从不同方向观察围墙和箱体，再回到起点附近 |
| 保存地图 | 得到 YAML、PGM，供后续地图服务器和 AMCL 使用 |
| 保存位姿图 | 得到 posegraph、data，供 SLAM 恢复建图使用 |
| 重启 SLAM 恢复 | 恢复地图尺寸、占据栅格符合已保存场景 |
| Nav2 地图服务器加载 | 独立通信域中实际加载 YAML/PGM 成功 |
| 整合入口验证 | 新仿真中启动建图成功，并再次恢复保存位姿图 |

## 2. 环境、参数与坐标关系

沿用 M4 的 Fortress 自建小车及围墙箱体场景。ROS_DOMAIN_ID=25，RMW_IMPLEMENTATION=rmw_cyclonedds_cpp，IGN_PARTITION=my_robot_m4。节点使用仿真时间，入口加载已验证的 TF 修复覆盖库。

| 参数 | 当前设置 |
|---|---|
| SLAM 模式 | online_async，mapping |
| 地图、里程计、底盘帧 | map、odom、base_footprint |
| 激光话题 | /scan，消息帧 laser_link |
| 地图分辨率 | 0.05 m |
| 激光使用范围 | 0.05～12.0 m |
| 最小处理时间间隔 | 0.2 秒 |
| 最小平移、旋转更新阈值 | 0.1 m、0.1 rad |
| 地图更新周期 | 2 秒 |
| 回环检测 | 启用 |

坐标链为 `map → odom → base_footprint → base_link → laser_link`。建图期间 map→odom 只由 SLAM Toolbox 提供；不应同时开启 AMCL 或另发固定 map→odom。

## 3. 补扫路线与结果

从原点附近开始，按轮式里程计反馈依次前往以下点位，最后将朝向调整到接近 0 rad。单点位置到达阈值为 0.07 m，最大线速度 0.15 m/s，最大角速度 0.4 rad/s。

| 顺序 | 目标点 m | 实际耗时 |
|---|---|---|
| 1 | (-1.8, 0) | 20.45 秒 |
| 2 | (-1.8, -1.8) | 16.79 秒 |
| 3 | (1.8, -1.8) | 29.00 秒 |
| 4 | (1.8, 1.8) | 28.71 秒 |
| 5 | (-1.8, 1.8) | 28.99 秒 |
| 6 | (0, 1.8) | 20.12 秒 |
| 7 | (0, 0) | 16.57 秒 |

脚本检查激光和里程计是否持续更新；前向约 ±0.35 rad 范围内有有限测距小于 0.42 m 时中止，并在退出前发送零速度。本轮路线全部完成，没有触发保护。该检查不是完整碰撞检测，不能把这条固定路线脚本当作通用导航程序。

## 4. 地图结果与质量边界

| 项目 | 保存结果 |
|---|---|
| 尺寸 | 119×118 格 |
| 分辨率 | 0.05 m |
| 原点 | (-2.97, -2.96, 0) |
| 占据、空闲、未知格数 | 563、13337、142 |
| 左右墙峰值间距 | 5.85 m |
| 上下墙峰值间距 | 5.80 m |
| 场景理论内墙间距 | 5.85 m |

墙面间距由占据格的行列直方图峰值估计，上下方向差异为一个栅格。这是局部尺寸诊断，不是整幅地图的误差评测；没有据此计算或宣称全局建图精度。

已检查 [地图预览](../../../artifacts/m5/map_preview.png)：箱体方形轮廓已形成，没有明显大范围重影，但边界上的部分墙线存在间断。地图尺寸本身也不代表每个区域都已充分观测。M6 使用前仍需检查边界与代价地图，必要时继续补扫或优化建图参数。

## 5. 文件保存在哪里

Ubuntu 源码包：`/home/geng/nav_ws/src/my_robot_navigation`。

```text
my_robot_navigation/
├── config/slam_mapping.yaml
├── launch/slam.launch.py       # 仅启动 SLAM，要求仿真已存在
├── launch/mapping.launch.py    # 新仿真、SLAM、可选 RViz/GUI
├── rviz/mapping.rviz
└── scripts/
    ├── start_mapping.sh
    └── mapping_sweep.py
```

Ubuntu 地图前缀：`/home/geng/nav_ws/artifacts/m5/maps/my_robot_training`。

| 扩展名 | 用途 |
|---|---|
| .yaml | 分辨率、原点、图像路径和阈值 |
| .pgm | 供地图服务器加载的栅格图像 |
| .posegraph | SLAM 的位姿图 |
| .data | 对应的 SLAM 扫描等数据 |

Windows 已备份到 `D:\Codex\ROS2-Nav2自主导航小车\artifacts\m5`，源码副本在项目 `src/my_robot_navigation`。YAML 使用相对图像文件名，复制时需将 YAML 和 PGM 放在一起；恢复 SLAM 时则需保留 posegraph 和 data，文件前缀一致。

## 6. 下次怎样开始新建图

先停止已有 M4/M5 实例和遥控，再在 Ubuntu 桌面终端执行以下命令。当前 SSH 启动的实例不会因为关掉 RViz 窗口就停止，应先结束对应启动进程，不能直接叠加启动。

```bash
source /opt/ros/humble/setup.bash
cd ~/nav_ws
chmod u+x src/my_robot_navigation/scripts/*.py src/my_robot_navigation/scripts/*.sh
colcon build --symlink-install --packages-select my_robot_navigation
bash ~/nav_ws/src/my_robot_navigation/scripts/start_mapping.sh
```

默认启动新仿真、SLAM 和 RViz；加 `rviz:=false` 可关闭可视化，加 `gui:=true` 可额外打开 Gazebo GUI，但本阶段没有验证双界面性能。

该入口默认是**重新开始一张地图**，不会自动加载本轮检查点，也不会自动保存覆盖旧地图。

另一个终端执行以下环境设置后，可以使用 M4 的键盘遥控方法低速采集数据：

```bash
source /opt/ros/humble/setup.bash
source ~/nav_ws/install/setup.bash
export ROS_DOMAIN_ID=25
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

若要重复本轮固定补扫路线，只能在新仿真原点附近、无人遥控且场景未改动时执行：

```bash
python3 ~/nav_ws/src/my_robot_navigation/scripts/mapping_sweep.py
```

## 7. 保存与恢复

小车停车后等待地图更新，再保存到一个新前缀。下例使用新文件名，避免覆盖已经验收的检查点。所有命令均在设置好域 25 的 Ubuntu 终端执行。

```bash
mkdir -p ~/nav_ws/artifacts/m5/maps
ros2 run nav2_map_server map_saver_cli \
  -f "$HOME/nav_ws/artifacts/m5/maps/my_robot_training_next" \
  --ros-args -p use_sim_time:=true -p save_map_timeout:=15.0

ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph \
  "{filename: '$HOME/nav_ws/artifacts/m5/maps/my_robot_training_next'}"
```

栅格地图可由 Nav2 map_server 加载；若继续建图，则需要恢复位姿图。下面的恢复示例限定在**当前场景的新仿真原点，尚未移动**时使用，initial_pose 表示机器人在已保存地图中的近似位置和朝向。机器人已经移动时必须提供对应真实初值，不能照填零。

```bash
ros2 service call /slam_toolbox/deserialize_map \
  slam_toolbox/srv/DeserializePoseGraph \
  "{filename: '$HOME/nav_ws/artifacts/m5/maps/my_robot_training', \
    match_type: 2, initial_pose: {x: 0.0, y: 0.0, theta: 0.0}}"
```

上述操作会将当前 SLAM 图替换为保存图，需要先保存当前仍有价值的建图成果。

## 8. 实际加载验证

1. 在原仿真中停车，记录当前 map→base_footprint 位姿；重启 SLAM，用该位姿恢复检查点。恢复后地图仍为 119×118 格、563 个占据格。
2. 在独立域 26 启动 Nav2 map_server，完成 configure、activate，实际加载 YAML/PGM 后收到相同尺寸及 563 个占据格；验证服务器随后退出，没有混入主域。
3. 结束分开启动的节点，运行整合入口。新仿真首先生成 117×117 格初始地图，再从出生点恢复保存图，得到 119×118 格、564 个占据格。新扫描参与处理可能造成少量格子变化，未要求逐格完全相同。

以上验证不是 Ubuntu 关机开机测试；也不代表已经完成 AMCL 定位或导航。

验收证据包括 [补扫记录](../../../artifacts/m5/mapping_sweep.json)、[SLAM 恢复结果](../../../artifacts/m5/restore_validation.json)、[地图文件加载结果](../../../artifacts/m5/map_file_validation.json)、[整合入口结果](../../../artifacts/m5/combined_launch_validation.json) 和 [地图指标](../../../artifacts/m5/map_metrics.json)。

## 9. 当前状态和下一阶段

本轮结束时，整合建图入口保持运行，小车已停车，SLAM 已恢复检查点，RViz 可查看地图。地图文件已保存，不依赖这些进程继续运行。

下一步 M6：停止 SLAM 的 map→odom 发布，加载本轮栅格地图，接入 AMCL 与 Nav2，按仿真模式的真实轮轴和车体范围配置 footprint、膨胀与速度，并验证自建小车绕障到点。BCR 的地图、话题和参数不能未经适配直接作为新小车的结果。
