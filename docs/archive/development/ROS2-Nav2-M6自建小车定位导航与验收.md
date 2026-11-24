# ROS 2 Nav2 项目 M6 自建小车定位导航与验收

2026-11-22，自建小车在 M5 保存地图上完成 AMCL 定位、NavFn 规划、速度控制及连续绕障到点验证。当前默认基线采用 DWB；在完整入口启动、RViz 开启的测试中，三个连续目标均成功，恢复次数均为 0。

本轮结论限于当前简单静态场景，不包含动态障碍、复杂仓库、Ubuntu 再次关机开机复测或真实机器人。M5 地图边缘墙线仍有局部间断，本轮路线避开这些边界区域，不能据此认定地图质量问题已全部消除。

## 1. 从建图切换到定位导航

M5 的 SLAM Toolbox 生成地图并发布 map→odom；M6 停止 SLAM，地图服务器加载已经保存的地图，由 AMCL 发布 map→odom。差速插件继续根据轮子运动提供 odom→base_footprint，robot_state_publisher 提供底盘到轮子和雷达的变换。

运行核查确认 `/tf` 发布节点仅有各一个 amcl、ros_gz_bridge、robot_state_publisher，没有残留 SLAM 实例，AMCL 实际加载已验证的 tf2 修复覆盖库。仿真真值 `/ground_truth` 仅用于评测，不代替 AMCL 或轮式里程计。

## 2. 当前配置

| 项目 | 配置 |
|---|---|
| 工作空间 | Ubuntu `/home/geng/nav_ws` |
| 导航包 | my_robot_navigation |
| 通信环境 | ROS_DOMAIN_ID=25，Cyclone DDS |
| Gazebo 隔离环境 | IGN_PARTITION=my_robot_m4 |
| 时钟 | 仿真时间 |
| 地图 | 包内 maps/my_robot_training.yaml 及 PGM，119×118，0.05 m |
| 底盘、里程计、地图帧 | base_footprint、odom、map |
| 激光、轮式里程计、控制 | /scan、/odom、/cmd_vel |
| 定位 | AMCL，激光最大范围 12 m，180 束采样 |
| 全局规划 | NavFn，use_astar=false |
| 当前局部控制器 | dwb_core::DWBLocalPlanner |
| 导航线速度上限 | 0.18 m/s |
| 控制器角速度上限 | 0.60 rad/s |
| 全局与局部代价地图半径 | 0.35 m |
| 膨胀半径 | 0.65 m |
| 目标位置、朝向容差 | 0.12 m、0.15 rad |
| 进度检查 | 15 秒内平移超过 0.10 m |

0.35 m 圆形半径依据 simulation=true 模式的实际模型选取：车体中心后移 0.04 m，车体最远角距参考点约 0.318 m，轮组和后支撑轮也在圆形范围内。M2 的 BCR 半径为 0.60 m，不能直接照搬。

AMCL alpha1～alpha5 均设为 0.01，运动更新阈值为 0.05 m、0.05 rad。它们是本仿真轮式里程计的初始调试参数，不是实物标定结果。速度平滑器的加减速限制与 M4 驱动能力配合，未通过关闭碰撞或障碍检查取得到点结果。

## 3. 控制器对照与参数优化

先沿用 M2 已验证的 RPP 思路，为自建小车适配尺寸、速度和话题。首个短距离目标 `(-0.6,0)` 成功，随后 `(2,0) → (0,0) → (0,1.6)` 三个目标也连续成功，恢复次数均为 0。

完整重启并打开默认 RViz 后，RPP 绕障成功，但末端调整触发一次“进度不足”恢复，耗时 38.82 秒。轨迹显示机器人已经到达目标附近，主要在调整朝向。将进度检查从 10 秒移动 0.15 m 改为 15 秒移动 0.10 m 后，同路线为 35.76 秒、0 次恢复；然而返回原点仍耗时 47.65 秒、1 次恢复。因此仅放宽进度检查不能消除现象。

对照使用 DWB，保留相同地图、AMCL、半径、速度上限和目标容差。DWB 的 RotateToGoal 在进入位置窗口后会保持末端旋转状态，适合检查此次临近目标的反复调整现象。最终三个连续目标均未触发恢复。

状态逻辑参考 [Nav2 1.1.20 RotateToGoal 实现](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_dwb_controller/dwb_critics/src/rotate_to_goal.cpp) 与 [同版本 RPP 实现](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_regulated_pure_pursuit_controller/src/regulated_pure_pursuit_controller.cpp)。轨迹和源码支持这个调试方向，但少量对照不构成两种控制器的统计性能结论；各自的局部参数不同，不能将全部耗时差异仅归因于状态逻辑。

当前 DWB 关键配置包括：trans_stopped_velocity=0.03 m/s、sim_time=1.5 s、xy_goal_tolerance=0.12 m、RotateToGoal.slowing_factor=3。避障评分、路径评分和速度限制均保留。M2 的 BCR 基线仍使用 RPP，本阶段没有改写其配置。

## 4. 最终连续目标验收

默认导航入口完整启动新仿真、单套 AMCL/Nav2 和 RViz 后，从出生点开始连续执行以下目标；中途没有重新发布初始定位。

| 目标及朝向 | 状态 | 现实耗时 | 恢复次数 | 真值路径长度 | 最大采样 AMCL/world 位置差 |
|---|---|---|---|---|---|
| (2.0,0,0) | 4，成功 | 34.43 秒 | 0 | 3.19 m | 0.031 m |
| (0,0,0) | 4，成功 | 30.16 秒 | 0 | 3.13 m | 0.034 m |
| (0,1.6,0) | 4，成功 | 18.22 秒 | 0 | 1.64 m | 0.028 m |

第一条路径越过中心为 `(1.2,0)`、尺寸 0.4×0.4 m 的箱体所在区域，实际发生绕行，不是原地接收目标或直接穿越箱体。

三个目标的真值采样中，车体、轮组包络和支撑轮投影与箱体的几何重叠次数均为 0；对应的最小采样净间隙约为 0.356、0.387、0.650 m。检查覆盖车体中心后移和实际轮组范围，不能替代接触传感器或连续碰撞检测。

AMCL/world 差使用邻近时间戳配对，允许时间差不超过 0.2 秒。map 与 world 没有进行独立刚性标定，该值只作本场景诊断，不是正式的定位精度指标。末次 AMCL 消息也不一定恰好对应 action 结束时刻，不能直接用它单独判定末端朝向是否满足容差；Nav2 的成功判定基于控制时的 TF。

## 5. 代码与产物

Windows 源码位于 `D:\Codex\ROS2-Nav2自主导航小车\src\my_robot_navigation`，Ubuntu 对应 `/home/geng/nav_ws/src/my_robot_navigation`。Windows 修改后需要同步，实际构建与运行在 Ubuntu。

| 文件 | 用途 |
|---|---|
| config/nav2_params.yaml | 当前默认 DWB 配置 |
| config/nav2_dwb_params.yaml | DWB 明确命名副本 |
| config/nav2_rpp_params.yaml | RPP 对照配置，保留最后一轮进度参数 |
| maps/my_robot_training.yaml、PGM | M5 生成并加载验证过的地图副本 |
| launch/nav_demo.launch.py | 新仿真、地图、AMCL/Nav2 及可选 RViz |
| scripts/start_navigation.sh | 加载环境、域 25 和 TF 修复库后启动 |
| rviz/navigation.rviz | 模型、激光、地图、路径和 Nav2 目标工具 |
| scripts/navigation_trial.py | 等待生命周期激活，再执行导航目标及记录数据 |
| scripts/summarize_navigation.py | 真值路径、几何重叠和定位配对检查 |

原始数据在 Ubuntu `~/nav_ws/artifacts/m6`，Windows 备份位于项目 `artifacts/m6`。主要记录如下。

- `smoke_results.json`：RPP 初始短距离测试。
- `navigation_results.json`：RPP 第一轮连续目标。
- `restart_results.json`：RPP 重启后 38.82 秒、1 次恢复的对照。
- `final_results.json`、`final_return_results.json`：仅调整进度参数后的 RPP 对照，文件名中的 final 不代表当前最终控制器。
- `dwb_results.json`、`dwb_results_summary.json`：当前 DWB 验收结果。
- `dwb_navigation.log`、`dwb_environment_check.json`：当前配置运行日志和检查证据。

当前配置 SHA-256 为 `b446cb8ff884dbfc9773921f1d0b863ac638b6b497d8dd7006833e846d3f1ca0`，原始地图图像 SHA-256 为 `0e02584868e87f542bcd0aca8be7522e5820f34fe5934be506a00f4fbbb59e67`。

## 6. 手动启动与操作

先停止已有 M4、M5 或 M6 实例，尤其不能同时保留 SLAM。对于当前 SSH 后台启动实例，应先停止对应启动进程再重开；只关闭 RViz 窗口不会停止仿真和导航。

Ubuntu 桌面终端执行：

```bash
source /opt/ros/humble/setup.bash
cd ~/nav_ws
chmod u+x src/my_robot_navigation/scripts/*.py src/my_robot_navigation/scripts/*.sh
colcon build --symlink-install --packages-select my_robot_navigation
bash ~/nav_ws/src/my_robot_navigation/scripts/start_navigation.sh
```

入口默认加载包内地图、配置出生点初始位姿、开启 RViz，并等待导航激活。不需要重新建图；初始位姿设置只适用于从固定出生点开始的新仿真实例。

资源较紧时可追加 `rviz:=false`；需要 Gazebo 窗口可追加 `gui:=true`，但本轮 M6 没有验收 Gazebo GUI 同开时的性能。默认是后台仿真加 RViz。

看到 RViz 的 Navigation 和 Localization 均为 active 后，用 Nav2 Goal 在远离围墙和箱体的白色区域点击并拖出目标朝向。导航期间不要同时键盘遥控，不要拖动模型或重置仿真时间。

停止时先等待到点停车，再在前台启动终端 Ctrl+C，等待进程退出后关闭 Ubuntu。

## 7. 重复验收

从新仿真出生点开始，在另一终端设置环境：

```bash
source /opt/ros/humble/setup.bash
source ~/nav_ws/install/setup.bash
export ROS_DOMAIN_ID=25
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
python3 ~/nav_ws/src/my_robot_navigation/scripts/navigation_trial.py new_results.json
python3 ~/nav_ws/src/my_robot_navigation/scripts/summarize_navigation.py \
  ~/nav_ws/artifacts/m6/new_results.json
```

默认目标为 `(2,0) → (0,0) → (0,1.6)`，朝向均为 0。脚本等待 AMCL、规划器、控制器、导航器激活，记录 action 状态、恢复次数、轮式里程计、真值、AMCL 和规划路径，结果写到 `~/nav_ws/artifacts/m6`。

可在输出文件名后追加 `smoke` 只测 (-0.6,0)，或 `avoidance` 只测 (2,0)。`return` 模式测试 (0,0)、(0,1.6)，本轮仅用于已到达箱体另一侧后的补充对照。测试脚本会实际驱动小车，不应与人工目标或遥控并行运行。

## 8. 完成范围与下一阶段

已完成自建小车的地图加载、AMCL 定位、NavFn/DWB 接入、参数调试、真实绕障和三个连续目标测试；默认整套入口已实际启动并验证，结束后小车已停车，仿真、AMCL/Nav2 和 RViz 保持运行。

尚未进行当前自建导航配置的 Ubuntu 关机开机复测、长时间重复统计、动态障碍或复杂仓库验证。下一阶段 M7 实现自研 A* 全局规划插件，使用本阶段已验证的模型、地图、定位和局部控制器作为对照基线；M8 再统一评测成功率、规划耗时、路径和演示资料。
