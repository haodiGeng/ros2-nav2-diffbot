# ROS 2 Nav2 项目 M4 自建小车仿真与验收

2026-11-22，M4 基础仿真与运动感知验收通过。自建机器人已在 Gazebo Fortress 中完成落地、低速前进、后退、原地转向和停车测试，ROS 端持续收到时钟、激光、轮式里程计、关节状态和动态 TF。

本轮使用无 GUI 仿真完成测试，尚未用自建机器人建图或自主导航，也没有进行 Ubuntu 重启及长时间耐久验收。下一阶段 M5 将用它重新进行 SLAM 建图。

## 1. 与 M3 的区别

M3 只有模型描述和静态展示。M4 增加物理场景、差速驱动、GPU 激光、真实仿真轮角及 ROS 桥接。运动由速度指令驱动轮子实现，ROS `/odom` 来自差速插件的轮式积分；独立 `/ground_truth` 只用于验收，不用于替代里程计或发布第二套 odom TF。

通信环境独立设置为 ROS_DOMAIN_ID=25、IGN_PARTITION=my_robot_m4。M2 的 BCR 使用域 23，M3 静态展示使用域 24，不应将这些终端环境混用。

## 2. 文件组织

Windows 源码位于项目 `src/my_robot_description` 和 `src/my_robot_gazebo`；Ubuntu 对应目录为 `~/nav_ws/src/` 下的两个包。

| 文件 | 用途 |
|---|---|
| `my_robot_description/urdf/robot_gazebo.xacro` | Fortress 差速、关节状态、真值和激光配置 |
| `my_robot_description/urdf/my_robot.urdf.xacro` | simulation 参数及仿真专用几何调整 |
| `my_robot_description/urdf/robot_base.xacro` | 轮胎接触几何和车体质量位置 |
| `my_robot_gazebo/worlds/training.sdf` | 6 米围墙场景、地面、灯光和固定箱体 |
| `my_robot_gazebo/launch/sim.launch.py` | 服务器、模型生成、状态发布和桥接 |
| `my_robot_gazebo/scripts/start_simulation.sh` | 统一环境并启动仿真 |
| `my_robot_gazebo/scripts/validate_sim.py` | 从新出生点执行有界运动测试，最终发送零速度 |

场景中的箱体中心为 `(1.2,0)`，尺寸为 `0.4×0.4×0.5 m`。雷达位于车体前方 x=0.10 m，因此初始正前方观测到箱体近面时，理论距离为 `1.2-0.2-0.1=0.9 m`。

## 3. 插件与接口

使用 Fortress 的 `ignition::gazebo::systems::DiffDrive`、`JointStatePublisher`、`OdometryPublisher` 和 `Sensors`，不使用 Gazebo Classic 的 gazebo_ros 插件。桥接采用已安装的 ros_gz_bridge。

| ROS 接口 | 来源或去向 |
|---|---|
| `/cmd_vel` | ROS → Gazebo 差速驱动 |
| `/odom` | Gazebo 轮式里程计 → ROS |
| `/tf` | 差速插件 odom→base_footprint |
| `/tf_static` | robot_state_publisher 提供固定安装关系 |
| `/joint_states` | Gazebo 中两驱动轮的关节状态 |
| `/scan` | GPU 激光，laser_link，360 束，15 Hz，0.05～12 m |
| `/clock` | 仿真时间 |
| `/ground_truth` | 独立的仿真世界位姿，供对照检查 |

没有启动 M3 的 joint_state_publisher，避免假定轮角和仿真轮角冲突。robot_state_publisher 使用仿真时间；当前不发布 map→odom，因为尚未启动 SLAM 或 AMCL。

TF 已实测连通 `odom → base_footprint → base_link → laser_link`，运动过程中查询到的时间戳和变换持续更新。

## 4. 本轮发现与修正

### 轮轴与里程计原点

M3 的驱动轴在 base_link 前方 0.08 m，不能直接当作差速积分的参考原点。首轮转向时参考点实际平移约 0.076 m，而轮式积分认为是原地转向。

仿真模式将 wheel_x 设为 0，让轮轴与参考原点对齐；车体视觉、碰撞和惯性中心一同后移 0.04 m，以使重心落在双轮及后支撑轮构成的支撑区域内。静态展示默认模式保持 M3 原配置。这意味着两种模式的几何不是完全相同，后续导航应以 simulation=true 的实际模型计算 footprint。

### 轮胎接触几何

轴对齐后转向位置误差消失，但实际转角约 0.998 rad，轮式积分约 0.899 rad，角差仍约 0.099 rad。该比例与宽圆柱轮接触边缘使有效轮距缩小的推断一致。

最终仿真轮胎采用窄中央胎面及略小半径的全宽侧肩近似圆角接触：中央胎面半径 0.075 m、宽 0.002 m；侧肩半径 0.073 m、宽 0.04 m。视觉轮胎仍为原圆柱，质量与惯性保留。这是仿真接触近似，不是实物轮胎标定。轮距仍为实际几何中心距 0.40 m，未靠改写轮距参数掩盖偏差。

加入严格的原地转向位置及里程计角差检查后，最终转角差降至约 0.00417 rad。首轮宽松检查虽曾输出 passed=true，但遗漏了转向一致性要求，不作为最终验收；修正前后结果均已保留。

## 5. 最终实测

测试使用 `/ground_truth` 对照实际运动，速度为前后 0.1 m/s、转向 0.3 rad/s，各运动指令持续约 3 秒，随后发送零速度并等待稳定。

| 项目 | 结果 |
|---|---|
| 5 秒静止 | 真值位置漂移采样值为 0 |
| 时间推进 | 约 5.0003 秒现实时间推进 5.000 秒仿真时间 |
| 激光连续性 | 5 秒收到 76 帧，360 束，frame 为 laser_link |
| 正前箱体距离 | 0.900055 m，与理论 0.9 m 一致 |
| 前进真值距离 | 0.300200 m |
| 后退真值距离 | 0.300200 m |
| 转向真值 | 0.904769 rad |
| 转向轮式里程计 | 0.900600 rad |
| 转向角差 | 0.004169 rad，约 0.239° |
| 原地转向平移 | 约 0.00000124 m |
| 5 秒停车 | 真值位置漂移采样值为 0 |

模型没有在这些测试中出现穿地、明显倾倒或失控。短时结果不能代替长时间稳定性、复杂接触、碰撞或真实里程计噪声评测。

## 6. 手动编译和启动

以下命令均在 Ubuntu 执行。先确保没有同一 M4 场景的旧实例。

```bash
source /opt/ros/humble/setup.bash
cd ~/nav_ws
chmod u+x src/my_robot_gazebo/scripts/*.py src/my_robot_gazebo/scripts/*.sh
colcon build --symlink-install --packages-select my_robot_description my_robot_gazebo
bash ~/nav_ws/src/my_robot_gazebo/scripts/start_simulation.sh
```

如需启动时同时打开三维窗口，最后一条改为：

```bash
bash ~/nav_ws/src/my_robot_gazebo/scripts/start_simulation.sh gui:=true
```

GUI 选项已提供，但本轮验收没有覆盖其性能。若后台已在运行，可在另一终端只连接 GUI，不要再运行整套启动入口：

```bash
source /opt/ros/humble/setup.bash
export IGN_PARTITION=my_robot_m4
export LIBGL_ALWAYS_SOFTWARE=1
export QT_QPA_PLATFORM=xcb
ign gazebo -g --force-version 6
```

## 7. 遥控和重复测试

新终端统一环境：

```bash
source /opt/ros/humble/setup.bash
source ~/nav_ws/install/setup.bash
export ROS_DOMAIN_ID=25
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

可运行已安装的键盘遥控节点。让键盘焦点停留在遥控终端，i 前进、逗号后退、j/l 转向、k 停止，Ctrl+C 退出：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -p speed:=0.1 -p turn:=0.3 -p repeat_rate:=10.0 -p key_timeout:=0.6 \
  -r cmd_vel:=/cmd_vel
```

默认首次按键可能开始发送运动命令，应先确认通道畅通。不要同时遥控和运行自动验收。

重复验收前完整停止并重新启动 M4 仿真，让机器人回到默认位置和朝向。否则初始箱体距离检查会因机器人已移动而失败。

```bash
python3 ~/nav_ws/src/my_robot_gazebo/scripts/validate_sim.py
```

脚本会实际驱动机器人，并写入 `~/nav_ws/artifacts/m4/validation.json`；它在退出前发送零速度。停止手动遥控时先按 k，确认停车后退出，再在仿真启动终端按 Ctrl+C。若本轮实例是通过 SSH 在后台启动，应按其对应启动进程停止，不能只关闭 GUI 窗口后就重复启动。

## 8. 验收证据和后续工作

Windows 副本位于 `artifacts/m4`：

- `validation.json`：最终通过的完整结果。
- `before_axle_fix.json`：轮轴修正前记录，旧 passed 字段不代表最终标准通过。
- `before_tread_fix.json`：轮轴已对齐但转角差仍不满足要求的记录。
- `simulation_tread_fix.log`：最终运行日志。
- `simulation_model.sdf`：最终仿真模型转换结果。

下一阶段 M5 复用该仿真入口，用自建机器人进行 SLAM 建图和地图保存；M6 再接入 AMCL 与 Nav2。BCR 的地图和导航参数保留为历史基线，不直接作为新模型的验收数据。

Fortress 接口参考 [官方差速示例](https://github.com/gazebosim/gz-sim/blob/ign-gazebo6/examples/worlds/diff_drive.sdf) 和 [Humble ros_gz_bridge 实现](https://github.com/gazebosim/ros_gz/blob/humble/ros_gz_bridge/src/parameter_bridge.cpp)。
