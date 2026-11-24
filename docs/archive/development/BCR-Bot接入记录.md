# BCR Bot 接入记录

2026-11-19 已在 Ubuntu 的 `/home/geng/bcr_ws` 创建独立候选工作空间。

- 上游 https://github.com/blackcoffeerobotics/bcr_bot
- 分支 `ros2`
- 固定提交 `e022fa0ce466bf8faec636454e90fbffa585a718`
- 已读取 Apache-2.0 许可证。
- `colcon build --symlink-install --packages-select bcr_bot` 成功，1 个包构建完成。

## 接入顺序

1. 最小仿真，使用简单场景，关闭相机和双目，保留 2D 雷达。
2. 核对 `/clock`、`/bcr_bot/scan`、`/bcr_bot/odom`、TF 和 `/bcr_bot/cmd_vel`，量化仿真速度。
3. 用 SLAM Toolbox 建图并保存独立地图。
4. 检查上游 Nav2 配置，切换到 AMCL 定位，验证到点及绕障。
5. 固定基线后加入自建模型、自研 A* 和评测。

源码检查发现，`ign.launch.py` 在 `gz_args` 内拼接 world_file，未直接提供无 GUI 开关。适配时应使用项目内的启动入口。生成模型的节点也需确认启用仿真时间。默认里程计来源为 world，初次复现应记录该设定，算法评测时再确定轮式里程计方案。

编译成功仅验证包的构建与安装，运行时仿真、渲染、建图及导航尚未验收。

## 2026-11-20 更新

最小仿真已通过验收，详见同目录的 BCR 最小仿真验收记录。补扫路线 `(0,1.2) → (2.1,1.2) → (2.1,0)` 均已低速到达，地图已保存为 Ubuntu `~/bcr_ws/validation/maps/bcr_minimal.yaml` 及 PGM，尺寸为 120 × 126，分辨率 0.05 m。

上游导航入口包含 bringup 之外额外的 AMCL、地图节点和静态 map→odom。当前改用单套 Nav2 bringup，设置车体保守半径 0.58 m。上游速度转发脚本未安装成 ROS 可执行程序，本轮直接用 Python 启动源码脚本。

运行时发现原 Gazebo TF 在静止时出现旧时间戳，已停止原 `/tf` 桥接，由同源里程计持续发布 `odom→base_footprint`。独立 TF 查询可取得实时变换，但 AMCL 尚未稳定发布 `map→odom`，Nav2 全局代价地图等待 map 变换，绕障目标被拒绝。此项尚未完成。

本轮诊断中 AMCL 的 base_frame_id 临时改为 base_footprint，保存的 nav2.yaml 仍以 base_link 为底盘帧。正式复现前应统一、核验配置。当前没有活动的成功绕障任务，未以静态 map→odom 代替 AMCL 验收。

### Cyclone DDS 对照

用户安装 `ros-humble-rmw-cyclonedds-cpp` 后，已停止旧测试进程并将仿真、桥接、速度转发与导航统一设置为 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`，保持 ROS_DOMAIN_ID=23。重启后目标改为从出生点前往 `(2.2,0)`，避免原地到点误判。

实时扫描和独立 TF 查询正常，但 AMCL 内部仍报告旧 TF 时间，未建立 map→odom，目标被拒绝。另生成 `nav2_footprint.yaml`，将 AMCL、导航器和代价地图统一使用 base_footprint 后重启，现象仍复现。故更换 DDS 和统一底盘帧均未单独解决问题，先前关于 DDS 导致故障的判断仍属于假设。

当前 Nav2 使用 `nav2_footprint.yaml`，相关日志为 Ubuntu `validation/cyclone_nav.log`。下一步需要独立开启 AMCL 调试日志，核查扫描消息过滤器及 TF 监听缓存的实际收包情况。

### TF 死锁定位与首次绕障成功

AMCL 调用栈显示扫描线程阻塞在 `Buffer::waitForTransform → BufferCore::addTransformableRequest`，TF 线程阻塞在 `testTransformableRequests` 的回调锁。系统 tf2/tf2_ros 为 0.25.23，锁等待链与上游问题 https://github.com/ros2/geometry2/issues/992 一致。更换 DDS 没有消除此问题。

在独立 `~/tf_fix_ws` 获取 geometry2 0.25.23，提交为 `9bc9e89285515c8981ecd829c9400807372468c4`。局部修改 `testTransformableRequests`，在内部锁保护下收集已完成的回调及参数并清理请求，释放锁后执行回调。只编译 tf2，通过覆盖工作空间加载。AMCL 的 `/proc/PID/maps` 已确认实际加载 `~/tf_fix_ws/install/tf2/lib/libtf2.so`。

本地修复仍需完整并发与取消行为回归验证，当前是项目内验证用补丁，尚无上游合并或通用稳定性结论。系统 `/opt/ros/humble` 安装包保留。

修复后，在简单场景中首次完成从原点到 `(2.2,0)` 的自主绕障。

- Nav2 action 状态 4（SUCCEEDED），服务端日志为 Reached the goal / Goal succeeded。
- 实际等待 21.20 秒，21 条规划路径记录。
- 方块中心 `(1.2,0)`，尺寸 `0.4×0.4×0.5 m`。
- 里程计终点约 `(2.160,-0.254)`，最大横向绕行约 0.834 m。
- 穿过方块横向覆盖区时，机器人中心横向偏移至少约 0.791 m。
- 按里程计估计，机器人中心到方块矩形边缘最小距离约 0.546 m。该距离不等同于车体净间隙，本轮没有接触传感器证据用于证明零碰撞。

### 重复测试与完整重启验收

2026-11-20 在首次成功后连续执行三次不同目标，均返回状态 4。

| 目标 | 实际耗时 | 按里程计采样计算的车体矩形与方块重叠次数 |
|---|---|---|
| `(0,0)` | 23.53 秒 | 0 |
| `(2.2,0)` | 19.35 秒 | 0 |
| `(0,1.4)` | 26.09 秒 | 0 |

重叠计算采用车体 0.9×0.64 米矩形及方块 0.4×0.4 米矩形，按每帧里程计姿态进行分离轴检测。这是离散采样下的几何检查，未覆盖所有车体附件，也不能替代接触传感器或连续碰撞检测。

随后停止旧仿真、定位和导航进程，用新统一入口完整重启。AMCL 与 Nav2 自动激活，再次从出生点前往 `(2.2,0)`，返回状态 4，耗时 22.74 秒，服务端记录 Goal succeeded。本轮重启指进程整套重启，尚未做 Ubuntu 关机开机后的复测。

统一入口为 Ubuntu `~/bcr_ws/validation/start_navigation.sh`，在 Ubuntu 桌面终端运行以下命令。

```bash
bash ~/bcr_ws/validation/start_navigation.sh
```

入口自动加载 Humble、BCR 工作空间和 TF 修复覆盖库，设置 Cyclone DDS、ROS_DOMAIN_ID=23、IGN_PARTITION=bcr_m2、软件渲染，启动无 GUI 仿真、单套 AMCL/Nav2 和速度转发。已设置仿真出生点初始位姿。重复运行前先停止已有的整套进程，避免重复实例。

Windows 副本位于 `artifacts/bcr/`，包括 `start_navigation.sh`、`navigation_demo.launch.py`、`nav2_demo.yaml`、`repeat_results.json`、`restart_avoidance_result.json`。Ubuntu 原始配置和日志在 `~/bcr_ws/validation/`。

M2 在 BCR 简单场景下的建图、保存地图、AMCL 定位、实际绕障到点及进程重启复现均已通过。复杂仓库、轮式里程计、TF 补丁的完整并发回归仍是后续工作。

Windows 首轮原始结果为 `artifacts/bcr/avoidance_result.json`，补丁为 `artifacts/bcr/tf2-deadlock.patch`。本次使用 world 里程计和 Cyclone DDS，ROS_DOMAIN_ID=23。TF 修复库完整并发回归仍待完成。

## 2026-11-21 冷启动排查与参数复测

Ubuntu 开机后的首轮绕障失败，状态为 6。AMCL 和 Nav2 均已激活，本次没有再次出现之前的 TF 死锁。规划器内部地图的起终点看似可通行，但对障碍附近起点规划失败；改用远离障碍的明确起点能够规划成功。

初始参数的 inflation_radius 为 0.55 m，小于 robot_radius 的 0.58 m。将两套代价地图膨胀半径增至 0.9 m 后，首轮绕障成功，耗时 18.76 秒，但随后连续测试仍失败，因此不能把增大膨胀范围视为完整修复。

继续采样发现，失败时 world 里程计位置约为 `(2.126,-0.509)`，AMCL 约为 `(1.206,-0.263)`，位置差约 0.95 m。此时 AMCL 所在的代价地图区域为 253，即障碍膨胀不可通行区。定位漂移会使规划器认为机器人进入了障碍附近，取整边界也可能加剧失败。这里的 world 里程计只用于独立对照，没有替换 AMCL 发布的 map→odom。

当前 `nav2_demo.yaml` 在保留 0.58 m 保守车体半径的基础上作了以下调整。

- 两套代价地图 inflation_radius 从 0.55 改为 0.9 m。
- AMCL alpha1～alpha5 从 0.2 改为 0.005，适配本轮噪声很小的 world 里程计。
- max_beams 从 60 改为 180，laser_max_range 从 100 改为仿真激光的 16 m。
- update_min_a、update_min_d 均改为 0.1，以增加定位更新机会。
- 保持 NavFn 的 use_astar=false。单独切换 A* 不能解决定位漂移造成的失败。

这些参数共同调整后，整套仿真进程重新启动，连续四个目标均成功，目标之间没有重新初始化 AMCL。

| 目标 | action 状态 | 现实耗时 | 最大采样位置偏差 |
|---|---|---|---|
| `(2.2,0)` | 4 | 16.52 秒 | 约 0.035 m |
| `(0,0)` | 4 | 18.01 秒 | 约 0.05 m |
| `(2.2,0)` | 4 | 81.22 秒 | 约 0.25 m |
| `(0,1.4)` | 4 | 35.18 秒 | 约 0.37 m |

位置偏差是 AMCL 消息与最近接收的 world 里程计位置之差，消息未做精确时间插值，且默认两坐标系在出生点对齐；它是诊断指标，不是严格定位精度评测。第三项目标出现 Failed to make progress，经过恢复后到达，不能将四次成功解读为导航已经稳定可靠。本次只记录 action 到达结果，没有补充接触传感器碰撞验收。

最新轨迹和定位采样为 Windows `artifacts/bcr/inflation_regression.json`，脚本为 `artifacts/bcr/navigation_regression.py`。日志 `cold_boot_20261121.log` 保留首次失败，`inflation_fix_navigation.log` 保留仅调整膨胀层后的失败，`amcl_fix_navigation.log` 保留最终四目标测试。最新参数已同步到 Windows，Ubuntu 入口仍为 `bash ~/bcr_ws/validation/start_navigation.sh`。

边界说明：本轮确认的是 Ubuntu 开机后发现问题、修改参数、进程整套重启后的复测。最终参数尚未再次经历 Ubuntu 关机开机验收；真实轮式里程计必须重新估计运动噪声，不能直接照搬 alpha=0.005。后续应先检查定位漂移及 81 秒绕障中的恢复过程，再固定可靠基线并进入自建机器人适配。

规划器坐标取整与代价读取的核查参考 [Nav2 1.1.20 NavfnPlanner 源码](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_navfn_planner/src/navfn_planner.cpp)。以上数值和通过情况来自本项目保存的实测数据。

### 2026-11-21 更新：绕障恢复优化完成

本节是当前推荐配置，覆盖前文的 DWB 和 0.58 m 半径设置。启动命令仍为 `bash ~/bcr_ws/validation/start_navigation.sh`，默认加载 `nav2_demo.yaml`。

检查 81 秒轨迹后发现，小车曾在约 `(0.55,-0.65)` 附近反复转向、少量移动，随后多次报 Failed to make progress。把 DWB 的 BaseObstacle.scale 从 0.02 提到 0.1，并把进展检查半径从 0.5 m 改为 0.2 m，仍然复现：第二次绕障耗时 86.91 秒，action 反馈的恢复次数为 7。因此，这两项调整不足以解决问题。

进一步核对模型发现，0.9 × 0.64 m 车身之外还有四个球形脚轮，轮心位于 `(±0.45,±0.27)`，半径 0.06 m。覆盖这些脚轮的圆半径至少约为 `sqrt(0.45²+0.27²)+0.06 = 0.5848 m`，之前 0.58 m 的配置略小。旧的 81 秒轨迹中，车身矩形重叠采样为 0，但脚轮投影重叠采样为 9；86.91 秒对照中脚轮重叠采样为 11。这是车体范围遗漏的重要证据，尚不是接触传感器记录，也不能据此单独证明全部停顿都由碰撞造成。

当前配置作了以下变更：

- 局部控制器改用 Nav2 Regulated Pure Pursuit（RPP），全局规划器仍为 NavFn。DWB 完整对照副本保存为 `nav2_demo.dwb_comparison.yaml`。
- 全局和局部地图的 `robot_radius` 均修正为 0.60 m，膨胀半径仍为 0.90 m。
- RPP 期望线速度为 0.26 m/s，前瞻距离 0.40 m，启用碰撞检测、曲率减速和障碍距离减速，未关闭障碍检查。
- 进展检查仍为 10 秒内移动超过 0.20 m；AMCL 参数与上一节最终配置相同。

RPP 以路径跟踪为主，并提供速度调节和碰撞检查，参数依据 [Nav2 Humble 对应 1.1.20 官方说明](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_regulated_pure_pursuit_controller/README.md)。本轮替换用于对照和稳定基线，不意味着 DWB 在其他正确配置下无法工作。

RPP 在原 0.58 m 半径下先完成四目标对照，耗时 18.06、17.56、16.56、24.06 秒，恢复次数均为 0。修正半径为 0.60 m 并完整重启仿真、定位、导航后，最终验收结果如下。

| 顺序与目标 | 状态 | 实际耗时 | 恢复次数 | 最大采样位置偏差 |
|---|---|---|---|---|
| 1，`(2.2,0)` | 4，成功 | 17.81 秒 | 0 | 0.047 m |
| 2，`(0,0)` | 4，成功 | 17.81 秒 | 0 | 0.053 m |
| 3，`(2.2,0)` | 4，成功 | 16.96 秒 | 0 | 0.126 m |
| 4，`(0,1.4)` | 4，成功 | 24.01 秒 | 0 | 0.159 m |

每轮四项目标连续执行，中途不重置 AMCL。最终四段轨迹对车身矩形及四脚轮投影的离散几何检查，重叠采样均为 0。位置偏差使用 AMCL 消息与邻近时间戳的 world 里程计比较，仅接受时间差不超过 0.2 秒的样本，不能当作真实轮式里程计精度。最大偏差仍达约 16 cm；本轮证据支持简单静态场景下明显减少恢复，不能推广为定位漂移已完全消除或所有场景均稳定。

本轮变更了控制器及半径，未对修正为 0.60 m 后的 DWB 作独立重测，所以不能把全部提升只归因于控制器更换。尚未再次进行 Ubuntu 关机开机验收；动态障碍和复杂仓库仍需另测。

Ubuntu 和 Windows 的 `validation/`、`artifacts/bcr/` 分别保存以下文件：

- `nav2_demo.yaml`、`nav2_rpp.yaml`：当前配置。
- `nav2_demo.dwb_comparison.yaml`：DWB 对照配置，含历史 0.58 m 半径，恢复使用前应同步修正半径。
- `controller_fix_regression.json`、`controller_fix_navigation.log`：DWB 调参对照。
- `rpp_regression.json`、`rpp_navigation.log`：RPP 第一轮对照。
- `rpp_final_regression.json`、`rpp_final_navigation.log`：0.60 m 半径最终验收。
- `navigation_regression.py`：四目标测试，支持用第一个命令行参数指定输出 JSON 文件名。
- `summarize_navigation.py`：离散几何复核，生成 `*_summary.json`。脚轮及车身检查不覆盖所有附件，也不能代替连续碰撞检测。
- `restart_navigation.py`：仅重启本项目统一入口及其已识别子进程，供重复验收使用。

重复测试需先完整重启仿真，使机器人回到出生点；另一终端执行以下命令。无需手动发布速度或重置定位。

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=23
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
python3 ~/bcr_ws/validation/navigation_regression.py new_regression.json
```

### 2026-11-21 更新：Ubuntu 关机开机验收通过

用户将 Ubuntu 完全关机后重新开机并登录桌面。本次 SSH 连接时系统已运行约 2 分钟，没有已有的 ROS launch 或 Gazebo 仿真实例。直接运行保存的统一入口，未修改参数、重新建图或手动发布初始定位，AMCL 和 Nav2 自动激活。启动阶段有短暂等待 TF 的日志，随后正常进入导航。

运行配置的 SHA-256 与 Windows 保存副本一致，为 `917bba1313cab9e40925008b170a9cc63f7ae01c8733f9a5875141e80806109e`。AMCL 进程实际加载 `/home/geng/tf_fix_ws/install/tf2/lib/libtf2.so`，确认 TF 修复覆盖库在重启后仍有效。本次系统 boot ID 为 `5c831a32-cf86-45a9-873f-ea690ee8b667`。

| 连续目标 | 最终状态 | 实际耗时 | 恢复次数 | 最大采样位置偏差 |
|---|---|---|---|---|
| `(2.2,0)` | 4，成功 | 17.71 秒 | 0 | 0.029 m |
| `(0,0)` | 4，成功 | 17.96 秒 | 0 | 0.050 m |
| `(2.2,0)` | 4，成功 | 16.41 秒 | 0 | 0.048 m |
| `(0,1.4)` | 4，成功 | 23.86 秒 | 0 | 0.122 m |

四项目标之间没有重置定位。车身矩形及四脚轮投影与测试箱的离散几何重叠采样均为 0，证据边界同前节；本轮不能替代连续碰撞检测。定位偏差仍是与 world 里程计的采样对比，不是真实轮式里程计精度。

原始记录保存在 Ubuntu `~/bcr_ws/validation/` 及 Windows `artifacts/bcr/`：`cold_reboot_rpp_regression.json`、`cold_reboot_rpp_navigation.log`、`cold_reboot_rpp_environment.json`、`cold_reboot_rpp_regression_summary.json`。

至此，BCR 简单静态场景下的建图、保存地图、AMCL 定位、Nav2 绕障到点、进程整套重启及一次 Ubuntu 完全关机开机复现均已通过。此前“尚未再次关机开机验收”的条目是历史状态。当前可作为下一阶段自建机器人适配的基线；复杂仓库、动态障碍、真实轮式里程计和 TF 修复库完整并发回归仍需另行验证。本次测试结束后机器人停在末项目标附近，仿真和导航服务保持运行。
