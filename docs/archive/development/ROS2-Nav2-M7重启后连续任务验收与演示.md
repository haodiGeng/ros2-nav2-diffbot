# M7 重启后连续任务验收与演示

后续更新：本页保留当时的失败证据。反向通行已完成修复及两轮完整六步复测，见 [反向通行卡顿修复与连续验收](ROS2-Nav2-M7反向通行卡顿修复与连续验收.md)。当前 M7 使用 Rotation Shim＋DWB，定位精度限制仍保留。

2026-11-24，Ubuntu 本次开机后完成连续任务验证和演示入口整理。结论为：**绕障、返回、正向宽通道、窄通道拒绝及拒绝后继续接单的流程通过；反向返回宽通道仍出现 DWB 进度不足，因此 M7 全场景稳定验收尚未全部通过。**

Ubuntu 报告的启动时间为 2026-11-24 09:47:53，boot ID 为 `1c97b420-ffbf-4607-88f2-fa4a403b9cb0`。本轮是在该次启动之后测试，没有为每个目标重新启动仿真或重置定位，也没有由脚本主动重启 Ubuntu。

## 场景与任务

新场景沿用已有机器人和 M7 门控配置，外墙内侧为 ±4 m，障碍箱在 (0,1.2)，左右分别有 0.8 m 与 1.4 m 开口。使用与几何配套的分析地图，分辨率 0.05 m。此地图用于验收，不替代 M5 原地图。额外净间隙仍为 0.15 m，DWB 前视范围仍为 0.6 m，没有修改规划器、控制器或安全约束。

完成的五步连续序列：

| 顺序 | 任务与目标 | 结果 | 耗时 s | 恢复次数 |
|---|---|---|---:|---:|
| 1 | 绕障到 (0,2.6,0) | 成功 | 33.15 | 0 |
| 2 | 返回 (0,0,0) | 成功 | 25.37 | 0 |
| 3 | 穿过宽通道到 (3.3,0,0) | 成功 | 21.65 | 0 |
| 4 | 请求穿窄通道到 (-3.3,0,0) | 预检拒绝，未发送导航目标 | 1.51 | 0 |
| 5 | 随后到右侧房间 (3.3,-0.8,0) | 成功 | 14.73 | 0 |

所有步骤复用同一仿真、同一套 AMCL/Nav2 和同一个任务入口实例，没有重新发布初始定位。第 4 步返回 ABORTED、路径为空；第 5 步证明拒绝后仍能处理新任务。沿真值位置按半径 0.35 m 的保守圆形模型核算，整个序列相对场景矩形障碍的最小采样余量约 0.205 m；这不是连续时间碰撞证明。

## 必须保留的失败序列

首轮包含额外任务：首次穿过宽通道后，再返回 (0,0)。前三步成功，第四步在约 16.65 秒后因 `Failed to make progress` 返回 ABORTED，恢复次数为 0。日志显示失败来自 DWB 跟踪进度检查，不是规划器拒绝起点。门控树按设计结束任务，没有执行恢复旋转。

该失败保存在 `artifacts/m7_continuous/`。后续五步验证保存在独立的 `artifacts/m7_continuous_v2/`，没有覆盖失败数据。第二组省去了反向穿宽通道步骤，因此**不能将第二组通过解释为反向通行问题已修复**。反向行驶姿态、局部评分与累计定位偏差的影响仍需进一步记录和对照。

## 手动演示入口

在 Ubuntu 桌面终端中，确认没有其他自建仿真或导航任务运行，再执行：

```bash
bash ~/nav_ws/src/my_robot_navigation/scripts/start_m7_demo.sh
```

默认打开 RViz，Gazebo 服务端在后台运行；需要 Gazebo GUI 时加 `gui:=true`，无界面运行加 `rviz:=false gui:=false`。本轮正式任务均为无 GUI 验证，未进行本次 GUI 显示验收。入口已做安装、Shell 语法与 launch 参数检查。

第二个终端逐条执行；每条完成后再运行下一条：

```bash
bash ~/nav_ws/src/my_robot_navigation/scripts/start_safe_navigation.sh 0 2.6 0
bash ~/nav_ws/src/my_robot_navigation/scripts/start_safe_navigation.sh 0 0 0
bash ~/nav_ws/src/my_robot_navigation/scripts/start_safe_navigation.sh 3.3 0 0
bash ~/nav_ws/src/my_robot_navigation/scripts/start_safe_navigation.sh -3.3 0 0
bash ~/nav_ws/src/my_robot_navigation/scripts/start_safe_navigation.sh 3.3 -0.8 0
```

第四条退出码 2 是预期的拒绝；其余成功时为 0。不要把以上命令用 `&&` 连接，否则第四条的预期非零退出码会阻止第五条执行。RViz 的默认 Nav2 Goal 按钮不等同于此门控入口。

演示脚本只启动一套场景，不自动发送目标。如果缺少场景文件，会调用 `continuous_acceptance.py --prepare-only` 生成世界和地图。启动终端 Ctrl+C 后退出场景。

## 数据与复测

自动五步复测使用：

```bash
source /opt/ros/humble/setup.bash
source ~/nav_ws/install/setup.bash
source ~/tf_fix_ws/install/setup.bash
python3 ~/nav_ws/src/my_robot_navigation/scripts/continuous_acceptance.py
```

脚本完成后关闭仿真，会覆盖 m7_continuous_v2 中的同名测试记录，复测前先备份。results.json 包含任务状态、耗时和真值轨迹；audit.json 保存本次启动信息及采样余量。源码和数据已同步 Windows 项目。

当前应继续排查“宽通道反向返回”的连续任务卡顿，再决定 M7 是否满足最终验收条件。此前普通窄通道及旧地图定位精度的限制也仍然保留。
