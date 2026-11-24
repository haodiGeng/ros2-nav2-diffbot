# BCR Bot 最小仿真验收

2026-11-19 在 VMware Ubuntu 22.04 / Humble / Fortress 环境验证通过。上游 ros2 分支提交为 `e022fa0ce466bf8faec636454e90fbffa585a718`。

## 运行配置

Ubuntu 工作空间为 `/home/geng/bcr_ws`。验证入口为 `validation/minimal.launch.py`，场景为 `validation/minimal.sdf`。启动无 GUI 仿真，关闭相机和双目，启用 2D 激光。当前使用上游默认的 world 里程计，后续轮式里程计和真实定位误差评测仍需单独验证。

ROS_DOMAIN_ID=23，IGN_PARTITION=bcr_m2。Ubuntu 桌面终端可按下列方式启动，先确保没有同一场景旧进程。

```bash
source /opt/ros/humble/setup.bash
source ~/bcr_ws/install/setup.bash
export ROS_DOMAIN_ID=23
export IGN_PARTITION=bcr_m2
export LIBGL_ALWAYS_SOFTWARE=1
export IGN_GAZEBO_RESOURCE_PATH="$HOME/bcr_ws/install/bcr_bot/share:$HOME/bcr_ws/install/bcr_bot/share/bcr_bot/models"
ros2 launch ~/bcr_ws/validation/minimal.launch.py
```

SSH 启动需采用当前桌面的 DISPLAY 和 XAUTHORITY，路径可能随桌面登录变化。

## 实测结果

| 项目 | 结果 |
|---|---|
| 首轮 30 秒采样 | 仿真时间 22.482 → 52.494 秒，910 帧激光，294 帧里程计 |
| 激光内容 | frame_id 为 two_d_lidar，有效距离约 1.001～4.121 米 |
| TF | 里程计坐标系到扫描帧可查询 |
| 运动 | 0.1 m/s 低速测试实际前进约 0.0907 米，随后发送零速度 |
| 两分钟连续运行 | 120.00 秒现实时间推进 119.982 秒仿真时间，收到 3636 帧激光 |
| 扫描连续性 | 最大相邻扫描时间戳间隔 0.033 秒 |
| 停车 | 两分钟首末里程计位置差 0 米 |

两分钟测试中未出现之前的时钟停滞。日志仍有 EGL 警告，但本轮时间、扫描和运动实测正常。简单场景结果不能代表复杂仓库下的性能。

下一步用 BCR Bot 的话题与 TF 配置进行 SLAM 建图，再验证 AMCL、Nav2 到点和绕障。Windows 启动与场景副本位于 `artifacts/bcr/`。
