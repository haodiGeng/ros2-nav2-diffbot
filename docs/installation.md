# 安装指南

## 环境要求

已验证组合为 Ubuntu 22.04.5、ROS 2 Humble、Gazebo Fortress 6.18.0、Nav2 1.1.20，以及 Cyclone DDS。实际系统包版本见 [环境清单](../support/environment/dpkg-versions.tsv)。

以下命令假设 ROS 2 Humble 及对应软件源已配置，项目安装于 `~/nav_ws`，tf2 覆盖工作空间位于 `~/tf_fix_ws`。现有启动脚本使用这两个路径；使用其他路径需要相应修改脚本。

## 1. 安装依赖

```bash
sudo apt update
sudo apt install git ros-humble-ros-gz ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-nav2-rotation-shim-controller ros-humble-slam-toolbox \
  ros-humble-rmw-cyclonedds-cpp python3-colcon-common-extensions \
  python3-rosdep python3-numpy python3-yaml python3-matplotlib
```

## 2. 获取源码

在目标目录尚不存在的环境中执行：

```bash
git clone https://github.com/haodiGeng/ros2-nav2-diffbot.git ~/nav_ws
```

也可以解压发布包，使四个 ROS 包位于 `~/nav_ws/src/`。已有工作空间应先备份，不直接覆盖正在使用的文件。

## 3. 初始化 rosdep

在尚未初始化 rosdep 的新系统中执行：

```bash
sudo rosdep init
```

之后更新依赖索引：

```bash
rosdep update
```

## 4. 构建 tf2 覆盖库

当前已验证环境依赖 tf2 回调解锁补丁。仓库提供 [已应用补丁的源码快照](../support/tf2/geometry2-patched-source.tar.gz)、[补丁差异](../support/tf2/tf2-callback-unlock.patch) 和 [上游提交](../support/tf2/geometry2-upstream-commit.txt)。

在新的覆盖工作空间中执行：

```bash
mkdir -p ~/tf_fix_ws/src
tar -xzf ~/nav_ws/support/tf2/geometry2-patched-source.tar.gz -C ~/tf_fix_ws/src
source /opt/ros/humble/setup.bash
cd ~/tf_fix_ws
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --packages-select tf2 --cmake-args -DBUILD_TESTING=OFF
```

源码快照已经带补丁，不再执行 `git apply`。只编译 `tf2` 包，其余依赖沿用系统安装。该补丁的全面并发语义验证尚未完成，不应将其视为已经被上游接受的通用修复。

## 5. 编译项目

```bash
source /opt/ros/humble/setup.bash
source ~/tf_fix_ws/install/setup.bash
cd ~/nav_ws
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --symlink-install
source ~/nav_ws/install/setup.bash
source ~/tf_fix_ws/install/setup.bash
```

项目包含 `my_robot_description`、`my_robot_gazebo`、`my_robot_navigation` 和 `my_planner` 四个包。

## 6. 运行内核测试

```bash
cd ~/nav_ws
colcon test --packages-select my_planner
colcon test-result --verbose
```

A* 内核包含 14 个 gtest 用例。colcon 的汇总还可能包含外层测试记录，不能直接用其总数替代 gtest 用例数。

## 构建与验证边界

源码已在同一 Ubuntu 虚拟机的独立目录完成四包构建、单元测试和绕障导航检查，未进行全新虚拟机从零安装的完整验证。

编译时可能出现系统 tf2 与覆盖库之间的运行搜索路径约束提示。已记录的进程映射确认实际加载了预期覆盖库；运行前仍需加载上面的环境。环境问题见 [常见问题](troubleshooting.md)。

安装完成后，按 [使用指南](usage.md) 启动导航。
