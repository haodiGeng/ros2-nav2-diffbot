# 第三方组件说明

本项目 ROS 包采用 Apache-2.0，声明见各自 `package.xml`。ROS 2、Nav2、Gazebo、BCR Bot 等依赖与参考项目保留各自许可证；来源与接入过程保存在开发记录归档中。

支持源码 [geometry2-patched-source.tar.gz](support/tf2/geometry2-patched-source.tar.gz) 来源于 https://github.com/ros2/geometry2 ，上游提交为 `9bc9e89285515c8981ecd829c9400807372468c4`。其中的 LICENSE 为 BSD 3-Clause，版权与许可证原文随源码快照保留。

该快照已经应用本项目运行环境所用的 tf2 回调解锁补丁，差异见 [tf2-callback-unlock.patch](support/tf2/tf2-callback-unlock.patch)。不要对已打补丁的快照再次应用补丁。补丁用于已有仿真环境的复现，未完成全面并发正确性验证，也未宣称已被上游接受。
