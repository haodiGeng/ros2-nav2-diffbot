# A* 全局规划器

`my_planner` 提供独立栅格搜索内核和 Nav2 `GlobalPlanner` 插件。实现目标是让连通性、对角移动规则、代价权重和额外净间距可以分别控制，便于参数实验。

## 代码结构

| 文件 | 职责 |
|---|---|
| [astar_core.hpp](../src/my_planner/include/my_planner/astar_core.hpp) | 栅格、搜索选项和结果接口 |
| [astar_core.cpp](../src/my_planner/src/astar_core.cpp) | A*、距离场与线段检查 |
| [astar_planner.cpp](../src/my_planner/src/astar_planner.cpp) | Nav2 适配、地图快照、路径与统计发布 |
| [插件描述](../src/my_planner/astar_planner_plugin.xml) | pluginlib 注册 |
| [内核测试](../src/my_planner/test/test_astar_core.cpp) | 14 个 gtest 用例 |

## 搜索规则

代价地图中 0～252 可通行，253、254 不可通行；255 根据未知区域策略处理。起终点越界、被阻挡、无路径或超时均返回失败。插件不会通过清除起点障碍或移动目标位置来取得成功结果。

4 连通搜索使用曼哈顿启发；8 连通使用 octile 启发。对角移动要求两个相邻正交格也可通行。返回路径的所有连接段还经过 supercover 检查，包括触及角点或栅格边界的格子。

相邻格 u、v 的边代价为：

```text
edge(u, v) = length(u, v) × [1 + cost_weight × (C(u) + C(v)) / (2 × 252)]
```

`length` 为直移或对角移动距离，允许未知区域时以 `unknown_cost` 替代未知格的 C。非负权重下，启发函数为该离散图的代价下界。这不代表连续空间最短路径，也不代表机器人最短行驶时间。

## 参数

参数以规划器别名为前缀，在插件初始化时读取。

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `use_8_connected` | true | true 为 8 连通，false 为 4 连通 |
| `allow_unknown` | false | 是否允许未知区域 |
| `unknown_cost` | 128 | 允许未知区域时使用的栅格代价 |
| `cost_weight` | 2.0 | 可通行格代价的权重，要求非负 |
| `min_obstacle_clearance` | 0.0 | 额外净间距，单位米；0 表示不开启额外筛选 |
| `max_search_time` | 1.0 | 搜索时间预算，单位秒 |

权重影响路径的软偏好，不保证精确的离墙距离。开启额外净间距后，距离场使用致命格和按策略拒绝的未知格作为障碍，并采用保守的格面积距离下界；地图边界也纳入约束。该筛选会拒绝部分实际可通过的格子。

净间距使用机器人外接圆和地图快照，不替代定位、局部碰撞检查或连续碰撞验证。

## Nav2 配置示例

```yaml
planner_server:
  ros__parameters:
    planner_plugins: [GridBased]
    GridBased:
      plugin: my_planner/AStarPlanner
      use_8_connected: true
      allow_unknown: false
      unknown_cost: 128
      cost_weight: 2.0
      min_obstacle_clearance: 0.0
      max_search_time: 1.0
```

仓库中的 `GridBased` 是普通 A*，`ClearanceGate` 是额外间距为 0.15 m 的独立别名；`NavFnBaseline` 用于对照。完整配置见 [导航配置](../src/my_robot_navigation/config/nav2_astar_params.yaml)。

规划器在锁内复制代价地图，释放锁后搜索。输出保留实际起终点，内部路径使用格中心；当前不做路径平滑。统计 JSON 发布到 `/planner_server/<别名>/statistics`，包含展开节点数、搜索时间等。

## 任务门控与接口边界

[任务入口](../src/my_robot_navigation/scripts/safe_navigate.py) 检查新鲜里程计、空闲状态和预检路径，并使用保持 `ClearanceGate` 的专用无恢复行为树。它是单调用者场景的任务入口，不是全局速度仲裁器，也不是急停系统。

插件采用当前 Humble 的两参数 `createPlan(start, goal)` 接口。内核支持取消回调，ROS 适配层只接入 shutdown 检查并限制搜索时间，没有提供每个 action 请求的即时取消回调。

NavFn 也会受代价地图膨胀配置影响，且已有对角夹缝实验没有复现 NavFn 穿角。自研插件的规则与可控性不能替代针对具体基线的实测比较。结果见 [评测说明](evaluation.md)。
