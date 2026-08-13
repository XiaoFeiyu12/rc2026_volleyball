# volleyball_plan — 排球击球规划包

## 概述

`volleyball_plan` 负责根据预测的排球轨迹计算机器人的**最优拦截击球位置**。它从预测轨迹的落点开始**反向搜索**，结合机器人机械臂的物理尺寸约束，找到机器人可以去到并能成功击球的最优目标点，输出机器人的移动坐标和击球时间。

## 架构

```
/predictor/ball_trajectory (预测轨迹) → PlannerNode → /planner/plan (击球规划)
                                                       ↓
                                                  /planner/marker (RViz可视化)
```

## 节点

### `planner_node` (主节点)

#### 订阅话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/predictor/ball_trajectory` | `volleyball_interfaces/msg/BallTrajectory` | 预测的排球飞行轨迹 |

#### 发布话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/planner/plan` | `volleyball_interfaces/msg/Plan` | 机器人的目标位置和击球时间 |
| `/planner/marker` | `visualization_msgs/MarkerArray` | (调试)RViz 可视化 Marker（目标点方向箭头） |

#### 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `base_to_armjoint` | `0.18` | 机器人底盘中心到机械臂关节底座的距离 (m) |
| `armjoint_to_strikingpoint` | `0.25` | 机械臂关节到有效击球点的距离 (m) |
| `delta_arm_angle` | `33.0` | 机械臂工作方向与竖直方向的夹角 (度) |

## 核心类

### `Planner` — 规划算法类

位于 `include/volleyball_plan/planner.hpp`

#### 规划算法 — 反向搜索

`plan()` 函数的执行流程：

1. **输入检查**: 如果接收到的轨迹为空（无球消息），直接返回空 Plan
2. **初始化**: 设置搜索方向向量 `diff = [0, 0, -1]ᵀ`（指向地面）
3. **反向遍历**: 从轨迹的最后一个点（落点）开始向前遍历
   - 对每个轨迹点，计算 `diff = 轨迹点位置 - 击打点偏移量(striking_bias)`
   - 当 `diff.z > 0` 时停止（找到地面以上的第一个可击打点）
4. **输出计算**:
   - `plan.x` = 轨迹点 X 坐标 + 击打点偏移量 X
   - `plan.y` = 轨迹点 Y 坐标 + 击打点偏移量 Y
   - `plan.landing_time` = 轨迹点时间戳到当前时间的剩余时间(s)
   - `plan.self_yaw` = 固定为 0（目前直接面对对方击球，不考虑角度）

#### 击打点偏移量计算

`striking_point_bias_` 在 `PlannerNode` 构造函数中计算：

```cpp
striking_point_bias_ = [L·sin(θ), 0, L·cos(θ) + H]ᵀ
```

其中：
- `L = armjoint_to_strikingpoint` — 机械臂到击球点的距离
- `θ = delta_arm_angle` — 机械臂工作角度
- `H = base_to_armjoint` — 底盘到关节的偏移

这个偏移量定义了**机器人能够成功击球的包络空间**——只有当球进入这个空间时，机器人才有能力拦截并击打。

## 数据结构

### `Plan` 消息定义 (`volleyball_interfaces/msg/Plan`)

```msg
float32 x          # 目标 X 坐标
float32 y          # 目标 Y 坐标
float32 self_yaw   # 机器人朝向角 (当前固定为 0)
float32 landing_time  # 距离球到达目标点的剩余时间 (s)
```

## (调试)可视化

规划器在 RViz 中发布一个方向箭头 Marker：

| Marker | 类型 | 颜色 | 说明 |
|--------|------|------|------|
| 目标点 | `ARROW` | 蓝色 | 从地面目标位置指向击球方向 |

箭头从 `(plan.x, plan.y, 0.0)` 出发，沿 `self_yaw` 方向延伸 1 米。生命周期为 1 秒。

## Launch 文件

```
volleyball_plan.launch.py → planner_node (加载 plan.yaml 参数)
```

## 依赖

- `rclcpp` — ROS2 C++ 客户端库
- `Eigen3` — 线性代数库
- `volleyball_interfaces` — 自定义消息（BallTrajectory, Plan）
- `visualization_msgs` — RViz 可视化 Marker

## 数据流

```
/predictor/ball_trajectory (预测轨迹点序列)
       │
       ▼  反向搜索：从落点向前找到可击打点
       │
/planner/plan (x, y, landing_time)
       │
       ▼
volleyball_robot (机器人底盘运动控制)
```
