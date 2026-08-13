# volleyball_predict — 排球轨迹预测包

## 概述

`volleyball_predict` 负责对追踪得到的排球运动状态进行**未来轨迹外推**。它基于**二次空气阻力物理模型**，使用**四阶龙格-库塔法（RK4）** 进行数值积分，预测排球从当前位置到落地之间的完整飞行轨迹，为轨迹规划提供依据。

## 架构

```
/tracker/target (追踪状态) → PredictNode → /predictor/ball_trajectory (预测轨迹)
                              ↓
                         /predictor/marker (RViz可视化)
```

## 状态判断逻辑

预测器通过球的位置与速度的**点积** `x·vx` 判断球的运动方向：

- **`TO_OWN_SIDE`（飞向我方）**: `dot < 0` → 球正朝机器人方向飞来 → **执行轨迹预测**
- **`TO_OPPONENT_SIDE`（飞向对方）**: `dot > 0` → 球正远离机器人 → **不发布预测轨迹**
- **`TO_STOP`（静止）**: `dot == 0` → 球静止 → **不发布预测轨迹**

> 仅在球飞向我方时预测，是因为规划器需要利用落点坐标计算机器人移动目标；球飞向对方时预测没有意义。

## 节点

### `predict_node` (主节点)

#### 订阅话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/tracker/target` | `volleyball_interfaces/msg/Ball` | 追踪滤波后的排球状态（含速度） |

#### 发布话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/predictor/ball_trajectory` | `volleyball_interfaces/msg/BallTrajectory` | 预测的飞行轨迹点序列 |
| `/predictor/marker` | `visualization_msgs/MarkerArray` | (调试)RViz 可视化 Marker（轨迹点 + 落点标记） |

#### 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `predict_timer_freq` | `40` | 预测发布频率 (Hz) |
| `predict_step` | `0.05` | RK4 积分步长 (s) |
| `odom_frame_id` | `odom` | 世界坐标系名称 |
| `k` | `0.03` | 二次空气阻力系数 |
| `g` | `9.80` | 重力加速度 (m/s²) |
| `lost_time_thres` | `3.0` | 丢检后停止预测的时间阈值 (s) |

## 核心类

### `Predictor` — 预测器算法类

位于 `include/volleyball_predict/predictor.hpp`

#### 物理模型

采用**二次空气阻力模型**：

```
F_drag = -k·v·|v|

运动方程:
d[px, vx, py, vy, pz, vz]ᵀ/dt = [vx, -k·vx·|v|, vy, -k·vy·|v|, vz, -k·vz·|v| - g]ᵀ
```

其中 `|v| = √(vx² + vy² + vz²)`。

#### RK4 数值积分 (`RK4`)

由于二次阻力模型无解析解，使用经典四阶龙格-库塔法求解：

```
k1 = f(x)
k2 = f(x + 0.5·dt·k1)
k3 = f(x + 0.5·dt·k2)
k4 = f(x + dt·k3)
x(t+dt) = x + (k1 + 2·k2 + 2·k3 + k4)·dt/6
```

#### 轨迹预测流程 (`predict`)

1. 接收球当前状态（位置、速度、时间戳）
2. 以当前状态为第一个轨迹点存入
3. 以 `predict_step` 为步长循环执行 RK4 积分
4. 每次积分后检查 `pz`（高度），当 `pz < 0`（球落地）时停止
5. 返回完整的 `BallTrajectory` 消息（含所有未来轨迹点）

### `PredictNode` — 预测节点

位于 `include/volleyball_predict/predictor_node.hpp`

#### 丢检处理

追踪消息缓存 `ball_cache_` 的时间戳与当前时间比较：
- `Δt > lost_time_thres` → 清空缓存，停止发布轨迹
- 下次收到追踪消息自动恢复预测

#### 丢检预测机制

当追踪短暂丢失时，如果球正飞向我方，预测器仍基于最后收到的状态持续发布预测轨迹，确保短时间内轨迹规划不会中断。

## (调试)可视化

预测器在 RViz 中发布两种 Marker：

| Marker | ID | 类型 | 颜色 | 说明 |
|--------|-----|------|------|------|
| 轨迹点 | 0~N | `SPHERE` (半径 0.08) | 红色+蓝色 (品红) | 每个预测轨迹点 |
| 落点 | 9999 | `CUBE` (0.2×0.2×0.05) | 绿色 | 球落地的地面位置 |

所有 Marker 的生命周期为 5 秒，固定帧为 `odom_frame_id`。

## Launch 文件

```
volleyball_predict.launch.py → predict_node (加载 predict.yaml 参数)
```

## 依赖

- `rclcpp` — ROS2 C++ 客户端库
- `Eigen3` — 线性代数库
- `geometry_msgs` — 几何消息类型
- `volleyball_interfaces` — 自定义消息（Ball, BallTrajectory）
- `visualization_msgs` — RViz 可视化 Marker

## 数据流

```
/tracker/target (含 vx, vy, vz)
       │
       ▼  方向判断 (dot = x·vx)
       │
  TO_OWN_SIDE ──→ Predictor.predict() ──→ /predictor/ball_trajectory
       │                                        │
 其他方向 ──→ 跳过预测，不发布                  ↓
                                        volleyball_plan (轨迹规划)
```
