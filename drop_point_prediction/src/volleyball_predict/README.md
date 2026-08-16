# volleyball_predict

排球轨迹预测包，作为流水线的第三环。订阅跟踪结果，基于线性阻力模型对球状态进行外推，直到球落地，打包成完整轨迹发布给下游规划。

## 功能

- 订阅 `/tracker/target` 滤波后的球状态（位置 + 速度）。
- 定时对球状态进行外推，逐步积分直到球高度降到地面以下，得到完整轨迹。
- 检测丢检：缓存球消息超时则认为丢检，停止预测。
- 发布轨迹与落点可视化。

## ROS 接口

节点名：`predict_node`

### Subscription

| 话题 | 消息类型 | 说明 |
|------|----------|------|
| `/tracker/target` | `volleyball_interfaces/msg/Ball` | 滤波后的球状态（位置 + 速度） |

### Publisher

| 话题 | 消息类型 | 说明 |
|------|----------|------|
| `/predictor/ball_trajectory` | `volleyball_interfaces/msg/BallTrajectory` | 预测的完整轨迹（含落点） |
| `/predictor/marker` | `visualization_msgs/msg/MarkerArray` | 轨迹点与落点可视化 |

### Parameter

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `predict_timer_freq` | int | `40` | 预测定时频率（Hz） |
| `odom_frame_id` | string | `odom` | 世界坐标系名称 |
| `predict_step` | double | `0.1` | 积分步长（s） |
| `k` | double | `0.10` | 空气阻力系数 |
| `m` | double | `0.27` | 排球质量（kg） |
| `g` | double | `9.80` | 重力加速度 |
| `lost_time_thres` | double | `3.0` | 丢检超时阈值（s） |

## 原理

> 待补充：以下为框架，细节请自行填充。

### 轨迹外推

- （待补充）线性阻力模型的解析积分公式，与 tracker 的过程模型一致。
- （待补充）从当前状态迭代到球落地（z < 0）的循环。

### 丢检处理

- （待补充）缓存球状态与超时判定。
