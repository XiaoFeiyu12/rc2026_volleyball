# volleyball_plan

接球规划包，作为流水线的最后一环。订阅预测轨迹，结合机器人机械臂安装位置，从落点反向寻找可击球点，计算接球位姿并发布。

## 功能

- 订阅 `/predictor/ball_trajectory` 预测轨迹。
- 根据机械臂基座与击球点的相对安装关系（基座偏移、臂长、臂角）计算击球点相对底盘的偏置向量。
- 从轨迹落点反向搜索机械臂能够击中的轨迹点，计算规划位姿与到达时间。
- 发布规划结果与可视化。

## ROS 接口

节点名：`planner_node`

### Subscription

| 话题 | 消息类型 | 说明 |
|------|----------|------|
| `/predictor/ball_trajectory` | `volleyball_interfaces/msg/BallTrajectory` | 预测的球轨迹 |

### Publisher

| 话题 | 消息类型 | 说明 |
|------|----------|------|
| `/planner/plan` | `volleyball_interfaces/msg/Plan` | 接球规划位姿（`x/y/self_yaw/landing_time`） |
| `/planner/marker` | `visualization_msgs/msg/MarkerArray` | 规划位姿可视化 |

### Parameter

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `base_to_armjoint` | double | `0.18` | 底盘中心到机械臂关节的高度偏置（m） |
| `armjoint_to_strikingpoint` | double | `0.25` | 机械臂关节到击球点的长度（m） |
| `delta_arm_angle` | double | `33.0` | 机械臂击球角度（°） |

## 原理

> 待补充：以下为框架，细节请自行填充。

### 击球点偏置

- （待补充）由 `base_to_armjoint`、`armjoint_to_strikingpoint`、`delta_arm_angle` 计算击球点相对底盘坐标系的偏置向量。

### 接球点搜索

- （待补充）从轨迹落点反向遍历，寻找减去击球点偏置后仍在地面以上的轨迹点。

### 规划输出

- （待补充）`Plan` 消息各字段（`x/y/self_yaw/landing_time`）的计算方式。
