# volleyball_plan

接球规划包，作为流水线的最后一环。订阅预测轨迹，结合机器人机械臂安装位置，从落点反向寻找可击球点，计算接球位姿并发布。

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
这里由 `base_to_armjoint`、`armjoint_to_strikingpoint`、`delta_arm_angle` 计算击球点相对底盘坐标系的偏置向量。  
然后从轨迹落点反向遍历，寻找减去击球点偏置后仍在地面以上的轨迹点。  
找到轨迹点后将该轨迹点的平面坐标作为规划坐标，将该点时间戳减去当前时间戳和通信延迟得到`landing_time`,最后作为规划结果发给串口包。

## 补充与改进点
1. 如你所见这个包写的挺潦草的，也是因为实测下来发现纯自动的情况下，尤其是击球盘设计的还小的情况下，以及底盘响应、机械刚度、测量精度、计算延迟等种种因素类加，很难完美做到碰到球，也导致很少心思花在优化plan包上。
2. 这里当时应该把击球偏置点的获取改为从tf树获取，而不是硬编码。
3. 包括从上游得到落点，很多时候是波动的，经常会出现规划点一会儿前一会儿后，应该考虑加入MPC作为优化规划指令的一环，避免下位机底盘，因为飘忽不定的规划点和惯性而抽搐无法流畅运动。
