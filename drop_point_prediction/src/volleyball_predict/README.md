# volleyball_predict

排球轨迹预测包，作为流水线的第三环。订阅跟踪结果，基于线性阻力模型对球状态进行外推，直到球落地，打包成完整轨迹发布给下游规划，并发布可视化结果。

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
这里相当于上一环track包卡尔曼滤波中无噪声输入情况下的预测外推，程序将外推结果的每一个轨迹点打包，循环求解至球坐标z<0得到完整轨迹，随后打包发送给下游规划器。其中外推积分过程原理同[track包的过程模型](../volleyball_track/README.md)。这里不再赘述。

## 补充与优化点
其实细想一下这里完成的工作更像是将上一环状态估计的结果收尾，其实可以直接合并回track包，没必要单独开一包，从而降低ros2通信延迟。

