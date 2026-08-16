# volleyball_ibvs

IBVS 反应式控制器，作为上位机算法流水线的第二环（接在 detector 之后）。订阅 detector 发布的排球 3D 位置，经 TF 变换到击球点坐标系，通过低通滤波 + PD/P 控制生成横向 / 纵向速度指令并发布给下位机。

## ROS 接口

节点名：`ibvs_controller_node`

### Subscription

| 话题 | 消息类型 | 说明 |
|------|----------|------|
| `/detector/ball` | `volleyball_interfaces/msg/Ball` | 排球 3D 位置（`x/y/z`，`camera_color_optical_frame`） |

### Publisher

| 话题 | 消息类型 | 说明 |
|------|----------|------|
| `/pid_camera` | `volleyball_interfaces/msg/PidCamera` | 速度指令 + 击球标志 |

`PidCamera` 字段说明：

| 字段 | 类型 | 说明 |
|------|------|------|
| `dx` | float32 | 横向速度指令（cm/s），对应 `vy_cmd` |
| `dy` | float32 | 纵向速度指令（cm/s），对应 `vx_cmd` |
| `is_hit` | bool | 击球标志（球进入击球窗口时为 `true`） |

### Parameter

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `Kp_lat` | float | `1.0` | 横向位置比例增益 |
| `Kd_lat` | float | `0.5` | 横向差分（微分）增益 |
| `Kp_long` | float | `0.8` | 纵向距离比例增益，单位 (cm/s)/m |
| `alpha` | float | `0.4` | 低通滤波系数（越小平滑效果越好） |
| `timeout` | float | `0.5` | 看门狗超时时间（s） |
| `target_frame` | string | `base_link` | 目标坐标系（TF 变换目标帧，即击球点坐标系） |
| `dist_offset` | float | `0.5` | 纵向距离补偿（m） |
| `hit_offset_x` | float | `0.2` | 击球窗口 X 轴范围（m） |
| `hit_offset_y` | float | `0.2` | 击球窗口 Y 轴范围（m） |
| `hit_offset_z` | float | `0.2` | 击球窗口 Z 轴范围（m） |

## 原理

### 坐标变换（TF2）

球位置由 detector 以 `camera_color_optical_frame` 发布，经 TF2 变换到 `target_frame`（击球点坐标系）。变换后取：
- `dist = x + dist_offset`：纵向（前进方向）距离这里说说`dist_offset`,因为机器人总是冲过来之后停在排球正前方一定距离，为了让排球完全接触击球盘而加上的补偿，测试时注意安全；
- `y = -y`：横向偏移（符号翻转以匹配控制方向约定）。

### 控制律

```
横向: vy_cmd = Kp_lat * y + Kd_lat * Δy   (PD)
纵向: vx_cmd = Kp_long * dist             (P)
```

输出单位为 cm/s，分别写入 `PidCamera.dx`（横向）与 `PidCamera.dy`（纵向）。

### 击球判定

当球在击球窗口内时置 `is_hit`：

```
0 < x < hit_offset_x 且 |y| < hit_offset_y 且 |z| < hit_offset_z
```

此时便会向下位机发送击打指令

## 补充
整体思路源于论文，当然论文比我这个复杂多了，我只是在时间紧迫关头勉强简单做做。