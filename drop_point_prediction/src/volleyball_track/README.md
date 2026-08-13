# volleyball_track — 排球追踪滤波包

## 概述

`volleyball_track` 负责对检测到的排球位置进行**滤波平滑与状态估计**。它实现了**扩展卡尔曼滤波（EKF）**，基于带线性空气阻力的物理运动模型预测排球位置与速度，并利用检测值进行修正，从而抑制检测跳变、填补短暂丢帧，输出稳定的排球运动状态。

## 架构

```
/detector/ball (检测) → 坐标系变换(TF2) → EKF 滤波 → /tracker/target (追踪)
                          ↓                        ↓
                  丢检自检定时器              RViz 可视化 Marker
```

## 核心流程

```
DETECTING ──(积累 detect_cnt_tres 帧)──→ TRACKING ──(检测到来)──→ TRACKING
                                            │
                                     (丢失超过 lost_time)
                                            ↓
                                         LOST ──(检测恢复)──→ TRACKING
```

1. **`DETECTING`（检测确认阶段）**: 接收到检测帧后计数，累积超过 `detect_cnt_tres` 帧后初始化滤波器并进入 `TRACKING`
2. **`TRACKING`（正常跟踪阶段）**: 每收到检测帧执行 ①预测 (`EKF::predict`) → ②更新 (`EKF::update`)
3. **`LOST`（丢检阶段）**: 当超过 `lost_time` 秒未收到检测数据，进入丢失状态。恢复检测后重新初始化滤波器

## 节点

### `tracker_node` (主节点)

#### 订阅话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/detector/ball` | `volleyball_interfaces/msg/Ball` | 检测到的排球位置（相机坐标系） |

#### 发布话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/tracker/target` | `volleyball_interfaces/msg/Ball` | 滤波后的排球状态（包含位置 + 速度，odom 坐标系） |
| `/tracker/marker` | `visualization_msgs/MarkerArray` | (调试)RViz 可视化 Marker（位置点、速度箭头、球体） |

#### 服务

| 服务 | 类型 | 说明 |
|------|------|------|
| `/tracker/reset` | `std_srvs/srv/Trigger` | 重置追踪器状态为 `DETECTING` |

#### 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `k` | `0.20` | 空气阻力系数 (kg/s) |
| `m` | `0.27` | 排球质量 (kg) |
| `g` | `9.80` | 重力加速度 (m/s²) |
| `Q_sigma_x` | `1.0` | 过程噪声标准差 — X 方向 |
| `Q_sigma_y` | `1.0` | 过程噪声标准差 — Y 方向 |
| `Q_sigma_z` | `10.0` | 过程噪声标准差 — Z 方向（允许高度方向更大不确定性） |
| `R_sigma_x` | `0.01` | 测量噪声标准差 — X 方向 |
| `R_sigma_y` | `0.01` | 测量噪声标准差 — Y 方向 |
| `R_sigma_z` | `0.05` | 测量噪声标准差 — Z 方向 |
| `detect_cnt_tres` | `3` | 进入追踪前需要的首帧确认帧数 |
| `lost_time` | `1.0` | 丢检超时判定阈值 (s) |
| `track_pub_rate` | `60` | 追踪结果发布频率 (Hz) |
| `target_frame_id` | `odom` | 目标输出坐标系 |

## 核心类

### `Tracker` — 追踪器状态机

位于 `include/volleyball_track/tracker.hpp`

- 管理 `DETECTING` / `TRACKING` / `LOST` 三种状态
- `update()`: 检测到来时执行预测 + 更新
- `predictOnly()`: 检测丢失时纯外推，累计超过阈值切至 `LOST`
- `reset()`: 手动复位至 `DETECTING`

### `EKF` — 扩展卡尔曼滤波器

位于 `include/volleyball_track/KF.hpp`

- **预测步**: 使用过程模型的雅可比矩阵线性化传播状态；更新协方差矩阵 `P = F·P·Fᵀ + Q`
- **更新步**: 计算卡尔曼增益 `K = P·Hᵀ·(H·P·Hᵀ + R)⁻¹`；更新后验状态；使用 **Joseph 形式** 更新协方差矩阵保证数值稳定性

### 过程模型

位于 `include/volleyball_track/process_model.hpp`

| 模型 | 说明 |
|------|------|
| `Ballist3D` | 简单抛体模型（无空气阻力），仅受重力 |
| `LinearDrag3D` ⭐ | **默认使用的模型**，带线性空气阻力 `F_drag = -k·v`，解析解形式（高效） |
| `QuadraticDrag3D` | 二次空气阻力模型 `F_drag = -k·v·|v|`，使用 RK4 数值积分 |

#### `LinearDrag3D` 状态转移

状态向量 `x = [px, vx, py, vy, pz, vz]ᵀ`

令 `β = k/m`，`φ = e^{-β·Δt}`，`η = (1-φ)/β`

```
px(t+Δt) = px(t) + η·vx(t)
vx(t+Δt) = φ·vx(t)
py(t+Δt) = py(t) + η·vy(t)
vy(t+Δt) = φ·vy(t)
pz(t+Δt) = pz(t) + η·vz(t) - g·(Δt - η)/β
vz(t+Δt) = φ·vz(t) - g·η
```

### 观测模型

位于 `include/volleyball_track/measure_model.hpp`

- `Position3D`: 观测直接映射到状态的位置分量 `z = [px, py, pz]ᵀ`
- 测量噪声矩阵 `R = diag(σx², σy², σz²)`

### `UKF` — 无迹卡尔曼滤波器

虽已实现但**未在默认配置中使用**，支持任意非线性过程/观测模型。

## 输入数据流向

```
/detector/ball  (camera_link 坐标系)
       │
       ▼  TF2 变换 (camera_link → odom)
       │
/tracker/target  (odom 坐标系，含 vx, vy, vz)
       │
       ▼
volleyball_predict (轨迹预测)
```

## 丢检处理机制

- `ball_lost_selfcheck_timer_callback` 以 **0.1s 周期** 检查自上次检测以来的时间间隔
- 超过 `lost_selfcheck_time_thres`（同 `lost_time`）后调用 `tracker_->predictOnly(dt)` 进行纯外推
- 外推累计超过 `lost_time_threshold` 则状态切至 `LOST`

## Launch 文件

```
volleyball_track.launch.py → tracker_node (加载 track.yaml 参数)
```

## 依赖

- `rclcpp` — ROS2 C++ 客户端库
- `Eigen3` — 线性代数库（矩阵运算、LDLT 分解）
- `volleyball_interfaces` — 自定义消息
- `tf2_ros` / `tf2_geometry_msgs` — 坐标变换
- `visualization_msgs` — RViz 可视化 Marker
- `std_srvs` — 标准服务类型（Trigger 重置）
