# CLAUDE.md

## 项目背景与开源目标

ROBOCON2026 沙排之王赛道上位机算法开源项目（福州大学浮舟湿地战队，作者 XiaoFeiyu12）。

本仓库根目录下有两个方案，同属一个 git 仓库：

- `drop_point_prediction/` — 原立项方案：识别定位 → 状态估计 → 轨迹预测 → 规划。高门槛、理论可行但难调试。
- `hybrid_vs/` — 末周临时方案：基于会议论文的 PID 视觉伺服。低门槛、短期效果好，但无法按球飞行方向控制。

**开源目标**：纪念性开源、供参考，两方案最终都未能稳定接球。改动以「各方案内部自洽、可读、符合规范」为准，不必追求两方案一致。

**重点注意事项**：两方案的 `src/` 下有同名共享包（`volleyball_detect`、`volleyball_interfaces`、`volleyball_robot`、`volleyball_robot_description`、`volleyball_serial_driver`）。`hybrid_vs` 是基于 `drop_point_prediction` 临时修改出来的，因此共享包里可能残留未适配对应方案的代码逻辑；操作共享包前，先 diff 两个文件夹核对差异。`hybrid_vs` 另有独有包 `volleyball_ibvs`。

## 代码规范（命名）

规范文档：`drop_point_prediction/浮舟湿地算法组开发规范.md`。核心命名规则：

- **变量**：小写 `snake_case`（阈值 `_threshold`、计数 `_cnt`、指针 `p_` 前缀或 `_ptr` 后缀），这个放在最后先汇总并再给指示进行修改。
- **成员变量**：`snake_case` + 尾下划线 `_`（含 struct 字段）。
- **常量/枚举**：`UPPER_SNAKE_CASE`，枚举值加枚举名前缀（如 `TRACK_STATE_IDLE`）；常量用 `const` 而非宏。
- **函数**：小写 `snake_case`，动宾组合。
- **类型**（类/结构体/枚举）：`PascalCase`，缩写当一个词（`Ekf` 而非 `EKF`、`StartRpc` 而非 `StartRPC`）。
- **命名空间 / 文件**：小写 `snake_case`。

**豁免**：滤波/几何代码里的数学符号（`F`/`Q`/`R`/`P`/`S`/`K`/`x`/`z` 等单字母）保留不改。

**联动要求**：ROS 参数名与变量名一一对应——改一个变量名时，同步改对应的 `declare_parameter`/`get_parameter` 字符串键，以及 `node_params.yaml` / `launch_params.yaml` 里的同名 key。

**待办（暂缓，未做）**：`thres`→`threshold` 缩写；指针变量 `p_`/`_ptr` 前后缀；`.msg` 文件单字母坐标字段。

## 常用命令

```bash
# 在仓库根目录操作；git 路径会带文件夹前缀
cd /home/xiaofeiyu/2026-robocon-volleyball-robot && git status && git log --oneline -10

# diff 两个方案的共享包，找未适配代码
for p in volleyball_detect volleyball_interfaces volleyball_plan volleyball_predict \
         volleyball_serial_driver volleyball_track volleyball_robot volleyball_robot_description; do
  echo "===== $p ====="; diff -rq drop_point_prediction/src/$p hybrid_vs/src/$p
done

# 逐行看某个共享包差异
git diff --no-index drop_point_prediction/src/volleyball_track hybrid_vs/src/volleyball_track
```