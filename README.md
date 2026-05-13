# Springflyer
核心调整是：

```text
主战场：关联点筛选
主框架：分步优化 / 两段更新
保护层：退化方向识别 + 可靠子空间投影
兜底层：IMU + 气压计 + 自适应姿态/低速约束
```

一句话：**先筛好点，再分步解，再只接受可靠方向的状态更新。**

---

# 一、总体架构

```text
第0层 系统优化
  ↓
第1层 传感器约束
  ↓
第2层 关联点筛选
  ↓
第3层 分步优化 / 两段更新
  ↓
第4层 退化保护
  ↓
第5层 输出与飞控联动
```

最终目标不是让 VINS 在白墙里“强行正常工作”，而是让系统做到：

```text
能看的方向用视觉优化
看不清的方向用 IMU 短时传播
z 方向用气压计辅助
roll/pitch 低动态时辅助锁住
退化方向不把错误视觉信息写进状态和边缘化先验
```

---

# 二、第0层：系统优化

N150 是纯小核平台，不能指望重后端。配置建议：

```yaml
image_resolution: 640x480
camera_fps: 15
imu_freq: 200
baro_freq: 50

ceres_num_threads: 1
ceres_linear_solver: DENSE_SCHUR
ceres_max_iter_normal: 5~8
ceres_max_iter_degraded: 3~5
max_solver_time_normal_ms: 25
max_solver_time_degraded_ms: 35

window_size_normal: 8~10
window_size_degraded: 5~7
pose_graph_enable: false
loop_closure_enable: false
```

系统侧：

```text
CPU governor performance
后端线程绑 Core0
前端线程绑 Core1
IMU/发布绑 Core2
ROS/杂项绑 Core3
关闭 image_transport 压缩
image queue 深度 1~2
```

---

# 三、第1层：传感器约束

## 1. 气压计高度因子

不要直接硬锁 z。气压计必须建 bias：

```text
r_baro = z_world + b_baro - z_baro
```

建议：

```yaml
baro_sigma_normal: 0.40~0.80
baro_sigma_degraded: 0.25~0.50
baro_bias_random_walk: 0.02~0.08 m/sqrt(s)
baro_outlier_thresh: 1.0~1.5m
```

气压计不要 50Hz 全部进后端。建议每个图像帧附近取若干 baro 样本做中值滤波，只加一个 baro factor。

## 2. Roll/Pitch 自适应先验

无人机加速时，加速度计不是重力计，所以不能固定强约束。

```text
低动态：roll/pitch 先验变硬
高动态：roll/pitch 先验变软或关闭
```

建议：

```yaml
rp_sigma_low_dynamic: 0.5~1.0 deg
rp_sigma_normal: 3.0~5.0 deg
rp_sigma_high_dynamic: 8.0 deg 或关闭
```

触发条件：

```text
abs(|acc|-g) 小
gyro_norm 小
持续时间 > 0.3~0.5s
```

## 3. 低速软约束，而不是硬 ZUPT

无人机悬停不是绝对零速。建议用 soft velocity prior：

```text
r_v = v - v_ref
```

`v_ref` 可以来自飞控期望速度，或者低动态时近似为 0。

```yaml
low_velocity_sigma: 0.05~0.15 m/s
low_velocity_duration: 0.5~1.0s
```

触发条件：

```text
IMU 方差低
光流中位数低
baro 高度变化低
飞控目标速度低
```

---

# 四、第2层：关联点筛选，作为核心模块

这是优化后方案的重点。

不要只追求“点多”，而要追求：

```text
残差干净
分布均匀
视差足够
深度层次好
信息方向不重复
IMU 预测一致
```

## 1. 特征配额

```yaml
max_frontend_features: 150
max_backend_features: 120~150
grid_rows: 4~6
grid_cols: 6~8
max_per_grid_cell: 2~4
min_track_length: 3
max_reproj_error_px: 2.5~3.0
imu_flow_gate_px: 3.0
```

## 2. 点分三类

### A 类：姿态点

特点：

```text
远点
小视差
长 track
残差稳定
```

用途：

```text
主要约束 R / yaw
弱约束平移
```

### B 类：平移点

特点：

```text
近点
大视差
三角化角足够
逆深度方差小
```

用途：

```text
主要约束 x/y/z 平移
```

### C 类：危险点

特点：

```text
短 track
大残差
光流与 IMU 预测不一致
深度不稳定
```

用途：

```text
只用于前端跟踪，不进后端优化
```

## 3. 筛选评分

每个 track 给一个质量分：

```text
score =
  角点响应
+ track 寿命
+ 空间分布奖励
+ 视差奖励
+ 三角化角奖励
+ IMU 光流一致性
- 重投影残差惩罚
- 动态嫌疑惩罚
- 信息冗余惩罚
```

推荐比例：

```yaml
rotation_features: 50~70
translation_features: 40~60
reserve_features: 20
```

白墙长廊里，真正有价值的可能只有几十个点。宁愿少而干净，不要多而重复。

---

# 五、第3层：分步优化 / 两段更新

这是把 LEOM/LOAM 退化思想 VIO 化后的主流程。

## 正常状态 HEALTHY

正常 VINS-Fusion 流程：

```text
IMU 预积分
视觉重投影
气压计高度
低动态 roll/pitch prior
低速软约束
```

滑窗：

```yaml
window_size: 8~10
iterations: 5~8
```

## 退化状态 DEGRADED

退化时采用两段更新。

### Step 1：姿态 / 高度 / 速度稳定化

使用：

```text
IMU 预积分
A 类姿态点
气压计高度因子
Roll/Pitch 自适应先验
低速软约束
```

主要优化：

```text
R
z
v
ba/bg 低动态时弱更新，强退化时冻结
baro_bias 慢更新
```

目标：先把姿态、速度、高度稳定住。

### Step 2：局部平移修正

使用：

```text
B 类平移点
少量 A 类点稳定 yaw
IMU propagation
```

主要优化：

```text
x
y
yaw
局部 SE(3) 小增量
```

只优化最近 2~3 帧，避免 N150 后端爆算力。

```yaml
window_size_degraded: 5~7
local_update_frames: 2~3
iterations_degraded: 3~5
```

---

# 六、第4层：退化检测与保护

退化检测分两级。

## 1. 廉价指标，每帧跑

```yaml
min_features_threshold: 30
min_track_length_median: 3
min_parallax_median_px: 5.0
max_far_point_ratio: 0.75
min_translation_feature_num: 20
max_reproj_error_median_px: 2.5
```

三选二或多条件触发：

```text
特征少
平移点少
视差小
远点比例过高
重投影残差变大
IMU/视觉预测不一致
```

## 2. Visual-only Hessian 检测

注意：**只用视觉残差构造 Hessian，不混入 IMU、气压计、roll/pitch prior。**

流程：

```text
取最新帧或最近 2~3 帧 pose Hessian
Schur 掉 landmark / inverse depth
得到 6x6 H_pose
尺度归一化
特征值分解
判断退化方向
```

尺度归一化：

```text
δx = [δp_x, δp_y, δp_z, δθ_x, δθ_y, δθ_z]

S = diag(
  1/0.5m,
  1/0.5m,
  1/0.5m,
  1/5deg,
  1/5deg,
  1/5deg
)
```

退化判断：

```yaml
hessian_ratio_threshold: 1e-4
condition_number_threshold: 1e4~1e5
```

但不要只靠固定阈值，最好加最近健康帧的滑动基线。

---

# 七、可靠子空间投影更新

优化结果不能全量接受。正确做法：

```text
IMU propagation 给基准
视觉优化给修正量
Hessian 判断修正量哪些方向可信
只接受可信方向
```

姿态必须在李代数上做：

```text
δξ_opt = Log(T_imu^-1 * T_opt)
δξ_final = P_reliable * δξ_opt
T_final = T_imu * Exp(δξ_final)
```

速度、bias：

```text
v_final = 可靠方向用优化，否则用 IMU propagation
ba/bg 在 DEGRADED 强退化时冻结
baro_bias 慢更新
外参/time offset 全程冻结
```

退化时不要让坏视觉进入边缘化：

```text
HEALTHY：正常边缘化
WEAK：谨慎边缘化
DEGRADED：暂停或限制边缘化
DEAD_RECKONING：不边缘化坏视觉帧
RECOVERY：异步局部 BA 后恢复
```

---

# 八、状态机

```text
HEALTHY
  ↓
WEAK
  ↓
DEGRADED
  ↓
DEAD_RECKONING
  ↓
LOST
```

建议定义：

```yaml
degraded_to_dead_reckoning_sec: 2.0~3.0
dead_reckoning_to_lost_sec: 5.0~8.0
recovery_required_good_frames: 10~20
```

状态含义：

```text
HEALTHY：正常 VIO
WEAK：视觉开始退化，但还能更新
DEGRADED：分步优化 + 投影更新
DEAD_RECKONING：主要靠 IMU/baro，通知飞控降速或悬停
LOST：紧急悬停/等待重定位
```

---

# 九、输出给飞控

不要只输出 pose。必须输出健康度和方向可信度。

```text
200Hz：IMU 高频传播位姿
15Hz：VIO 优化位姿
每帧：health state
每帧：6DoF confidence
每帧：退化方向 covariance 放大
```

建议输出：

```text
pose
velocity
bias 状态
health_state
degenerate_axis_body
degenerate_axis_world
covariance_scale_x/y/z/roll/pitch/yaw
```

飞控策略：

```text
HEALTHY：正常飞
WEAK：轻微限速
DEGRADED：限制水平速度和 yaw rate
DEAD_RECKONING：减速/悬停
LOST：紧急悬停或返控
```

---

# 十、推荐关键参数

```yaml
# camera / imu
image_resolution: 640x480
camera_fps: 15
imu_freq: 200
baro_freq: 50

# backend
ceres_num_threads: 1
ceres_max_iter_normal: 5~8
ceres_max_iter_degraded: 3~5
max_solver_time_normal_ms: 25
max_solver_time_degraded_ms: 35
window_size_normal: 8~10
window_size_degraded: 5~7
local_update_frames: 2~3

# frontend
max_frontend_features: 150
max_backend_features: 120~150
klt_pyramid_levels: 3
grid_rows: 4~6
grid_cols: 6~8
max_per_grid_cell: 2~4
min_track_length: 3
max_reproj_error_px: 2.5~3.0
imu_flow_gate_px: 3.0
min_parallax_px: 5.0
min_triangulation_angle_deg: 1.0~2.0

# feature allocation
rotation_features: 50~70
translation_features: 40~60
reserve_features: 20

# barometer
baro_sigma_normal: 0.40~0.80
baro_sigma_degraded: 0.25~0.50
baro_bias_random_walk: 0.02~0.08
baro_outlier_thresh: 1.0~1.5

# roll pitch prior
rp_sigma_low_dynamic: 0.5~1.0 deg
rp_sigma_normal: 3.0~5.0 deg
rp_sigma_high_dynamic: 8.0 deg or disabled

# low velocity update
low_velocity_sigma: 0.05~0.15
low_velocity_duration: 0.5~1.0

# degeneracy
min_features_threshold: 30
min_translation_feature_num: 20
min_parallax_median_px: 5.0
max_far_point_ratio: 0.75
hessian_ratio_threshold: 1e-4
condition_number_threshold: 1e4~1e5

# state machine
degraded_to_dead_reckoning_sec: 2.0~3.0
dead_reckoning_to_lost_sec: 5.0~8.0
recovery_required_good_frames: 10~20
```

---

# 十一、落地路线

```text
W1：系统裁剪
关 pose graph、限特征、单线程 Ceres、限优化时间、绑核、图像队列限制

W2：传感器因子
气压计 bias 因子、Roll/Pitch 自适应先验、低速软约束、健康度状态机

W3：关联点筛选
track 评分、IMU 光流门控、近远/视差分类、grid 均匀化、后端点配额

W4：退化检测
cheap metrics + visual-only 6x6 Hessian + 方向可信度输出

W5：分步优化和投影更新
Step1 姿态/z/v，Step2 x/y/yaw，李代数投影更新，bias 冻结

W6：边缘化保护和恢复
退化期跳过/限制边缘化，恢复期异步 local BA，实飞调参
```

如果工期紧，优先级是：

```text
P0：系统裁剪
P1：关联点筛选
P2：气压计 bias + 健康度
P3：分步优化
P4：Hessian 投影保护
P5：边缘化保护 / recovery BA
```

---

# 十二、最终结论

优化后的总体方案应该从“靠传感器约束硬拉”改成：

```text
关联点筛选决定精度
分步优化保证收敛
Hessian 退化分析负责安全保护
IMU + 气压计负责短时兜底
飞控根据健康度主动限速/悬停
```

这套方案比单纯加气压计、加 Roll/Pitch、加 ZUPT 更稳。白墙长廊里，真正靠谱的工程路径是：**筛干净点，分清点的约束类型，两段更新，只融合可靠方向。**
