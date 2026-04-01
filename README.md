
2026.04.01
基于 **Isaac Sim 4.5.0** 与 **ROS2** 的铰接式装载车（Wheel Loader）横向 PID 闭环控制 Demo。

该项目实现了一个面向铰接转向工程车辆的路径跟踪控制器。  
控制器以 **前桥中心（front axle center）** 作为跟踪参考点，计算车辆相对于参考路径的横向偏差 `e_y`，通过 **横向 PID 控制** 生成铰接关节命令，驱动车辆沿预设轨迹行驶，并支持终点自动停车和 CSV 数据记录/分析。

---

## 1. 功能简介

当前实现的主要功能：

- 基于 **横向 PID** 的闭环路径跟踪
- 控制对象为 **铰接关节** `chassis_body_revolute`
- 通过 `/joint_command` 控制轮子速度与铰接角
- 通过 `/joint_states_rebuilt` 获取仿真反馈
- 路径支持：
  - 直线路径
  - 正弦曲线路径
- 支持终点自动停车
- 自动记录运行数据到 CSV
- 支持 Python 脚本绘制：
  - 参考路径 vs 实际轨迹
  - 横向误差随时间变化曲线

---

## 2. 系统架构

系统由两部分构成：

### 2.1 Isaac Sim 侧
负责：
- 车辆仿真
- 状态发布（前桥中心 Pose）
- 关节命令桥接
- 关节状态反馈

关键话题：
- `/wheel_loader/front_axle_pose`
- `/joint_states_rebuilt`
- `/joint_command`

### 2.2 ROS2 控制器侧
负责：
- 订阅仿真反馈
- 生成参考路径
- 计算横向误差
- 执行 PID 控制
- 输出铰接角与轮速命令
- 写入 CSV 数据

---

## 3. 核心控制逻辑

### 3.1 跟踪参考点
本项目使用 **前桥中心（front axle center）** 作为路径跟踪点。

原因：
- 对铰接式车辆来说更符合行驶几何
- 比 base_link 更稳定
- 不受工作装置（铲斗/机械臂）姿态影响

---

### 3.2 横向误差定义
采用 Frenet 误差定义：

\[
e_y = -\sin(\psi_r)(x-x_r) + \cos(\psi_r)(y-y_r)
\]

其中：
- `(x, y)` 为当前前桥中心位置
- `(x_r, y_r)` 为参考路径最近点
- `\psi_r` 为参考路径切线方向

---

### 3.3 横向 PID 控制
控制器核心为：

\[
\phi_{ref} = -(K_p e_y + K_i \int e_y dt + K_d \frac{de_y}{dt})
\]

其中：
- `Kp`：比例项，用于快速纠偏
- `Ki`：积分项，用于消除稳态误差
- `Kd`：微分项，用于抑制振荡和滞后
- `phi_ref`：期望铰接角

随后再经过：
- 铰接角限幅
- 铰接角变化率限制（rate limit）

最终输出 `phi_cmd`。

---

## 4. 项目目录结构

```bash
wheel_loader_controller/
├── include/
│   └── wheel_loader_controller/
│       └── curve_path_lateral_pid_tracker.hpp
├── src/
│   └── curve_path_lateral_pid_tracker.cpp
├── launch/
│   └── curve_path_lateral_pid_tracker.launch.py
├── scripts/
│   └── plot_curve_pid_log.py
├── CMakeLists.txt
├── package.xml
└── README.md
```

---

## 5. 依赖环境

### 软件环境
- Ubuntu 22.04
- ROS2 Humble
- Isaac Sim 4.5.0
- Python 3.10

### ROS2 依赖
- `rclcpp`
- `sensor_msgs`
- `geometry_msgs`

### Python 依赖
- `pandas`
- `matplotlib`
- `numpy`

安装：
```bash
pip install pandas matplotlib numpy
```

---

## 6. 编译方法

在 ROS2 工作空间中执行：

```bash
cd ~/ros2_ws
colcon build --packages-select wheel_loader_controller --symlink-install
source install/setup.bash
```

---

## 7. 运行方法

### 7.1 启动 Isaac Sim
确保 Isaac Sim 场景中：
- 车辆模型已加载
- front axle pose 发布正常
- joint control graph 正常工作
- `/joint_states_rebuilt` 和 `/joint_command` 链路已打通

### 7.2 启动控制器
```bash
ros2 launch wheel_loader_controller curve_path_lateral_pid_tracker.launch.py
```

---

## 8. Launch 参数说明

`launch/curve_path_lateral_pid_tracker.launch.py` 中可配置以下参数：

### PID 参数
- `kp`：比例增益
- `ki`：积分增益
- `kd`：微分增益

### 速度参数
- `target_velocity`：轮子关节目标角速度（rad/s）

### 路径参数
- `path_x_start`：路径起点 x
- `path_x_end`：路径终点 x
- `path_dx`：路径离散采样间隔
- `path_amplitude`：正弦路径振幅
- `path_frequency`：正弦路径频率
- `path_y_offset`：路径整体 y 偏移

### 控制约束
- `articulation_rate_limit`：每周期允许的最大铰接角变化量
- `max_steering` / `min_steering`：铰接角上下限

### 停车控制
- `stop_at_end`：是否终点停车
- `stop_dist`：距离终点多少米开始停车

---

## 9. 日志与数据分析

程序运行时会生成：

```bash
tracking_performance.csv
```

CSV 字段说明：

- `time`：时间戳
- `x, y`：实际前桥中心位置
- `ref_x, ref_y`：参考路径最近点位置
- `err_y`：横向误差
- `phi_cmd`：控制输出的铰接角命令

### 绘图分析
运行：

```bash
python3 scripts/plot_curve_pid_log.py tracking_performance.csv
```

绘图内容：
- 实际轨迹 vs 参考路径
- 横向误差曲线

---

## 10. 调参建议

### 低速（如 0.35 ~ 0.5）
推荐参数：
```python
kp = 0.55
ki = 0.015
kd = 0.12
```

特点：
- 收敛快
- 平稳
- 适合初始验证

### 中高速（如 3.0）
推荐参数：
```python
kp = 0.22
ki = 0.005
kd = 0.45
articulation_rate_limit = 0.06
```

特点：
- 降低比例项，避免过冲
- 增大微分项，抑制滞后与蛇行
- 放宽转向速率限制，避免“转不过弯”

---

## 11. 常见问题

### Q1：车辆不动
检查：
- `/joint_command` 是否只有一个订阅者
- `velocityCommand` 是否连接到 IsaacArticulationController
- `/joint_states_rebuilt` 中 wheel velocity 是否非零

### Q2：绘图时报 CSV 没有列名
原因：
- 程序被强行结束，CSV 缓冲区未刷新
- 旧文件无表头残留

解决：
```bash
rm tracking_performance.csv
```
重新运行程序

### Q3：轨迹图和仿真里显示的参考路径不一致
原因：
- 参考路径公式里用了 `path_y_offset`
- 但仿真画点脚本可能没加 `y_offset`

参考路径实际公式：

\[
y = path\_y\_offset + path\_amplitude \cdot \sin(path\_frequency \cdot x)
\]

---

## 12. 与真实车辆的耦合度

### 可迁移部分
- 前桥中心作为跟踪参考点
- Frenet 横向误差定义
- 横向 PID 控制框架
- 铰接角作为控制输出对象

### 需要补齐的部分
- 真实车辆状态估计（GNSS/IMU/编码器融合）
- 真实纵向动力学（油门/刹车替代轮速控制）
- 地面摩擦与轮胎模型
- 安全冗余与故障保护

结论：
该项目适合作为 **铰接车辆横向控制 baseline** 和 **仿真验证平台**。

---

## 13. 应用场景

该 demo 适用于：

- 无人装载机路径跟踪
- 场内低速运输车辆
- 矿区/料场/港口内部固定路线跟踪
- 工程机械自动驾驶基础控制验证

不适用于：
- 高速公路自动驾驶
- 强非结构化越野环境
- 高精度铲斗末端作业控制

---

## 14. 后续优化方向

- 引入速度调度（按曲率/误差限速）
- 引入捕获模式（大偏差并入）
- 使用外部 Path 订阅替代内置路径生成
- 加入 Stanley / MPC 等高级控制方法
- 迁移到实车并进行 Sim-Real-Sim 标定

---

## 15. 作者说明

当前版本定位为：

> **铰接式工程车辆横向 PID 闭环控制 Demo**

重点在于：
- 打通控制闭环
- 建立可重复实验平台
- 提供后续高级控制算法的对比基线

```

---

如果你愿意，我下一步还可以继续帮你补两样非常有用的东西：

1. **`CMakeLists.txt` 完整模板**
2. **`package.xml` 完整模板**

这样你的项目就可以直接作为一个完整 ROS2 包整理归档。