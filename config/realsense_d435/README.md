# RealSense D435 配置文件

## 概述

本目录包含 Intel RealSense D435 双目红外相机与 PX4 ICM-42688-P IMU 的 VINS-Fusion 配置文件。

## 标定信息

- **标定日期**: 2025-11-17
- **标定工具**: Kalibr
- **标定质量**: **S 级 (卓越)**
  - 重投影误差: 0.18 px (cam0), 0.18 px (cam1)
  - IMU 陀螺仪残差: 0.85 σ
  - IMU 加速度计残差: 0.40 σ
  - 时间偏移: -14.18 ms
  - 双目基线: 50.040 mm (误差 0.08%)

## 文件说明

### 1. `realsense_stereo_imu_config.yaml`
主配置文件，包含：
- VINS 运行参数
- 相机-IMU 外参 (T_ic 变换矩阵)
- IMU 噪声参数 (Allan 方差标定)
- 时间同步参数
- 特征跟踪和优化参数

### 2. `left.yaml`
左相机 (Cam0) 内参配置：
- 焦距: fx=388.265, fy=389.585
- 主点: cx=325.313, cy=237.966
- 畸变系数 (radtan 模型)

### 3. `right.yaml`
右相机 (Cam1) 内参配置：
- 焦距: fx=389.422, fy=390.626
- 主点: cx=324.695, cy=238.046
- 畸变系数 (radtan 模型)

## 使用方法

### 方法 1: 使用 Launch 文件 (推荐)

```bash
# 1. 编译 VINS-Fusion-ROS2-humble (如果尚未编译)
cd /home/fsuav/fast-drone-250-humble-6.1
colcon build --packages-select camera_models vins_fusion_ros2_humble loop_fusion global_fusion --symlink-install
source install/setup.bash

# 2. 启动 VINS 节点
ros2 launch vins_fusion_ros2_humble vins_d435.launch.py

# 3. (可选) 启动 RViz 可视化
ros2 launch vins_fusion_ros2_humble vins_rviz.launch.xml
```

### 方法 2: 直接运行节点

```bash
# 1. Source 工作空间
cd /home/fsuav/fast-drone-250-humble-6.1
source install/setup.bash

# 2. 运行 VINS 节点
ros2 run vins_fusion_ros2_humble vins_fusion_ros2_humble_node \
    /home/fsuav/fast-drone-250-humble-6.1/src/realflight_modules/VINS-Fusion-ROS2-humble/config/realsense_d435/realsense_stereo_imu_config.yaml
```

## ROS2 Topic 配置

### 输入 Topics (订阅)
- `/camera/imu` - IMU 数据 (sensor_msgs/Imu)
- `/camera/infra1/image_rect_raw` - 左红外相机图像
- `/camera/infra2/image_rect_raw` - 右红外相机图像

### 输出 Topics (发布)
- `/vins/imu_propagate` - IMU 传播位姿
- `/vins/odometry` - VIO 里程计
- `/vins/path` - 轨迹路径
- `/vins/point_cloud` - 特征点云

## 启动 RealSense 相机

在运行 VINS 之前，需要先启动 RealSense 相机节点：

```bash
# 启动 D435 相机 (双目红外 + IMU)
ros2 launch realsense2_camera rs_launch.py \
    enable_infra1:=true \
    enable_infra2:=true \
    enable_depth:=false \
    enable_color:=false \
    enable_gyro:=true \
    enable_accel:=true \
    unite_imu_method:=2 \
    infra1_width:=640 \
    infra1_height:=480 \
    infra2_width:=640 \
    infra2_height:=480
```

### 参数说明
- `enable_infra1/infra2:=true` - 启用双目红外相机
- `enable_depth:=false` - 禁用深度 (VIO 不需要，节省性能)
- `enable_color:=false` - 禁用彩色相机
- `unite_imu_method:=2` - 使用线性插值融合 IMU (推荐)
- 分辨率: 640×480 (与标定一致)

## 配置参数详解

### 关键参数说明

#### 1. 外参估计模式
```yaml
estimate_extrinsic: 0  # 0=信任标定 (推荐), 1=在线优化, 2=从头标定
```
**推荐值**: `0` (完全信任 S 级标定)

#### 2. 时间偏移估计
```yaml
estimate_td: 0  # 0=信任标定 (推荐), 1=在线估计
td: -0.014182214771096076  # 时间偏移 (秒)
```
**推荐值**: `estimate_td: 0`, `td: -0.01418` (信任标定值)

#### 3. IMU 噪声参数
```yaml
acc_n: 0.0036962596   # 加速度计噪声密度
gyr_n: 0.0000768388   # 陀螺仪噪声密度
acc_w: 0.0004762768   # 加速度计 bias 游走
gyr_w: 0.0000071650   # 陀螺仪 bias 游走
```
**来源**: Allan 方差标定 (4 份独立 3 小时数据)

#### 4. 特征跟踪参数
```yaml
max_cnt: 150      # 最大特征点数 (100-200)
min_dist: 30      # 特征点最小间距 (像素)
freq: 10          # 发布频率 (Hz)
```

## 性能调优

### 实时性优化
如果遇到性能问题，可以调整以下参数：

```yaml
max_cnt: 100              # 减少特征点数量
max_solver_time: 0.03     # 减少优化时间
max_num_iterations: 6     # 减少迭代次数
```

### 精度优化
如果需要更高精度（牺牲实时性）：

```yaml
max_cnt: 200              # 增加特征点数量
max_solver_time: 0.06     # 增加优化时间
max_num_iterations: 10    # 增加迭代次数
```

## 故障排查

### 1. VINS 不收敛/发散
**可能原因**:
- IMU 数据不正确
- 时间同步问题
- 特征跟踪失败

**解决方法**:
- 检查 IMU topic: `ros2 topic hz /camera/imu`
- 检查图像 topic: `ros2 topic hz /camera/infra1/image_rect_raw`
- 确保相机和 IMU 时间戳同步
- 增加 `max_cnt` (特征点数量)

### 2. 特征点过少
**解决方法**:
- 增加 `max_cnt` (如 200)
- 减少 `min_dist` (如 20)
- 检查场景纹理是否丰富
- 检查图像是否过暗/过亮

### 3. 初始化失败
**解决方法**:
- VINS 需要 IMU 激励 (移动相机，包含旋转和平移)
- 确保有足够的特征点跟踪
- 检查 IMU 数据是否正常

## 参考资料

- **标定数据位置**: `/home/fsuav/vins_ws/src/kalibr_data/202511170142-数据包/`
- **标定报告**: `calibration_data_5phase_20251117_000019/camera_imu_calibration_5phase/`
- **VINS-Fusion 文档**: https://github.com/HKUST-Aerial-Robotics/VINS-Fusion
- **RealSense ROS2**: https://github.com/IntelRealSense/realsense-ros

## 更新日志

- **2025-11-17**: 基于 S 级标定创建初始配置
  - 重投影误差: 0.18 px
  - IMU 残差: 0.85 σ (陀螺仪), 0.40 σ (加速度计)
  - 时间偏移: -14.18 ms
