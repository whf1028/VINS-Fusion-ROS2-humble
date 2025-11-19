# 我的说明文档

# 使用背景
## 四旋翼无人机
### holybr pixhawk 4板子，px4:v1.16版本飞控，realsense D435双目相机
### 需要使用VIO来进行视觉惯性导航

### 项目借鉴fast-drone-250项目，故开始先选择vins-fusion作为VIO。
### 遇到第一个问题就是开源的vins-fusion没有明确支持ros2-humble的，找的个别名字带有ros2-humble且在环境中能编译的，结果标定好参数后，发布的imu_propagate死活都是发散的。
### 这个开源的vins-fusion修改了部分代码，实现了在ubuntu22.04系统，ros2-humble环境上，标定参数配置精确的情况下，能达到预期功能，检测发布的imu_propagate位置数据不发散，符合预期值。