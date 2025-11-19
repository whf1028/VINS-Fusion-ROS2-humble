#!/bin/bash
# ============================================================================
# VINS-Fusion RealSense D435 快速启动脚本
# ============================================================================
# 用途: 一键启动 VINS-Fusion 与 RealSense D435
# 日期: 2025-11-17
# ============================================================================

set -e  # 遇到错误立即退出

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}======================================${NC}"
echo -e "${GREEN}VINS-Fusion RealSense D435 启动脚本${NC}"
echo -e "${GREEN}======================================${NC}"

# 检查工作空间
WORKSPACE="/home/fsuav/vins_ws"
if [ ! -d "$WORKSPACE" ]; then
    echo -e "${RED}错误: 工作空间不存在: $WORKSPACE${NC}"
    exit 1
fi

cd "$WORKSPACE"

# 检查是否已编译
if [ ! -d "$WORKSPACE/install/vins_fusion_ros2_humble" ]; then
    echo -e "${YELLOW}警告: VINS 包尚未编译${NC}"
    echo -e "${YELLOW}正在编译 VINS-Fusion...${NC}"
    colcon build --packages-select camera_models vins_fusion_ros2_humble loop_fusion global_fusion --symlink-install
    if [ $? -ne 0 ]; then
        echo -e "${RED}编译失败!${NC}"
        exit 1
    fi
fi

# Source 工作空间
echo -e "${GREEN}1. Sourcing 工作空间...${NC}"
source "$WORKSPACE/install/setup.bash"

# 检查 RealSense 相机节点是否在运行
echo -e "${GREEN}2. 检查 RealSense 相机状态...${NC}"
if ! ros2 topic list | grep -q "/camera/infra1/image_rect_raw"; then
    echo -e "${YELLOW}警告: RealSense 相机节点未运行${NC}"
    echo -e "${YELLOW}请在另一个终端中运行以下命令启动相机:${NC}"
    echo ""
    echo -e "${YELLOW}ros2 launch realsense2_camera rs_launch.py \\${NC}"
    echo -e "${YELLOW}    enable_infra1:=true \\${NC}"
    echo -e "${YELLOW}    enable_infra2:=true \\${NC}"
    echo -e "${YELLOW}    enable_depth:=false \\${NC}"
    echo -e "${YELLOW}    enable_color:=false \\${NC}"
    echo -e "${YELLOW}    unite_imu_method:=2${NC}"
    echo ""
    read -p "是否继续启动 VINS? (y/N) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo -e "${RED}已取消${NC}"
        exit 1
    fi
fi

# 启动 VINS
echo -e "${GREEN}3. 启动 VINS-Fusion...${NC}"
CONFIG_FILE="$WORKSPACE/src/VINS-Fusion-ROS2-humble/config/realsense_d435/realsense_stereo_imu_config.yaml"

if [ ! -f "$CONFIG_FILE" ]; then
    echo -e "${RED}错误: 配置文件不存在: $CONFIG_FILE${NC}"
    exit 1
fi

echo -e "${GREEN}配置文件: $CONFIG_FILE${NC}"
echo -e "${GREEN}启动方式: ros2 launch vins_fusion_ros2_humble vins_d435.launch.py${NC}"
echo ""

# 使用 launch 文件启动
ros2 launch vins_fusion_ros2_humble vins_d435.launch.py

# 如果直接运行节点 (备用方案)
# ros2 run vins_fusion_ros2_humble vins_fusion_ros2_humble_node "$CONFIG_FILE"
