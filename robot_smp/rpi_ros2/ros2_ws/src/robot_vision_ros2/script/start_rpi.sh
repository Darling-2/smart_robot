#!/bin/bash
# =============================================
# 树莓派 一键启动脚本 (视觉 + 巡检巡航)
# 放到树莓派: ~/start_rpi.sh
#
# 架构说明:
#   香橙派: 底盘 + 雷达 + Nav2 (硬件+导航闭环)
#   树莓派: 相机 + YOLO + 巡检记录 + 巡航指令
#   衔接:   waypoint_patrol 通过 DDS 跨网调用
#           香橙派上的 Nav2 navigate_to_pose Action
# =============================================

set -e

# ---------- 后台进程清理 ----------
cleanup() {
    echo ""
    echo "[INFO] Stopping all ROS2 processes..."
    kill $(jobs -p) 2>/dev/null
    wait 2>/dev/null
    echo "[INFO] All processes stopped."
}
trap cleanup EXIT INT TERM

# ---------- 环境变量 (必须与香橙派一致) ----------
export ROS_DOMAIN_ID=30
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

export BASE_TYPE=NanoRobot

# ---------- CycloneDDS 配置检查 ----------
if [ -f "$HOME/cyclonedds.xml" ]; then
    export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml
    echo "[OK] CycloneDDS config: $HOME/cyclonedds.xml"
else
    echo "[ERROR] $HOME/cyclonedds.xml not found!"
    echo "        跨机通信需要此文件。请从项目复制:"
    echo "        cp robot_vision_ros2/config/cyclonedds.xml ~/cyclonedds.xml"
    echo "        并修改其中的 IP 地址为实际地址。"
    exit 1
fi

# ---------- source ----------
source /opt/ros/foxy/setup.bash
source ~/colcon_ws/install/setup.bash

echo "============================================"
echo "  RPi5: ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "  RPi5: checking OrangePi topics..."
echo "============================================"

# ---------- 等待香橙派话题就绪 ----------
echo "[1/3] Checking /odom ..."
timeout 15 ros2 topic echo /odom --once > /dev/null 2>&1 \
  && echo "  [OK] /odom found" \
  || echo "  [WARN] /odom not found, check OrangePi"

echo "[2/3] Checking /scan ..."
timeout 15 ros2 topic echo /scan --once > /dev/null 2>&1 \
  && echo "  [OK] /scan found" \
  || echo "  [WARN] /scan not found, check OrangePi"

echo "[3/3] Checking navigate_to_pose action ..."
timeout 15 ros2 action list 2>/dev/null | grep -q navigate_to_pose \
  && echo "  [OK] Nav2 action found" \
  || echo "  [WARN] Nav2 action not found, check OrangePi navigation"

echo ""
echo "============================================"
echo "  Starting inspection system..."
echo "============================================"

# ---------- 选择启动模式 ----------
# 模式1: 仅视觉检测 (不巡航, 手动控制小车)
# ros2 launch robot_vision_ros2 yolo.launch.py

# 模式2: 视觉 + 巡检巡航 (完整自动巡检)
ros2 launch robot_vision_ros2 inspection.launch.py
