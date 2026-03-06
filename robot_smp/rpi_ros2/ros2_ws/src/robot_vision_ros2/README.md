# robot_vision_ros2 (树莓派运行)

## 架构
```
香橙派(车端)                    树莓派(计算端)
┌────────────────┐   网线ZMQ   ┌──────────────────┐
│ base_control   │◄───────────►│ v4l2_camera      │
│ 雷达驱动       │   /odom     │ yolo_detector    │
│ Nav2 导航      │   /scan     │ inspection_mgr   │
│                │   /tf       │ waypoint_patrol   │
│                │◄────────────│  (发送导航目标)   │
│ move_base      │  ZMQ Goal   │                   │
└────────────────┘             └──────────────────┘
```

## 关键话题
- 本机: /image_raw → /yolo/detections, /yolo/annotated
- 跨网: waypoint_patrol → ZMQ nav_goal → move_base
- 跨网: move_base/result → ZMQ nav_result

## 启动方式
- 仅视觉: yolo.launch.py
- 完整巡检: inspection.launch.py

可调参数（常用）
- model_path: YOLO 权重路径（默认 yolov8n.pt）
- device: cpu 或 cuda:0（树莓派默认 cpu）
- max_fps: 推理限帧，降低CPU负载（默认 5）
- wait_at_point: 到达巡检点后等待检测时间（秒）
- loop: 是否循环巡检

## 部署
1. 把本文件夹复制到树莓派工作区
2. 安装 requirements.txt
3. colcon build
4. 运行 start_rpi.sh
