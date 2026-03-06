#!/usr/bin/env python3
# coding=utf-8

import json
import time
from typing import List, Dict

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import qos_profile_system_default

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def _parse_bool(value) -> bool:
    """安全解析布尔值，兼容 LaunchConfiguration 传入的字符串。
    注意：bool('false') == True，所以必须用此函数。"""
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.lower() in ('true', '1', 'yes')
    return bool(value)


class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('result_topic', '/yolo/detections')
        self.declare_parameter('annotated_topic', '/yolo/annotated')
        self.declare_parameter('publish_annotated', True)
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('conf', 0.40)
        self.declare_parameter('iou', 0.45)
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('max_fps', 5.0)

        self.image_topic = self.get_parameter('image_topic').value
        self.result_topic = self.get_parameter('result_topic').value
        self.annotated_topic = self.get_parameter('annotated_topic').value
        self.publish_annotated = _parse_bool(self.get_parameter('publish_annotated').value)
        self.model_path = self.get_parameter('model_path').value
        self.conf = float(self.get_parameter('conf').value)
        self.iou = float(self.get_parameter('iou').value)
        self.device = self.get_parameter('device').value
        self.max_fps = float(self.get_parameter('max_fps').value)
        self._last_infer_time = 0.0

        self.bridge = CvBridge()

        self.pub_result = self.create_publisher(String, self.result_topic, qos_profile_system_default)
        self.pub_annotated = self.create_publisher(Image, self.annotated_topic, qos_profile_sensor_data)

        self.sub_image = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data
        )

        # 懒加载：首帧到来时才加载模型，避免 __init__ 阻塞 10-30 秒
        self.model = None
        self._model_load_attempted = False

        # ---- 统计数据 ----
        self._stat_infer_count = 0     # 总推理帧数
        self._stat_det_count = 0       # 总检测目标数
        self._stat_start_time = 0.0    # 统计起始时间
        self._stat_timer = self.create_timer(30.0, self._log_stats)  # 每30秒输出

        self.get_logger().info(
            f'YOLO detector started (model will load on first image), '
            f'image_topic={self.image_topic}')

    def _log_stats(self):
        """每 30 秒输出一次推理统计，方便监控模型是否正常工作。"""
        if self._stat_start_time <= 0 or self._stat_infer_count == 0:
            return
        elapsed = time.time() - self._stat_start_time
        if elapsed <= 0:
            return
        fps = self._stat_infer_count / elapsed
        self.get_logger().info(
            f'[Stats] {self._stat_infer_count} frames in {elapsed:.0f}s '
            f'({fps:.1f} FPS), {self._stat_det_count} detections total')

    def _load_model(self, model_path: str, device: str):
        try:
            from ultralytics import YOLO
            model = YOLO(model_path)
            try:
                model.to(device)
            except Exception:
                pass
            return model
        except Exception as exc:
            self.get_logger().error(f'Failed to import/load ultralytics YOLO: {exc}')
            return None

    def image_callback(self, msg: Image):
        if self.max_fps > 0:
            now = time.time()
            min_interval = 1.0 / self.max_fps
            if now - self._last_infer_time < min_interval:
                return
            self._last_infer_time = now

        # 懒加载模型：首帧到来时加载，避免阻塞节点启动
        if self.model is None:
            if self._model_load_attempted:
                return  # 已尝试加载但失败，不再重试
            self._model_load_attempted = True
            self.get_logger().info(f'Loading YOLO model: {self.model_path} ...')
            self.model = self._load_model(self.model_path, self.device)
            if self.model is None:
                self.get_logger().error(
                    'YOLO model load failed. '
                    'Please install ultralytics and check model_path.')
                return
            self.get_logger().info('YOLO model loaded successfully.')

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'cv_bridge convert failed: {exc}')
            return

        try:
            results = self.model(cv_image, conf=self.conf, iou=self.iou, verbose=False)
        except Exception as exc:
            self.get_logger().error(f'YOLO inference failed: {exc}')
            return

        detections = self._build_detection_list(results)

        # 更新统计
        self._stat_infer_count += 1
        self._stat_det_count += len(detections)
        if self._stat_start_time <= 0:
            self._stat_start_time = time.time()

        payload = {
            'stamp': {
                'sec': msg.header.stamp.sec,
                'nanosec': msg.header.stamp.nanosec,
            },
            'frame_id': msg.header.frame_id,
            'detections': detections,
        }

        result_msg = String()
        result_msg.data = json.dumps(payload, ensure_ascii=False)
        self.pub_result.publish(result_msg)

        # 仅当有检测结果时才渲染标注图 → 节省 RPi5 CPU
        # (无目标时 plot() 仍会渲染空框架，浪费算力)
        if self.publish_annotated and len(results) > 0 and len(detections) > 0:
            try:
                annotated = results[0].plot()
                annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
                annotated_msg.header = msg.header
                self.pub_annotated.publish(annotated_msg)
            except Exception as exc:
                self.get_logger().error(f'Publish annotated failed: {exc}')

    def _build_detection_list(self, results) -> List[Dict]:
        if len(results) == 0:
            return []

        res0 = results[0]
        names = res0.names if hasattr(res0, 'names') else {}
        boxes = res0.boxes

        dets: List[Dict] = []
        if boxes is None:
            return dets

        for box in boxes:
            xyxy = box.xyxy[0].tolist()
            conf = float(box.conf[0]) if hasattr(box, 'conf') else 0.0
            cls_id = int(box.cls[0]) if hasattr(box, 'cls') else -1
            cls_name = names.get(cls_id, str(cls_id))

            dets.append({
                'class_id': cls_id,
                'class_name': cls_name,
                'conf': round(conf, 4),
                'bbox_xyxy': [round(v, 2) for v in xyxy],
            })

        return dets


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down YOLO detector...')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
