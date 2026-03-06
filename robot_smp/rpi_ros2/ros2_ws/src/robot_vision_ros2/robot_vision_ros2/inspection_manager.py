#!/usr/bin/env python3
# coding=utf-8
"""
巡检记录管理节点 (Inspection Manager)
功能：
  1. 订阅 YOLO 检测结果，记录到 CSV
  2. 订阅标注图像，保存截图
  3. 订阅巡检状态，关联检测与巡检点
  4. 图片保存防洪：同类去重冷却 + 最大数量限制
  5. 退出时输出统计摘要
"""

import json
import os
import csv
import time
import shutil
from datetime import datetime

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def _parse_bool(value) -> bool:
    """安全解析布尔值，兼容 LaunchConfiguration 传入的字符串。"""
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.lower() in ('true', '1', 'yes')
    return bool(value)


class InspectionManager(Node):
    def __init__(self):
        super().__init__('inspection_manager')

        # ---- 参数 ----
        self.declare_parameter('detections_topic', '/yolo/detections')
        self.declare_parameter('annotated_topic', '/yolo/annotated')
        self.declare_parameter('status_topic', '/inspection/status')
        self.declare_parameter('save_dir', '/tmp/inspection')
        self.declare_parameter('min_conf', 0.5)
        self.declare_parameter('save_images', True)
        self.declare_parameter('image_save_cooldown', 5.0)   # 同类图片最小保存间隔(秒)
        self.declare_parameter('max_saved_images', 500)       # 最大保存图片数

        self.detections_topic = self.get_parameter('detections_topic').value
        self.annotated_topic = self.get_parameter('annotated_topic').value
        self.status_topic = self.get_parameter('status_topic').value
        self.save_dir = self.get_parameter('save_dir').value
        self.min_conf = float(self.get_parameter('min_conf').value)
        self.save_images = _parse_bool(self.get_parameter('save_images').value)
        self.image_save_cooldown = float(self.get_parameter('image_save_cooldown').value)
        self.max_saved_images = int(self.get_parameter('max_saved_images').value)

        # ---- 文件目录 ----
        self.img_dir = os.path.join(self.save_dir, 'images')
        os.makedirs(self.save_dir, exist_ok=True)
        os.makedirs(self.img_dir, exist_ok=True)

        self.csv_path = os.path.join(self.save_dir, 'inspection_log.csv')
        self._init_csv()

        # ---- 状态 ----
        self.bridge = CvBridge()
        self._latest_annotated = None
        self._latest_annotated_stamp = None
        self._saved_image_count = 0
        self._total_detection_count = 0
        self._valid_detection_count = 0
        self._class_last_save_time = {}       # {class_name: last_save_timestamp}
        self._current_waypoint = ''           # 当前巡检点名称
        self._start_time = time.time()

        # ---- 订阅 ----
        self.sub_det = self.create_subscription(
            String, self.detections_topic,
            self.detection_callback, qos_profile_system_default)

        self.sub_status = self.create_subscription(
            String, self.status_topic,
            self.status_callback, qos_profile_system_default)

        if self.save_images:
            self.sub_img = self.create_subscription(
                Image, self.annotated_topic,
                self.annotated_callback, qos_profile_sensor_data)

        self.get_logger().info(
            f'Inspection manager started | save_dir={self.save_dir} | '
            f'cooldown={self.image_save_cooldown}s | max_images={self.max_saved_images}')

    # ---- CSV ----

    def _init_csv(self):
        if os.path.exists(self.csv_path):
            return
        with open(self.csv_path, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'waypoint', 'frame_id',
                             'class_name', 'conf', 'bbox_xyxy'])

    # ---- 回调 ----

    def status_callback(self, msg: String):
        """订阅巡检状态，跟踪当前巡检点。"""
        try:
            data = json.loads(msg.data)
            state = data.get('state', '')
            detail = data.get('detail', '')
            if state in ('inspecting', 'navigating'):
                self._current_waypoint = detail
            elif state == 'finished':
                self.get_logger().info('Patrol finished signal received.')
        except Exception:
            pass

    def annotated_callback(self, msg: Image):
        try:
            self._latest_annotated = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self._latest_annotated_stamp = msg.header.stamp
        except Exception:
            pass

    def detection_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception:
            return

        frame_id = payload.get('frame_id', '')
        detections = payload.get('detections', [])
        if not detections:
            return

        self._total_detection_count += len(detections)
        now_str = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        now_ts = time.time()
        det_stamp = payload.get('stamp', {})
        valid_classes = []  # 本帧有效检测的类别名

        with open(self.csv_path, 'a', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            for det in detections:
                conf = float(det.get('conf', 0.0))
                if conf < self.min_conf:
                    continue
                cls_name = det.get('class_name', '')
                valid_classes.append(cls_name)
                self._valid_detection_count += 1
                writer.writerow([
                    now_str,
                    self._current_waypoint,
                    frame_id,
                    cls_name,
                    conf,
                    det.get('bbox_xyxy', []),
                ])

        if not valid_classes:
            return

        # ---- 图片保存（含防洪策略）----
        if not self.save_images or self._latest_annotated is None:
            return
        if self._saved_image_count >= self.max_saved_images:
            return  # 已达上限

        # 帧时间戳匹配检查
        if self._latest_annotated_stamp is not None and det_stamp:
            if self._latest_annotated_stamp.sec != det_stamp.get('sec', -1):
                return  # 帧不匹配

        # 同类冷却去重：同一类别在 cooldown 秒内只保存一张
        should_save = False
        for cls in valid_classes:
            last_t = self._class_last_save_time.get(cls, 0.0)
            if now_ts - last_t >= self.image_save_cooldown:
                should_save = True
                self._class_last_save_time[cls] = now_ts

        if not should_save:
            return

        # 磁盘空间检查（剩余 < 100MB 时停止保存）
        try:
            disk = shutil.disk_usage(self.save_dir)
            if disk.free < 100 * 1024 * 1024:
                self.get_logger().warn('Disk space < 100MB, image saving paused.')
                return
        except Exception:
            pass

        self._saved_image_count += 1
        wp_tag = self._current_waypoint.replace(' ', '_') if self._current_waypoint else 'unknown'
        img_name = (datetime.now().strftime('%Y%m%d_%H%M%S')
                    + f'_{wp_tag}_{self._saved_image_count}.jpg')
        img_path = os.path.join(self.img_dir, img_name)
        try:
            cv2.imwrite(img_path, self._latest_annotated)
            self.get_logger().info(f'[{self._saved_image_count}/{self.max_saved_images}] '
                                  f'Saved: {img_name}')
        except Exception as exc:
            self.get_logger().warn(f'Failed to save image: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = InspectionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        elapsed = time.time() - node._start_time
        mins = int(elapsed // 60)
        secs = int(elapsed % 60)
        node.get_logger().info(
            f'\n========== Inspection Summary ==========\n'
            f'  Duration:      {mins}m {secs}s\n'
            f'  Total dets:    {node._total_detection_count}\n'
            f'  Valid dets:    {node._valid_detection_count} (conf >= {node.min_conf})\n'
            f'  Images saved:  {node._saved_image_count}\n'
            f'  CSV log:       {node.csv_path}\n'
            f'========================================')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
