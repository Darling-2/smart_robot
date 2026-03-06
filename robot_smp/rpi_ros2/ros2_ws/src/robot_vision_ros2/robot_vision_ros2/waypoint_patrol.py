#!/usr/bin/env python3
# coding=utf-8
"""
巡检点巡航节点 (Waypoint Patrol)
功能：
  1. 从 YAML 读取巡检点列表
  2. 依次调用 Nav2 导航到每个巡检点
  3. 到达后停留等待视觉检测
  4. 所有点巡完后可循环或停止
  5. 发布巡检状态供 inspection_manager 使用

架构说明：
  本节点运行在树莓派上，通过 DDS 跨网调用
  香橙派上 Nav2 的 navigate_to_pose Action。
  使用 MultiThreadedExecutor + 独立巡逻线程
  避免 spin_until_future_complete 死锁。
"""

import json
import yaml
import time
import math
import threading

try:
    import zmq
except ImportError:  # pragma: no cover - handled at runtime
    zmq = None

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import qos_profile_system_default

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from action_msgs.msg import GoalStatus


def _parse_bool(value) -> bool:
    """安全解析布尔值，兼容 LaunchConfiguration 传入的字符串。
    注意：bool('false') == True，所以必须用此函数。"""
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.lower() in ('true', '1', 'yes')
    return bool(value)


class WaypointPatrol(Node):
    def __init__(self):
        super().__init__('waypoint_patrol')

        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('loop', False)
        self.declare_parameter('wait_at_point', 5.0)
        self.declare_parameter('status_topic', '/inspection/status')
        self.declare_parameter('nav_timeout', 120.0)  # 单点导航超时(秒)
        self.declare_parameter('use_zmq_bridge', False)
        self.declare_parameter('zmq_goal_addr', 'tcp://192.168.50.2:5555')
        self.declare_parameter('zmq_status_addr', 'tcp://192.168.50.2:5556')

        self.waypoints_file = self.get_parameter('waypoints_file').value
        self.loop = _parse_bool(self.get_parameter('loop').value)
        self.wait_at_point = float(self.get_parameter('wait_at_point').value)
        self.status_topic = self.get_parameter('status_topic').value
        self.nav_timeout = float(self.get_parameter('nav_timeout').value)
        self.use_zmq_bridge = _parse_bool(self.get_parameter('use_zmq_bridge').value)
        self.zmq_goal_addr = self.get_parameter('zmq_goal_addr').value
        self.zmq_status_addr = self.get_parameter('zmq_status_addr').value

        # 使用 ReentrantCallbackGroup 允许 Action 回调在 executor 中并发处理
        self._cb_group = ReentrantCallbackGroup()

        self.status_pub = self.create_publisher(
            String, self.status_topic, qos_profile_system_default)

        self.nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=self._cb_group)

        self._active_goal_handle = None  # 用于关闭时取消活跃导航目标
        self._zmq_context = None
        self._zmq_goal_socket = None
        self._zmq_status_socket = None
        self._zmq_poller = None
        self._zmq_goal_seq = 0

        if self.use_zmq_bridge:
            if zmq is None:
                self.get_logger().error('pyzmq not installed. Disable use_zmq_bridge or install pyzmq.')
                self.use_zmq_bridge = False
            else:
                self._zmq_context = zmq.Context()
                self._zmq_goal_socket = self._zmq_context.socket(zmq.PUSH)
                self._zmq_goal_socket.connect(self.zmq_goal_addr)
                self._zmq_status_socket = self._zmq_context.socket(zmq.SUB)
                self._zmq_status_socket.connect(self.zmq_status_addr)
                self._zmq_status_socket.setsockopt_string(zmq.SUBSCRIBE, '')
                self._zmq_poller = zmq.Poller()
                self._zmq_poller.register(self._zmq_status_socket, zmq.POLLIN)
                self.get_logger().info(
                    f'ZMQ bridge enabled: goal={self.zmq_goal_addr}, status={self.zmq_status_addr}')

        self.waypoints = self._load_waypoints(self.waypoints_file)
        if not self.waypoints:
            self.get_logger().error('No waypoints loaded! Check waypoints_file parameter.')
            return

        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints, loop={self.loop}')

        # 启动巡逻线程（独立于 executor spin，避免死锁）
        self._patrol_thread = threading.Thread(
            target=self._patrol_worker, daemon=True)
        self._patrol_thread.start()

    def _load_waypoints(self, filepath: str):
        if not filepath:
            self.get_logger().warn('waypoints_file not set, using default demo points')
            return [
                {'name': 'A', 'x': 1.0, 'y': 0.0, 'yaw': 0.0},
                {'name': 'B', 'x': 2.0, 'y': 1.0, 'yaw': 1.57},
                {'name': 'C', 'x': 0.0, 'y': 1.0, 'yaw': 3.14},
            ]
        try:
            with open(filepath, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
            return data.get('waypoints', [])
        except Exception as exc:
            self.get_logger().error(f'Failed to load waypoints: {exc}')
            return []

    # ---- 巡逻线程 ----

    def _patrol_worker(self):
        """巡逻主入口（在独立线程中运行，不阻塞 ROS2 回调处理）"""
        # 等待系统初始化
        time.sleep(3.0)

        if self.use_zmq_bridge:
            self.get_logger().info('ZMQ bridge enabled. Skipping Nav2 action server wait.')
        else:
            self.get_logger().info('Waiting for Nav2 action server (timeout 60s)...')
            if not self.nav_client.wait_for_server(timeout_sec=60.0):
                self.get_logger().error('Nav2 action server not available! Check OrangePi.')
                self._publish_status('error', 'Nav2 not available')
                return

        self.get_logger().info('Nav2 ready. Starting patrol.')
        self._run_patrol()

    def _run_patrol(self):
        round_count = 0
        while rclpy.ok():
            round_count += 1
            self.get_logger().info(f'=== Patrol round {round_count} ===')
            self._publish_status('patrol_start', f'round {round_count}')

            for idx, wp in enumerate(self.waypoints):
                if not rclpy.ok():
                    return

                name = wp.get('name', f'WP{idx}')
                x = float(wp['x'])
                y = float(wp['y'])
                yaw = float(wp.get('yaw', 0.0))

                self.get_logger().info(
                    f'Navigating to [{name}] ({x:.2f}, {y:.2f}, yaw={yaw:.2f})')
                self._publish_status('navigating', name)

                success = self._navigate_to(x, y, yaw)

                if not rclpy.ok():
                    return

                if not success:
                    self.get_logger().warn(f'Failed to reach [{name}], skipping.')
                    self._publish_status('nav_failed', name)
                    continue

                self.get_logger().info(
                    f'Arrived at [{name}], inspecting for {self.wait_at_point}s...')
                self._publish_status('inspecting', name)

                # 非阻塞等待（允许 Ctrl+C 退出）
                wait_end = time.time() + self.wait_at_point
                while time.time() < wait_end and rclpy.ok():
                    time.sleep(0.5)

                self._publish_status('inspect_done', name)

            self._publish_status('patrol_done', f'round {round_count}')

            if not self.loop:
                self.get_logger().info('Patrol complete (no loop).')
                break

        self._publish_status('finished', 'all done')
        self.get_logger().info('Patrol finished.')

    def _navigate_to(self, x: float, y: float, yaw: float) -> bool:
        """发送导航目标并等待结果。
        在巡逻线程中运行，通过轮询 future.done() 等待，
        由 MultiThreadedExecutor 在主线程处理 Action 回调。"""
        if self.use_zmq_bridge:
            return self._navigate_to_zmq(x, y, yaw)

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0

        # yaw -> quaternion
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        # 异步发送目标（附加 feedback 回调）
        self._nav_feedback_distance = -1.0
        send_future = self.nav_client.send_goal_async(
            goal, feedback_callback=self._nav_feedback_cb)

        # 轮询等待目标被接受（executor 会在其线程中处理回调并完成 future）
        while not send_future.done() and rclpy.ok():
            time.sleep(0.1)

        if not rclpy.ok():
            return False

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn('Navigation goal was rejected.')
            return False

        self._active_goal_handle = goal_handle
        self.get_logger().info(
            f'Navigation goal accepted, waiting for result (timeout={self.nav_timeout}s)...')

        # 轮询等待导航结果，带超时保护
        result_future = goal_handle.get_result_async()
        log_interval = 5.0
        last_log_time = time.time()
        nav_start_time = time.time()

        while not result_future.done() and rclpy.ok():
            time.sleep(0.5)

            # 超时检测
            elapsed = time.time() - nav_start_time
            if elapsed > self.nav_timeout:
                self.get_logger().warn(
                    f'Navigation timeout ({self.nav_timeout}s)! Cancelling goal...')
                self._cancel_active_goal()
                return False

            # 定期打印进度
            if time.time() - last_log_time >= log_interval:
                last_log_time = time.time()
                if self._nav_feedback_distance >= 0:
                    self.get_logger().info(
                        f'  Remaining distance: {self._nav_feedback_distance:.2f}m '
                        f'({elapsed:.0f}s/{self.nav_timeout:.0f}s)')

        self._active_goal_handle = None

        if not rclpy.ok():
            return False

        result = result_future.result()
        if result is None:
            self.get_logger().warn('Navigation result is None (timeout?).')
            return False

        return result.status == GoalStatus.STATUS_SUCCEEDED

    def _navigate_to_zmq(self, x: float, y: float, yaw: float) -> bool:
        if self._zmq_goal_socket is None or self._zmq_status_socket is None:
            self.get_logger().error('ZMQ sockets not initialized.')
            return False

        self._zmq_goal_seq += 1
        goal_id = f'wp_{self._zmq_goal_seq}_{int(time.time() * 1000)}'
        payload = {
            'type': 'nav_goal',
            'goal_id': goal_id,
            'x': x,
            'y': y,
            'yaw': yaw,
            'frame_id': 'map',
            'timestamp': time.time(),
        }

        try:
            self._zmq_goal_socket.send_string(json.dumps(payload, ensure_ascii=False))
        except Exception as exc:
            self.get_logger().error(f'Failed to send ZMQ goal: {exc}')
            return False

        self.get_logger().info(
            f'ZMQ goal sent: id={goal_id} ({x:.2f}, {y:.2f}, yaw={yaw:.2f})')

        start_time = time.time()
        while rclpy.ok():
            elapsed = time.time() - start_time
            if elapsed > self.nav_timeout:
                self.get_logger().warn(
                    f'ZMQ navigation timeout ({self.nav_timeout}s) for goal {goal_id}')
                return False

            try:
                events = dict(self._zmq_poller.poll(500))
            except Exception:
                events = {}

            if self._zmq_status_socket in events:
                try:
                    raw = self._zmq_status_socket.recv_string()
                    msg = json.loads(raw)
                except Exception:
                    continue

                if msg.get('type') != 'nav_result':
                    continue
                if msg.get('goal_id') != goal_id:
                    continue

                status = int(msg.get('status', -1))
                if status == GoalStatus.STATUS_SUCCEEDED:
                    return True
                self.get_logger().warn(f'ZMQ nav failed (status={status}) for goal {goal_id}')
                return False

        return False

    def _cancel_active_goal(self):
        """取消当前活跃的导航目标。"""
        if self._active_goal_handle is not None:
            try:
                self._active_goal_handle.cancel_goal_async()
                self.get_logger().info('Active navigation goal cancelled.')
            except Exception:
                pass
            self._active_goal_handle = None

    def _nav_feedback_cb(self, feedback_msg):
        """接收 Nav2 导航反馈，提取剩余距离。"""
        try:
            fb = feedback_msg.feedback
            if hasattr(fb, 'distance_remaining'):
                self._nav_feedback_distance = float(fb.distance_remaining)
        except Exception:
            pass

    def _publish_status(self, state: str, detail: str = ''):
        msg = String()
        msg.data = json.dumps({'state': state, 'detail': detail}, ensure_ascii=False)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointPatrol()

    # 使用多线程执行器：主线程处理 ROS 回调/Action，巡逻逻辑在独立线程
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down waypoint patrol...')
        node._cancel_active_goal()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
