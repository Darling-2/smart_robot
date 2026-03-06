#!/usr/bin/env python3
# coding: utf-8
"""ZeroMQ bridge for ROS1 move_base navigation.

- Receives nav goals over ZMQ (PULL) and publishes /move_base_simple/goal
- Publishes move_base result over ZMQ (PUB)
"""

import json
import math
import time
import threading

import rospy
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult

try:
    import zmq
except ImportError:  # pragma: no cover
    zmq = None


def _yaw_to_quat(yaw: float):
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return qz, qw


class ZmqNavBridge:
    def __init__(self):
        if zmq is None:
            raise RuntimeError('pyzmq is not installed.')

        self.goal_topic = rospy.get_param('~goal_topic', '/move_base_simple/goal')
        self.result_topic = rospy.get_param('~result_topic', '/move_base/result')
        self.bind_goal = rospy.get_param('~bind_goal', 'tcp://0.0.0.0:5555')
        self.bind_status = rospy.get_param('~bind_status', 'tcp://0.0.0.0:5556')
        self.frame_id = rospy.get_param('~frame_id', 'map')

        self._goal_pub = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=10)
        self._result_sub = rospy.Subscriber(
            self.result_topic, MoveBaseActionResult, self._result_cb, queue_size=10
        )

        self._context = zmq.Context()
        self._goal_pull = self._context.socket(zmq.PULL)
        self._goal_pull.bind(self.bind_goal)

        self._status_pub = self._context.socket(zmq.PUB)
        self._status_pub.bind(self.bind_status)

        self._poller = zmq.Poller()
        self._poller.register(self._goal_pull, zmq.POLLIN)

        self._last_goal_id = ''

        self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._recv_thread.start()

        rospy.loginfo(
            'ZMQ nav bridge started: goal=%s status=%s',
            self.bind_goal,
            self.bind_status,
        )

    def _recv_loop(self):
        while not rospy.is_shutdown():
            try:
                events = dict(self._poller.poll(200))
            except Exception:
                continue

            if self._goal_pull not in events:
                continue

            try:
                raw = self._goal_pull.recv_string()
                msg = json.loads(raw)
            except Exception:
                continue

            if msg.get('type') != 'nav_goal':
                continue

            goal_id = msg.get('goal_id', '')
            x = float(msg.get('x', 0.0))
            y = float(msg.get('y', 0.0))
            yaw = float(msg.get('yaw', 0.0))
            frame_id = msg.get('frame_id', self.frame_id)

            pose = PoseStamped()
            pose.header.frame_id = frame_id
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = x
            pose.pose.position.y = y
            qz, qw = _yaw_to_quat(yaw)
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw

            self._goal_pub.publish(pose)
            self._last_goal_id = goal_id

    def _result_cb(self, msg: MoveBaseActionResult):
        payload = {
            'type': 'nav_result',
            'goal_id': self._last_goal_id,
            'status': int(msg.status.status),
            'text': msg.status.text,
            'timestamp': time.time(),
        }
        try:
            self._status_pub.send_string(json.dumps(payload, ensure_ascii=False))
        except Exception:
            pass


def main():
    rospy.init_node('zmq_nav_bridge', anonymous=False)
    try:
        ZmqNavBridge()
    except Exception as exc:
        rospy.logerr('ZMQ nav bridge failed to start: %s', exc)
        return
    rospy.spin()


if __name__ == '__main__':
    main()
