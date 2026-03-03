#!/usr/bin/env python3

from typing import Callable

import rclpy
import rclpy.action
from rclpy.node import Node
from rclpy.action import ActionServer 
from rclpy.time import Time
from rclpy.executors import MultiThreadedExecutor

import sensor_msgs
from sensor_msgs.msg import Image
from hub_interfaces.action import Start

from cv_bridge import CvBridge
import cv2
import numpy as np

from queue import SimpleQueue
from threading import Lock

import sensor_msgs.msg

class CameraServicesNode(Node):
    """
    Action: /start_light (hub_interfaces/srv/Start)
    Action: /detect_color (hub_interfaces/srv/DetectColor)
    """

    def __init__(self):
        super().__init__('camera_services_node')
        self._image_topic = self.declare_parameter('image_topic', '/camera/image_raw').get_parameter_value().string_value
        self._threshold = self.declare_parameter('threshold', 800.0).get_parameter_value().double_value
        self._cv2_bridge = CvBridge()

        self._image_sub = None

        self._running = Lock()
        self._callbacks = {}

        # Action Server
        self._action_server = ActionServer(self, Start, 'start_light', self.start_light_execute, cancel_callback=lambda req: rclpy.action.CancelResponse.ACCEPT)

        self.get_logger().info(f"CameraServicesNode ready. Listening to {self._image_topic}.")
    
    def _register_callback(self, callback: Callable[[sensor_msgs.msg.Image, np.ndarray], None], name: str):
        with self._running:
            if name in self._callbacks:
                raise ValueError(f"Callback with name {name} already registered")
            
            self._callbacks[name] = callback

            if self._image_sub is None:
                self._image_sub = self.create_subscription(Image, self._image_topic, self.image_callback, 10)

    def _unregister_callback(self, name: str):
        with self._running:
            if name in self._callbacks:
                del self._callbacks[name]
            else:
                raise ValueError(f"No callback with name {name} registered")

            if len(self._callbacks) == 0 and self._image_sub is not None:
                self.destroy_subscription(self._image_sub)
                self._image_sub = None

    def start_light_execute(self, goal_handle: rclpy.action.server.ServerGoalHandle):
        self.get_logger().info(f"Received StartLight action request: threshold={self._threshold}")

        # Use a queue to communicate between the callback and this execution thread
        dbrightness_queue = SimpleQueue()
        last_msg = None

        def callback(msg: sensor_msgs.msg.Image, frame: np.ndarray):
            nonlocal last_msg
            nonlocal dbrightness_queue

            current_stamp = Time.from_msg(msg.header.stamp)
            height, width = frame.shape[:2]
            cropped_bgr = frame[height//4:(3*height)//4, (7*width)//8:]
            cropped_gray = cv2.cvtColor(cropped_bgr, cv2.COLOR_BGR2GRAY)

            if last_msg is not None:
                last_stamp, last_frame = last_msg
                if last_stamp >= current_stamp:
                    self.get_logger().warn("Received out-of-order frame. Ignoring.")
                    return
                diff = np.clip(cv2.subtract(cropped_gray, last_frame), a_min=0, a_max=None)
                dbrightness = np.mean(diff) / (current_stamp.nanoseconds - last_stamp.nanoseconds) * 1e9
                dbrightness_queue.put(dbrightness)

            last_msg = (current_stamp, np.copy(cropped_gray))
            
        # Register callback
        name = f"start_light_{goal_handle.goal_id}"
        self._register_callback(callback, name)

        # Wait for frame
        success = False
        dbrightness = dbrightness_queue.get()
        while not goal_handle.is_cancel_requested:
            if dbrightness > self._threshold:
                self.get_logger().info(f"Light detected! dBrightness={dbrightness:.2f}")
                success = True
                break
            else:
                dbrightness = dbrightness_queue.get()
        else:
            self.get_logger().info("StartLight action canceled")

        # Stop subscription
        self._unregister_callback(name)

        if success:
            goal_handle.succeed()
        else:
            goal_handle.canceled()
    
        response = Start.Result()
        return response

    def image_callback(self, msg: sensor_msgs.msg.Image):
        try:
            frame = self._cv2_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"cv_bridge conversion failed: {e}")
            return
        
        with self._running:
            for callback in self._callbacks.values():
                try:
                    callback(msg, frame)
                except Exception as e:
                    self.get_logger().warn(f"Callback raised exception: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = CameraServicesNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()