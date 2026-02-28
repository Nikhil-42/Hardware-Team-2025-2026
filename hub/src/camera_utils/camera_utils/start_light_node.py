#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.time import Time
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import Image
from hub_interfaces.action import StartLight

from cv_bridge import CvBridge
import cv2
import numpy as np

from queue import SimpleQueue


class StartLightNode(Node):
    """
    Service: /arm_start_light (hub_interfaces/srv/ArmStartLight)
      arm=true  -> start scanning
      arm=false -> stop scanning

    Subscribes:
      image topic (sensor_msgs/Image), e.g. /image_raw or /camera/image_raw

    Publishes:
      /start_light/detected (Bool)
      /start_light/max_brightness (Float32)
    """

    def __init__(self):
        super().__init__('start_light_node')
        self._image_topic = self.declare_parameter('image_topic', '/camera/image_raw').get_parameter_value().string_value
        self._threshold = self.declare_parameter('threshold', 800.0).get_parameter_value().double_value
        self._cv2_bridge = CvBridge()

        self._last_msg = None
        self._dbrightness_queue = SimpleQueue()

        # Action Server
        self._action_server = ActionServer(self, StartLight, 'start_light', self.start_light_callback)

        self.get_logger().info(f"StartLightNode ready. Listening to {self._image_topic}. Service: /start_light")

    def image_callback(self, msg: Image):
        try:
            frame_bgr = self._cv2_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"cv_bridge conversion failed: {e}")
            return

        current_stamp = Time.from_msg(msg.header.stamp)
        height, width = frame_bgr.shape[:2]
        cropped_bgr = frame_bgr[height//4:(3*height)//4, (7*width)//8:]
        cropped_gray = cv2.cvtColor(cropped_bgr, cv2.COLOR_BGR2GRAY)

        if self._last_msg is not None:
            last_stamp, last_frame = self._last_msg
            diff = np.clip(cv2.subtract(cropped_gray, last_frame), a_min=0, a_max=None)
            dbrightness = np.mean(diff) / (current_stamp.nanoseconds - last_stamp.nanoseconds) * 1e9
            self._dbrightness_queue.put(dbrightness)

        self._last_msg = (current_stamp, np.copy(cropped_gray))

    def start_light_callback(self, goal_handle):
        self.get_logger().info(f"Received StartLight action request: threshold={self._threshold}")

        # Clear any old dbrightness values
        for _ in range(self._dbrightness_queue.qsize()):
            try:
                self._dbrightness_queue.get_nowait()
            except:
                break
        
        # Subscribe to camera_ros image stream
        image_sub = self.create_subscription(Image, self._image_topic, self.image_callback, 10)

        # Wait for frame
        dbrightness = self._dbrightness_queue.get()
        while dbrightness < self._threshold:
            dbrightness = self._dbrightness_queue.get()

        # Stop subscription
        self.destroy_subscription(image_sub)
        goal_handle.succeed()
        response = StartLight.Result()
        return response


def main(args=None):
    rclpy.init(args=args)
    node = StartLightNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()