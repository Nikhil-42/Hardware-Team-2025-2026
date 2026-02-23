#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2  # still used for grayscale + minMaxLoc

from hub_interfaces.srv import ArmStartLight


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

        self._armed = False
        self._detected = False
        self._threshold = 240.0  # tune later

        self.bridge = CvBridge()

        # Service
        self.create_service(ArmStartLight, 'arm_start_light', self.arm_callback)

        # Publishers
        self.pub_detected = self.create_publisher(Bool, 'start_light/detected', 10)
        self.pub_max = self.create_publisher(Float32, 'start_light/max_brightness', 10)

        # Subscribe to camera_ros image stream
        # CHANGE THIS TOPIC if your camera publishes a different name
        image_topic = '/image_raw'
        self.sub = self.create_subscription(Image, image_topic, self.image_callback, 10)

        self.get_logger().info(f"StartLightNode ready. Listening to {image_topic}. Service: /arm_start_light")

    def arm_callback(self, request: ArmStartLight.Request, response: ArmStartLight.Response):
        self._armed = bool(request.arm)
        if self._armed:
            self._detected = False
            response.message = "Armed: scanning frames from ROS camera topic"
        else:
            response.message = "Disarmed: scanning stopped"

        response.success = True
        self.get_logger().info(response.message)
        return response

    def image_callback(self, msg: Image):
        # Always publish detected state so late subscribers see it
        self.pub_detected.publish(Bool(data=self._detected))

        if not self._armed:
            return

        try:
            frame_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"cv_bridge conversion failed: {e}")
            return

        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        _, max_val, _, max_loc = cv2.minMaxLoc(gray)

        max_val = float(max_val)  # 0..255
        self.pub_max.publish(Float32(data=max_val))

        if (not self._detected) and max_val >= self._threshold:
            self._detected = True
            self.get_logger().info(
                f"START LIGHT DETECTED: max={max_val:.1f} at (x={max_loc[0]}, y={max_loc[1]})"
            )
            self.pub_detected.publish(Bool(data=True))


def main(args=None):
    rclpy.init(args=args)
    node = StartLightNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()