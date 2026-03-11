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
from hub_interfaces.action import GetColor

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
        self._action_server = ActionServer(self, GetColor, 'detect_color', self.read_antenna_color_execute, cancel_callback=lambda req: rclpy.action.CancelResponse.ACCEPT)

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
                    
    def read_antenna_color_execute(self, goal_handle: rclpy.action.server.ServerGoalHandle):
        
        # Use a queue to communicate between the callback and this execution thread
        self.get_logger().info("read_antenna_color called")
        image_queue = SimpleQueue()
        last_msg = None

        def callback(msg: sensor_msgs.msg.Image, frame: np.ndarray):
            nonlocal last_msg
            nonlocal image_queue

            current_stamp = Time.from_msg(msg.header.stamp)

            if last_msg is not None:
                last_stamp, last_frame = last_msg
                if last_stamp >= current_stamp:
                    self.get_logger().warn("Received out-of-order frame. Ignoring.")
                    return
                image_queue.put(frame)

            last_msg = (current_stamp, np.copy(frame))
            
        name = f"read_antenna_color"
        self._register_callback(callback, name)
                
        # Wait for frame
        success = False
        img = image_queue.get()
        color = self.get_color(img, goal_handle)
        while not goal_handle.is_cancel_requested:
            if color != 0:
                self.get_logger().info(f"Antenna Color Found something something")
                success = True
                break
            else:
                self.get_logger().info(f"Antenna Color no find something something")
                img = image_queue.get()
                color = self.get_color(img, goal_handle)
        else:
            goal_handle.abort()
            self.get_logger().info("get color action canceled")
            
        self._unregister_callback(name)
        
        if success:
            goal_handle.succeed()
        else:
            goal_handle.canceled()
            
        # return something
        self.get_logger().info(f"Color: {color}")
        self.get_logger().info("sending color result")
        response = GetColor.Result()
        response.color = int(color)
        return response

    def get_color(self, img, goal_handle):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 250, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        masked_img = img

        if contours:
            # Find the largest contour
            largest_contour = max(contours, key=cv2.contourArea)

            # Calculate Moments for the largest contour
            M = cv2.moments(largest_contour)

            # Calculate x,y coordinates of the centroid
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])


                (x, y), radius = cv2.minEnclosingCircle(largest_contour)
                center = (int(x), int(y))
                mask = np.zeros(img.shape[:2], dtype=np.uint8)
                cv2.circle(mask, center, int(radius + 50), 255, -1)

                masked_img = cv2.bitwise_and(img, img, mask=mask)
            else:
                self.get_logger().info("No contour found")
                return 0
                # goal_handle.abort()

        # masked_img_n = np.array(masked_img)

        # B = masked_img[:,:,0]
        # G = masked_img[:,:,1]
        # R = masked_img[:,:,2]
        B, G, R = cv2.split(masked_img)

        # Remove black pixels
        not_black = ~((B <= 20) & (G <= 20) & (R <= 20))

        # Remove White Pixels
        not_white = ~((B >= 200) & (G >= 200) & (R >= 200))

        mask = not_black & not_white

        mask_uint8 = (mask.astype(np.uint8)) * 255

        if np.count_nonzero(mask) < 50:
            self.get_logger().info("Bad mask")
            return 0

        masked_pixels = masked_img[mask_uint8 > 0]

        # 3. Calculate the Average B, G, and R
        # Using the mean of only the masked pixels
        avg_bgr = np.mean(masked_pixels, axis=0)
        avg_b = avg_bgr[0]
        avg_g = avg_bgr[1]
        avg_r = avg_bgr[2]

        # 4. The "Purple vs Blue" logic
        # Purple has a high Blue AND a significant Red component.
        # Blue has a high Blue but very low Red.

        # Calculate Red-to-Blue ratio
        rb_ratio = avg_r / (avg_b + 1e-5) # avoid division by zero

        # 5. Categorize based on Hue and Ratios
        # We'll convert the average BGR to HSV for the primary hue check
        avg_pixel_bgr = np.uint8([[avg_bgr]])
        avg_hsv = cv2.cvtColor(avg_pixel_bgr, cv2.COLOR_BGR2HSV)[0][0]

        h = avg_hsv[0] # Hue
        s = avg_hsv[1] # Saturation
        v = avg_hsv[2] # Value/Brightness

        b, g, r = avg_bgr

        # 2. Logic for Green vs. Aquamarine vs. Blue
        
        # Check for Green/Aquamarine first
        if 40 <= h <= 105:
            # If Hue is > 85, it's leaning blue (Aquamarine)
            if h > 85:
                # Check if Green is still significantly higher than Blue
                if g > b:
                    # return "Aquamarine/Light Green"
                    return ord('G')
                else:
                    # return "Light Blue"
                    return ord('B')
            return ord('G')

        # 3. Check for Blue vs. Purple
        if 105 < h <= 135:
            # Check Red-to-Blue ratio for Purple as discussed before
            if r / (b + 1e-5) > 0.5:
                return ord('P')
            return ord('B')

        # 4. Check for Red
        if h <= 15 or h >= 160:
            return ord('R')

        # goal_handle.abort()
        self.get_logger().info("No color detected")
        return 0


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
