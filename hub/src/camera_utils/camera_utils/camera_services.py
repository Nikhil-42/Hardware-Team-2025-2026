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
        color = self.detect_led_color(img)
        while not goal_handle.is_cancel_requested:
            if color != 0:
                self.get_logger().info(f"Antenna Color Found something something")
                success = True
                break
            else:
                self.get_logger().info(f"Antenna Color no find something something")
                img = image_queue.get()
                color = self.detect_led_color(img)
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

    def detect_led_color(self, img, color_threshold=None, blue_green_ratio=None):
        # if color_threshold is None or blue_green_ratio is None:
        #     ct, bgr = load_config()
        #     color_threshold  = color_threshold  or ct
        #     blue_green_ratio = blue_green_ratio or bgr
        
        color_threshold = 33.5
        blue_green_ratio = 1.33

        # --- Step 1: Find the brightest spot to confirm LED is on ---
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray_blur = cv2.GaussianBlur(gray, (5, 5), 0)
        _, max_val, _, max_loc = cv2.minMaxLoc(gray_blur)
        # print(f"Peak brightness: {max_val} at {max_loc}")

        # --- Step 2: Check if LED is OFF ---
        if max_val < 80:
            # print("Result: LED is OFF (no bright spot detected)")
            return 0

        # --- Step 3: Threshold to find white blob, use its CENTROID as center ---
        _, bright_mask = cv2.threshold(gray_blur, 240, 255, cv2.THRESH_BINARY)
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(bright_mask)

        bx, by = max_loc
        cx, cy = bx, by
        blob_radius = 1

        for i in range(1, num_labels):
            x, y, w, h, area = stats[i]
            if x <= bx <= x + w and y <= by <= y + h:
                cx, cy = int(centroids[i][0]), int(centroids[i][1])
                blob_radius = int(np.sqrt(area / np.pi))
                # print(f"Blob centroid: ({cx}, {cy}), area: {area}px, effective radius: {blob_radius}px")
                break
            
        if blob_radius < 7:
            self.get_logger().info(f"blob_radius: {blob_radius}")
            return 0

        inner_radius = blob_radius + 5
        outer_radius = inner_radius + 30
        # print(f"Sampling ring: inner={inner_radius}px, outer={outer_radius}px")

        # --- Step 4: Sample an annular ring around the centroid ---
        h_img, w_img = img.shape[:2]
        Y, X = np.ogrid[:h_img, :w_img]
        dist = np.sqrt((X - cx)**2 + (Y - cy)**2)
        ring_mask = (dist >= inner_radius) & (dist <= outer_radius)
        ring_pixels = img[ring_mask]

        if len(ring_pixels) == 0:
            # print("Result: LED is OFF or too dim")
            return 0

        # --- Step 5: Average the color in the ring ---
        avg_bgr = ring_pixels.mean(axis=0)
        b, g, r = avg_bgr
        # print(f"Average BGR in ring: B={b:.1f}, G={g:.1f}, R={r:.1f}")

        # --- Step 6: Remove white baseline to isolate true color signal ---
        self.get_logger().info(f"red: {r} green: {g} blue: {b}")
        white_baseline = min(r, g, b)
        cr = r - white_baseline
        cg = g - white_baseline
        cb = b - white_baseline
        # print(f"After white removal: cR={cr:.1f}, cG={cg:.1f}, cB={cb:.1f}")

        # --- Step 7: Classify ---
        r_on = cr > color_threshold/2
        g_on = cg > color_threshold
        b_on = cb > color_threshold

        self.get_logger().info(f"red: {cr} green: {cg} blue: {cb}")
        
        found = False

        if not r_on and not g_on and not b_on:
            color = 0
        elif g_on and b_on:
            # Blue LED bleeds green — use ratio to distinguish
            self.get_logger().info("Found Blue Green")
            color = ord('B') if cb > cg * blue_green_ratio else ord('G')
            found = True
        elif g_on:
            self.get_logger().info("Found Green")
            color = ord('G')
            found = True
        elif r_on and b_on:
            self.get_logger().info("Found Red Blue")
            color = ord('P')
            found = True
        elif r_on:
            self.get_logger().info("Found Red")
            color = ord('R')
            found = True
        elif b_on:
            self.get_logger().info("Found Blue")
            color = ord('B')
            found = True
        else:
            # dominant = np.argmax([cr, cg, cb])
            color = 0
            # color = ["red", "green", "blue"][dominant] + " (ambiguous)"

        # print(f"Result: LED color is → {color.upper()}")

        # --- Step 8: Annotate and save debug image ---
        if (found):
            debug = img.copy()
            cv2.circle(debug, (cx, cy), inner_radius, (0, 255, 255), 2)
            cv2.circle(debug, (cx, cy), outer_radius, (0, 255, 0), 2)
            cv2.circle(debug, (cx, cy), 3, (255, 0, 255), -1)
            # cv2.putText(debug, color.upper(), (cx + outer_radius + 5, cy),
            #             cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.imwrite("/home/ieee/Hardware-Team-2025-2026/hub/src/debug_led.png", debug)
            self.get_logger().info("Debug image saved to debug_led.png")

        # print(f"Color: {color}")

        return color


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
