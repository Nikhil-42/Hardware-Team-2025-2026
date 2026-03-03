import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from rclpy.action.server import ServerGoalHandle
from time import sleep
from hub_interfaces.srv import Enable, Finger, Gate, ReportColor
from hub_interfaces.action import Start
from py_hub.lcd_driver import LCD
from gpiozero import LED, Motor, Servo, Button


class ManipulationService(Node):
    def __init__(self):
        super().__init__('manipulation_service')
        self.enable_srv = self.create_service(Enable, 'enable', self.enable)
        # Quickly extends and retracts the specified finger
        self.click_srv = self.create_service(Finger, 'click', self.click)
        # Expels the specified finger from the hand
        self.eject_srv = self.create_service(Finger, 'eject', self.eject)
        # Retracts a new finger into the hand
        self.retract_srv = self.create_service(Finger, 'retract', self.retract)
        self.gate_srv = self.create_service(Gate, 'gate', self.gate)
        # Displays colors on LCD Screen
        self.lcd_srv = self.create_service(ReportColor, 'report_lcd', self.report_lcd)
        # Start button action server
        self._action_cb_group = ReentrantCallbackGroup()
        self.start_button_actsrv = ActionServer(self, Start, 'start_button', self.start_button_callback, cancel_callback=lambda req: CancelResponse.ACCEPT, callback_group=self._action_cb_group)
        
        self.start_button = Button(21, pull_up=True, bounce_time=0.1)
        self.enable_pin = LED(18)
        self.enable_pin.off()
        
        self.finger_motors = [
            Motor(23, 24),  # Finger 1
            Motor(12, 25),   # Finger 2
            Motor(4, 17),   # Finger 3
            Motor(22, 27),  # Finger 4
        ]

        self.finger_map = {
            3: 0,
            7: 2,
            8: 1,
            -1: 3,
        }

        self.gate_servos = [
            Servo(5, min_pulse_width=0.0006, max_pulse_width=0.0014),  # Gate 1
            Servo(6, min_pulse_width=0.0016, max_pulse_width=0.0024),  # Gate 2
        ]

        self.colors = ["?", "?", "?", "?"]
        self.lcd_display = LCD()

        self.get_logger().info("Closing gates")
        self.gate_servos[1].min()
        sleep(0.5)
        self.gate_servos[0].max()

    def enable(self, request: Enable.Request, response: Enable.Response):
        if request.state:
            self.enable_pin.on()
        else:
            self.enable_pin.off()
        response.success = True
        return response

    def gate(self, request: Gate.Request, response: Gate.Response):
        if request.open:
            self.get_logger().info("Opening gates")
            self.gate_servos[0].min()
            sleep(1.0)
            self.gate_servos[1].max()
        else:
            self.get_logger().info("Closing gates")
            self.gate_servos[1].min()
            sleep(1.0)
            self.gate_servos[0].max()
        response.success = True
        return response

    def eject(self, request: Finger.Request, response: Finger.Response):
        if request.idx not in self.finger_map:
            self.get_logger().info('Invalid finger: %d' % (request.idx))
            response.success = False
            return response
        
        finger_index = self.finger_map[request.idx]
        m = self.finger_motors[finger_index]
        
        m.forward()
        sleep(3.0)
        m.stop()
        self.get_logger().info(f"Ejected finger {request.idx}")
        response.success = True
        return response
        

    def retract(self, request: Finger.Request, response: Finger.Response):
        if request.idx not in self.finger_map:
            self.get_logger().info('Invalid finger: %d' % (request.idx))
            response.success = False
            return response
        
        finger_index = self.finger_map[request.idx]
        m = self.finger_motors[finger_index]
        
        m.backward()
        sleep(1.5)
        m.stop()
        self.get_logger().info(f"Retracted finger {request.idx}")
        response.success = True
        return response

    def click(self, request: Finger.Request, response: Finger.Response):
        if request.idx not in self.finger_map:
            self.get_logger().info('Invalid finger: %d' % (request.idx))
            response.success = False
            return response
        
        finger_index = self.finger_map[request.idx]
        m = self.finger_motors[finger_index]
        self.get_logger().info(f"Beep [pin {request.idx}]")
        m.forward()
        sleep(0.3)
        m.backward()
        sleep(0.5)
        m.stop()
        sleep(0.5)
        self.get_logger().info(f"Boop [pin {request.idx}]")
        response.success = True        
        return response
    
    def report_lcd(self, request: ReportColor.Request, response: ReportColor.Response):
        self.get_logger().info(f"LCD adding message: {chr(request.color)} at antenna {request.antenna}")
        self.colors[request.antenna - 1] = chr(request.color)
        line1 = "    ".join(str(i) for i in range(1, 4+1))
        line2 = "    ".join(self.colors)
        self.lcd_display.display_string(line1, self.lcd_display.LINE_1)
        self.lcd_display.display_string(line2, self.lcd_display.LINE_2)

        response.success = True
        return response
    
    def start_button_callback(self, goal_handle: ServerGoalHandle):
        def wait_with_cancel(fn, *args, **kwargs):
            while True:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return False

                if fn(*args, **kwargs):
                    return True

        if not wait_with_cancel(self.start_button.wait_for_release, timeout=1):
            return Start.Result()
        
        if not wait_with_cancel(self.start_button.wait_for_press, timeout=1):
            return Start.Result()
        
        if not wait_with_cancel(self.start_button.wait_for_release, timeout=1):
            return Start.Result()

        goal_handle.succeed()
        response = Start.Result()
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ManipulationService()

    executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor.spin()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()