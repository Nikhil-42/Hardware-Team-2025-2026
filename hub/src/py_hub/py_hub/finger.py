from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node
from time import sleep
from hub_interfaces.srv import Finger
from gpiozero import LED, Motor

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(Finger, 'finger', self.finger)
        self.enable_fingers = LED(18)
        self.enable_fingers.off()
        
        self.finger_motors = [
            Motor(23, 24),  # Finger 1
            Motor(12, 25),   # Finger 2
            Motor(4, 17),   # Finger 3
            Motor(22, 27),  # Finger 4
        ]

    def finger(self, request: Finger.Request, response: Finger.Response):
        if request.idx < 1 or request.idx > 4:
            self.get_logger().info('Invalid finger index: %d' % (request.idx))
            response.success = False
            return response
        
        self.enable_fingers.on()
        finger_index = request.idx - 1
        m = self.finger_motors[finger_index]
        m.forward()
        self.get_logger().info(f"Beep [pin {request.idx}]")
        sleep(0.25)
        m.backward()
        sleep(0.25)
        self.get_logger().info(f"Boop [pin {request.idx}]")
        m.stop()
        self.enable_fingers.off()
        response.success = True        
        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()