import rclpy
from rclpy.node import Node
from time import sleep
from hub_interfaces.srv import Finger, Gate
from gpiozero import LED, Motor, Servo

class ManipulationService(Node):
    def __init__(self):
        super().__init__('finger_service')
        # Quickly extends and retracts the specified finger
        self.srv = self.create_service(Finger, 'click', self.click)
        # Expels the specified finger from the hand
        self.srv = self.create_service(Finger, 'eject', self.eject)
        # Retracts a new finger into the hand
        self.srv = self.create_service(Finger, 'retract', self.retract)
        self.srv = self.create_service(Gate, 'gate', self.gate)
        
        self.enable_fingers = LED(18)
        self.enable_fingers.off()
        
        self.finger_motors = [
            Motor(23, 24),  # Finger 1
            Motor(12, 25),   # Finger 2
            Motor(4, 17),   # Finger 3
            Motor(22, 27),  # Finger 4
        ]

        self.gate_servos = [
            Servo(5, min_pulse_width=0.0006, max_pulse_width=0.0014),  # Gate 1
            Servo(6, min_pulse_width=0.0016, max_pulse_width=0.0024),  # Gate 2
        ]

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
        if request.idx < 1 or request.idx > 4:
            self.get_logger().info('Invalid finger index: %d' % (request.idx))
            response.success = False
            return response
        
        finger_index = request.idx - 1
        m = self.finger_motors[finger_index]
        
        self.enable_fingers.on()
        m.forward()
        sleep(3.0)
        m.stop()
        self.enable_fingers.off()
        self.get_logger().info(f"Ejected finger {request.idx}")
        response.success = True
        return response
        

    def retract(self, request: Finger.Request, response: Finger.Response):
        if request.idx < 1 or request.idx > 4:
            self.get_logger().info('Invalid finger index: %d' % (request.idx))
            response.success = False
            return response
        
        finger_index = request.idx - 1
        m = self.finger_motors[finger_index]
        
        self.enable_fingers.on()
        m.backward()
        sleep(1.5)
        m.stop()
        self.enable_fingers.off()
        self.get_logger().info(f"Retracted finger {request.idx}")
        response.success = True
        return response

    def click(self, request: Finger.Request, response: Finger.Response):
        if request.idx < 1 or request.idx > 4:
            self.get_logger().info('Invalid finger index: %d' % (request.idx))
            response.success = False
            return response
        
        finger_index = request.idx - 1
        m = self.finger_motors[finger_index]
        self.get_logger().info(f"Beep [pin {request.idx}]")
        self.enable_fingers.on()
        m.forward()
        sleep(1.0)
        m.backward()
        sleep(1.0)
        m.stop()
        self.enable_fingers.off()
        self.get_logger().info(f"Boop [pin {request.idx}]")
        response.success = True        
        return response


def main(args=None):
    rclpy.init(args=args)

    finger_service = ManipulationService()

    rclpy.spin(finger_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()