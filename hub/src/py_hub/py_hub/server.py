import rclpy
from rclpy.node import Node
from time import sleep
from hub_interfaces.srv import Enable, Finger, Gate
from gpiozero import LED, Motor, Servo

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
        
        self.enable_pin = LED(18)
        self.enable_pin.off()
        
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
        if request.idx < 1 or request.idx > 4:
            self.get_logger().info('Invalid finger index: %d' % (request.idx))
            response.success = False
            return response
        
        finger_index = request.idx - 1
        m = self.finger_motors[finger_index]
        
        m.forward()
        sleep(3.0)
        m.stop()
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
        
        m.backward()
        sleep(1.5)
        m.stop()
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
        m.forward()
        sleep(0.25)
        m.backward()
        sleep(0.5)
        m.stop()
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