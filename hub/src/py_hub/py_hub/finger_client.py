import sys

from hub_interfaces.srv import Finger
import rclpy
from rclpy.node import Node


class FingerClientAsync(Node):

    def __init__(self, action):
        super().__init__('finger_client_async')
        self.cli = self.create_client(Finger, action)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Finger.Request()

    def send_request(self, idx):
        self.req.idx = idx
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    minimal_client = FingerClientAsync(sys.argv[1])
    response = minimal_client.send_request(int(sys.argv[2]))
    minimal_client.get_logger().info(f'Result of {sys.argv[1]}: finger{sys.argv[2]} {response.success}')

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()