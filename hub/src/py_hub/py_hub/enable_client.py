import sys

from hub_interfaces.srv import Enable
import rclpy
from rclpy.node import Node


class EnableClientAsync(Node):

    def __init__(self):
        super().__init__('enable_client_async')
        self.cli = self.create_client(Enable, 'enable')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Enable.Request()

    def send_request(self, state):
        self.req.state = state
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    minimal_client = EnableClientAsync()
    response = minimal_client.send_request(bool(int(sys.argv[1])))
    minimal_client.get_logger().info(f'Result of enable={sys.argv[1]} {response.success}')

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()