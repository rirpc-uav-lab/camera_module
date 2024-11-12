import sys

from uav_msgs.srv import Micasense
import rclpy
from rclpy.node import Node


class TestClient(Node):

    def __init__(self):
        super().__init__('test_client')
        self.cli = self.create_client(Micasense, 'commands_service')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Micasense.Request()

    def send_request(self):
        self.req.cmd = 1
        
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    minimal_client = TestClient()
    response = minimal_client.send_request()

    print(response.result)
   
    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()