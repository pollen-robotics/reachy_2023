import sys

from zuuu_interfaces.srv import SetSpeed
import rclpy
from rclpy.node import Node
import time
import traceback


class SetSpeedServiceTest(Node):
    def __init__(self):
        super().__init__('set_speed_service_test')
        self.cli = self.create_client(SetSpeed, 'SetSpeed')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetSpeed.Request()
        self.t0 = time.time()
        self.create_timer(0.1, self.main_tick)
        self.state = 0

    def main_tick(self):
        self.get_logger().info("Drawing a square of 1x1m")
        self.req.duration = 2.0
        if time.time() < (self.t0 + 3.0):
            self.req.x_vel = 0.5
            self.req.y_vel = 0.0
            self.req.rot_vel = 0.0
            if self.state == 0:
                self.get_logger().info(f"{self.state}")
                self.cli.call_async(self.req)
                self.state += 1
        elif time.time() < (self.t0 + 6.0):
            self.req.x_vel = 0.0
            self.req.y_vel = -0.5
            self.req.rot_vel = 0.0
            if self.state == 1:
                self.get_logger().info(f"{self.state}")
                self.cli.call_async(self.req)
                self.state += 1
        elif time.time() < (self.t0 + 9.0):
            self.req.x_vel = -0.5
            self.req.y_vel = 0.0
            self.req.rot_vel = 0.0
            if self.state == 2:
                self.get_logger().info(f"{self.state}")
                self.cli.call_async(self.req)
                self.state += 1
        elif time.time() < (self.t0 + 12.0):
            self.req.x_vel = 0.0
            self.req.y_vel = 0.5
            self.req.rot_vel = 0.0
            if self.state == 3:
                self.get_logger().info(f"{self.state}")
                self.cli.call_async(self.req)
                self.state += 1
        else:
            self.get_logger().info('Test finished')
            self.destroy_node()

    def send_request(self):
        self.req.a = int(sys.argv[1])
        self.req.b = int(sys.argv[2])
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    client = SetSpeedServiceTest()

    try:
        rclpy.spin(client)
    except Exception as e:
        traceback.print_exc()
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
