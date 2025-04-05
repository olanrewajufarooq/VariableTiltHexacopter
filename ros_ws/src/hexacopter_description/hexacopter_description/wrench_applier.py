import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from gz_msgs.srv import ApplyLinkWrench
from builtin_interfaces.msg import Time

class WrenchArrayApplier(Node):
    def __init__(self):
        super().__init__('wrench_array_applier')

        self.link_names = [f'prop_{i+1}' for i in range(6)]

        # Setup service clients per propeller
        self.clients = {
            name: self.create_client(
                ApplyLinkWrench,
                f'/world/empty/model/variable_tilt_hexacopter/link/{name}/apply_wrench'
            ) for name in self.link_names
        }

        # Wait for services
        for name, client in self.clients.items():
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for service for {name}...')

        # Create subscriptions to 6 topics
        for i in range(6):
            topic = f'/hexacopter/prop_{i+1}/cmd_wrench'
            self.create_subscription(
                Wrench,
                topic,
                lambda msg, i=i: self.apply_wrench(i, msg),
                10
            )
            self.get_logger().info(f'Subscribed to: {topic}')

    def apply_wrench(self, i, wrench: Wrench):
        now = self.get_clock().now().to_msg()
        request = ApplyLinkWrench.Request()
        link = self.link_names[i]
        request.link_name = link
        request.force = wrench.force
        request.torque = wrench.torque
        request.start_time = Time(sec=now.sec, nanosec=now.nanosec)
        request.duration.nanosec = 100_000_000

        self.clients[link].call_async(request)
        self.get_logger().info(f'Wrench applied to {link} | F: {wrench.force}, T: {wrench.torque}')

def main(args=None):
    rclpy.init(args=args)
    node = WrenchArrayApplier()
    rclpy.spin(node)
    rclpy.shutdown()
