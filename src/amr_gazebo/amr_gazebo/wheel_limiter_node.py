"""Wheel-space limiter for simulated velocity commands."""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from amr_gazebo.wheel_limiter import WheelLimiter


class WheelLimiterNode(Node):
    def __init__(self):
        super().__init__('wheel_limiter')
        self.limiter = WheelLimiter()
        self.publisher = self.create_publisher(Twist, '/cmd_vel_limited', 10)
        self.debug_publisher = self.create_publisher(TwistStamped, '/wheel_limiter/command_stamped', 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_command, 10)
        self.create_timer(.02, self.tick)

    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_command(self, msg):
        try:
            self.limiter.command(msg.linear.x, msg.angular.z, self.now_seconds())
        except ValueError:
            self.get_logger().warning('Ignoring non-finite command')

    def tick(self):
        now = self.get_clock().now()
        v, w = self.limiter.step(now.nanoseconds * 1e-9)
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.publisher.publish(msg)
        stamped = TwistStamped()
        stamped.header.stamp = now.to_msg()
        stamped.header.frame_id = 'base_footprint'
        stamped.twist = msg
        self.debug_publisher.publish(stamped)


def main(args=None):
    rclpy.init(args=args)
    node = WheelLimiterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
