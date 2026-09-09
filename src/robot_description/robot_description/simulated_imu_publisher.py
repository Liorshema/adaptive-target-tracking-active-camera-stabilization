import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class SimulatedImuPublisher(Node):

    def __init__(self):
        super().__init__('simulated_imu_publisher')

        self.publisher_ = self.create_publisher(
            Imu,
            '/simulated_imu',
            10
        )

        self.timer = self.create_timer(
            0.1,
            self.publish_imu
        )

    def publish_imu(self):
        msg = Imu()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # Robot initially assumed stationary and level
        msg.orientation.w = 1.0

        msg.angular_velocity.x = 0.0
        msg.angular_velocity.y = 0.0
        msg.angular_velocity.z = 0.0

        msg.linear_acceleration.x = 0.0
        msg.linear_acceleration.y = 0.0
        msg.linear_acceleration.z = 9.81

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = SimulatedImuPublisher()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()