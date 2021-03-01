import rclpy
from rclpy.node import Node

from spot_interfaces.msg import Joints


class JointPublisher(Node):

    def __init__(self):
        super().__init__('joint_publisher')
        self.publisher_ = self.create_publisher(Joints, 'joints', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        joints = Joints()
        joints.front_right_hip = 0.0
        joints.front_right_knee = 0.0
        joints.front_right_ankle = 0.0
        joints.front_left_hip = 0.0
        joints.front_left_knee = 0.0
        joints.front_left_ankle = 0.0
        joints.back_right_hip = 0.0
        joints.back_right_knee = 0.0
        joints.back_right_ankle = 0.0
        joints.back_left_hip = 0.0
        joints.back_left_knee = 0.0
        joints.back_left_ankle = 0.0

        self.publisher_.publish(joints)
        self.get_logger().info('Publishing joints')


def main(args=None):
    rclpy.init(args=args)

    joint_publisher = JointPublisher()

    rclpy.spin(joint_publisher)

    joint_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
