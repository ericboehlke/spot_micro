import rclpy
from rclpy.node import Node

import numpy as np
from time import sleep
from spot_interfaces.msg import Joints


class JointKeyboard(Node):

    def __init__(self):
        super().__init__('joint_keyboard_controller')
        self.publisher_ = self.create_publisher(Joints, 'joints', 10)
        self.joint_angles = [0.0] * 12
        self.get_logger().info('starting joint_keyboard_controller node')

    def make_msg(self, angle_list):
        joints = Joints()
        joints.front_right_hip = angle_list[0]
        joints.front_right_knee = angle_list[1]
        joints.front_right_ankle = angle_list[2]
        joints.front_left_hip = angle_list[3]
        joints.front_left_knee = angle_list[4]
        joints.front_left_ankle = angle_list[5]
        joints.back_right_hip = angle_list[6]
        joints.back_right_knee = angle_list[7]
        joints.back_right_ankle = angle_list[8]
        joints.back_left_hip = angle_list[9]
        joints.back_left_knee = angle_list[10]
        joints.back_left_ankle = angle_list[11]
        return joints


    def stand(self, height):
        timestep = 0.05
        speed = 10  # units per second
        goal_angles = np.array([0, height/4, 3*height/4] * 4)
        self.joint_angles = np.array(self.joint_angles)
        while (np.abs(goal_angles - self.joint_angles) > np.ones(12)*.0001).any():
            diff_angles = np.maximum(np.minimum(goal_angles - self.joint_angles, np.ones(12) * speed * timestep), -np.ones(12) * speed * timestep)
            self.joint_angles += diff_angles
            self.publisher_.publish(self.make_msg(self.joint_angles))
            sleep(timestep)

    def shell(self, string):
        args = string.split()
        if args[0].lower() == "lay":
            self.get_logger().info('laying down')
            self.stand(0)
        elif args[0].lower() == "stand":
            self.get_logger().info('standing up')
            try:
                self.stand(float(args[1]))
            except:
                self.stand(50)
        else:
            self.get_logger().info('input error')



def main(args=None):
    rclpy.init(args=args)

    joint_keyboard = JointKeyboard()

    while True:
        joint_keyboard.shell(input("> "))

    joint_keyboard.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
