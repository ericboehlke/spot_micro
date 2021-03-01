import rclpy
from rclpy.node import Node
from adafruit_servokit import ServoKit

from spot_interfaces.msg import Joints

class Motor:
    MIN = 0
    MAX = 180
    FORWARD = 1
    REVERSE = -1

    def __init__(self, channel, direction, origin):
        self.channel = channel
        self.direction = direction
        self.origin = origin
        self._current_angle = 0

        self.set_angle(self._current_angle)

    def _translate(self, angle):
        return (self.direction * angle) + self.origin

    def _translate_reverse(self, channel_angle):
        return self.direction * (channel_angle - self.origin)

    def set_angle(self, angle):
        raw_angle = min(max(Motor.MIN, self._translate(angle)), Motor.MAX)
        self.channel.angle = raw_angle
        self._current_angle = self._translate_reverse(raw_angle)

    def adjust_angle(self, angle_delta):
        self._current_angle += angle_delta
        self.set_angle(self._current_angle)


class Leg:
    def __init__(self, hip, knee, ankle):
        self.hip = hip
        self.knee = knee
        self.ankle = ankle


class Dog:
    def __init__(self, front_right, front_left, back_right, back_left):
        self.front_right = front_right
        self.front_left = front_left
        self.back_right = back_right
        self.back_left = back_left


blh_direction = Motor.FORWARD   
blk_direction = Motor.REVERSE
bla_direction = Motor.FORWARD
flh_direction = Motor.REVERSE 
flk_direction = Motor.REVERSE
fla_direction = Motor.FORWARD
brh_direction = Motor.REVERSE 
brk_direction = Motor.FORWARD
bra_direction = Motor.REVERSE
frh_direction = Motor.FORWARD
frk_direction = Motor.FORWARD
fra_direction = Motor.REVERSE

blh_origin = 90  
blk_origin = 140
bla_origin = 0
flh_origin = 90 
flk_origin = 140
fla_origin = 0
brh_origin = 90 
brk_origin = 35
bra_origin = 180
frh_origin = 90
frk_origin = 45
fra_origin = 178

class JointsNode(Node):

    def __init__(self):
        super().__init__('joints_node')

        kit = ServoKit(channels=16)

        # back left leg (orange)
        blh = kit.servo[0]
        blk = kit.servo[1]
        bla = kit.servo[2]

        # front left leg (blue)
        flh = kit.servo[4]
        flk = kit.servo[5]
        fla = kit.servo[6]

        # back right leg (purple)
        brh = kit.servo[8]
        brk = kit.servo[9]
        bra = kit.servo[10]

        # front right leg (pink)
        frh = kit.servo[12]
        frk = kit.servo[13]
        fra = kit.servo[14]

        self.spot = Dog(Leg(Motor(frh, frh_direction, frh_origin),
                            Motor(frk, frk_direction, frk_origin),
                            Motor(fra, fra_direction, fra_origin)),
                        Leg(Motor(flh, flh_direction, flh_origin),
                            Motor(flk, flk_direction, flk_origin),
                            Motor(fla, fla_direction, fla_origin)),
                        Leg(Motor(brh, brh_direction, brh_origin),
                            Motor(brk, brk_direction, brk_origin),
                            Motor(bra, bra_direction, bra_origin)),
                        Leg(Motor(blh, blh_direction, blh_origin),
                            Motor(blk, blk_direction, blk_origin),
                            Motor(bla, bla_direction, bla_origin)))

        self.subscription = self.create_subscription(
            Joints,
            'joints',
            self.joints_callback,
            10)
        self.subscription  # prevent unused variable warning

    def joints_callback(self, msg):
        self.spot.front_right.hip = msg.front_right_hip
        self.spot.front_right.knee = msg.front_right_knee
        self.spot.front_right.ankle = msg.front_right_ankle
        self.spot.front_left.hip = msg.front_left_hip
        self.spot.front_left.knee = msg.front_left_knee
        self.spot.front_left.ankle = msg.front_left_ankle
        self.spot.back_right.hip = msg.back_right_hip
        self.spot.back_right.knee = msg.back_right_knee
        self.spot.back_right.ankle = msg.back_right_ankle
        self.spot.back_left.hip = msg.back_left_hip
        self.spot.back_left.knee = msg.back_left_knee
        self.spot.back_left.ankle = msg.back_left_ankle

        self.get_logger().info('Updated joints')


def main(args=None):
    rclpy.init(args=args)

    joint_node = JointsNode()

    rclpy.spin(joint_node)

    joint_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
