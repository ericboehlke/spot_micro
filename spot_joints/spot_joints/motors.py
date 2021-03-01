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

