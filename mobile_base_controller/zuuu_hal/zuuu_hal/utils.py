import math
import time


def angle_diff(a: float, b: float) -> float:
    """Returns the smallest distance between 2 angles
    """
    d = a - b
    d = ((d + math.pi) % (2 * math.pi)) - math.pi
    return d


def sign(x: float) -> int:
    """Returns 1 if x >= 0, -1 otherwise
    """
    if x >= 0:
        return 1
    else:
        return -1


class PID:
    def __init__(self, p: float = 1.0, i: float = 0.0, d: float = 0.0, max_command: float = 10.0,
                 max_i_contribution: float = 5.0):
        """PID implementation with anti windup.
        Keyword Arguments:
            P {float} -- Proportional gain (default: {1.0})
            I {float} -- Integral gain (default: {0.0})
            D {float} -- Differential gain (default: {0.0})
            max_command {float} -- The output command will be trimmed to +- max_command (default: {10})
            max_i_contribution {float} -- The integral contribution will be trimmed to
                +- max_i_contribution (default: {5})
        """
        self.p = p
        self.i = i
        self.d = d
        self.max_command = max_command
        self.max_i_contribution = max_i_contribution
        self.goal_value = 0
        self.current_value = 0

        self.prev_error = 0
        self.i_contribution = 0
        self.prev_t = time.time()

    def set_goal(self, goal_value: float) -> None:
        """Sets the goal state
        """
        self.goal_value = goal_value
        # Reseting the persistent data because the goal state changed
        self.reset()

    def reset(self) -> None:
        """Resets the integral portion, dt and the differential contribution
        """
        self.i_contribution = 0
        self.prev_t = time.time()
        self.prev_error = self.goal_value - self.current_value

    def limit(self, x: float, limit: float) -> float:
        """Limits x to stay in +-limit"""
        if x > abs(limit):
            return abs(limit)
        elif x < -abs(limit):
            return -abs(limit)
        else:
            return x

    def tick(self, current_value: float, is_angle: bool = False) -> None:
        """PID calculations. If is_angle is True, then the error will be calculated as the smallest angle between
        the goal and the current_value
        Arguments:
            current_value {float} -- Current state, usually the feedback value

        Returns:
            float -- The output command
        """
        self.current_value = current_value
        if is_angle:
            error = angle_diff(self.goal_value, self.current_value)
        else:
            error = self.goal_value - self.current_value
        t = time.time()
        dt = t - self.prev_t
        self.prev_t = t
        delta_error = error - self.prev_error
        self.prev_error = error

        p_contribution = self.p * error
        if dt != 0:
            d_contribution = self.d * delta_error / dt
        else:
            d_contribution = 0.0

        self.i_contribution = self.i_contribution + self.i * error
        self.i_contribution = self.limit(
            self.i_contribution, self.max_i_contribution)

        self.command = p_contribution + self.i_contribution + d_contribution
        self.command = self.limit(self.command, self.max_command)

        return self.command
