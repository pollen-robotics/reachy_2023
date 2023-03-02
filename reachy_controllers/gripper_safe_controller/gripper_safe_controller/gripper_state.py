from collections import deque
from enum import Enum

import numpy as np

UPDATE_FREQ = 100  # Hz
DT = 1 / UPDATE_FREQ

MAX_TORQUE = 0.5
P_SAFE_CLOSE = 3.0
P_DIRECT_CONTROL = 5.0

HISTORY_LENGTH = 10
SKIP_EARLY_DTS = 15
MIN_MOVING_DIST = 0.004
MAX_ERROR = np.deg2rad(10.0)
MOVING_SPEED = np.deg2rad(110)
INC_PER_DT = DT * MOVING_SPEED

MX28_A_GAIN = 3.0
MX28_B_GAIN = 0.05


class CollisionState(Enum):
    NO_COLLISION = 0
    ENTERING_COLLISION = 1
    STILL_COLLIDING = 2
    LEAVING_COLLISION = 3


class GripperState:
    """Represent the current gripper state."""
    def __init__(
        self,
        name: str,
        is_direct: bool,
        present_position: float, user_requested_goal_position: float,
        p: float = P_DIRECT_CONTROL, i: float = 0.0, d: float = 0.0,
    ) -> None:

        self.name = name
        self.is_direct = is_direct

        self.present_position = deque([present_position], HISTORY_LENGTH)
        self.user_requested_goal_position = deque([user_requested_goal_position], HISTORY_LENGTH)

        self.interpolated_goal_position = deque([user_requested_goal_position], HISTORY_LENGTH)
        self.error = deque([], HISTORY_LENGTH//2)
        self.in_collision = deque([False], HISTORY_LENGTH)

        self.safe_computed_goal_position = user_requested_goal_position

        self.elapsed_dts_since_change_of_direction = 0
        self.elapsed_dts_since_collision = 0

        self.pid = p, i, d

    def update(self, new_present_position: float, new_user_requested_goal_position: float):
        self.present_position.append(new_present_position)

        if self.has_changed_direction(new_user_requested_goal_position):
            self.elapsed_dts_since_change_of_direction = 0

        self.user_requested_goal_position.append(new_user_requested_goal_position)

        collision_state = self.check_collision_state()

        if collision_state == CollisionState.NO_COLLISION:
            interpolated_goal_position = self.compute_close_smart_goal_position()
            self.safe_computed_goal_position = new_user_requested_goal_position

        elif collision_state == CollisionState.ENTERING_COLLISION:
            self.set_pid(p=P_SAFE_CLOSE, i=0.0, d=0.0)
            interpolated_goal_position = self.compute_fake_error_goal_position()
            self.safe_computed_goal_position = interpolated_goal_position

        elif collision_state == CollisionState.STILL_COLLIDING:
            interpolated_goal_position = self.compute_fake_error_goal_position()
            self.safe_computed_goal_position = interpolated_goal_position

        elif collision_state == CollisionState.LEAVING_COLLISION:
            self.set_pid(p=P_DIRECT_CONTROL, i=0.0, d=0.0)
            interpolated_goal_position = self.compute_close_smart_goal_position()
            self.safe_computed_goal_position = new_user_requested_goal_position

        self.interpolated_goal_position.append(interpolated_goal_position)
        self.error.append(interpolated_goal_position - new_present_position)

        self.in_collision.append(collision_state in (CollisionState.ENTERING_COLLISION, CollisionState.STILL_COLLIDING))

        self.elapsed_dts_since_change_of_direction += 1
        self.elapsed_dts_since_collision += 1

    def check_collision_state(self) -> CollisionState:
        if not hasattr(self, '_hidden_collision_state'):
            self._hidden_collision_state = CollisionState.NO_COLLISION

        if not self.in_collision[-1] and self.entering_collision():
            self._hidden_collision_state = CollisionState.STILL_COLLIDING
            return CollisionState.ENTERING_COLLISION

        if self.in_collision[-1] and self.leaving_collision():
            self._hidden_collision_state = CollisionState.NO_COLLISION
            self.elapsed_dts_since_collision = 0
            return CollisionState.LEAVING_COLLISION

        return self._hidden_collision_state

    def entering_collision(self) -> bool:
        if self.elapsed_dts_since_change_of_direction <= SKIP_EARLY_DTS:
            return False

        filtered_error = np.mean(self.error)
        return (
            (filtered_error > MAX_ERROR and self.user_requested_goal_position[-1] > self.present_position[-1])
            if self.is_direct
            else (filtered_error < -MAX_ERROR and self.user_requested_goal_position[-1] < self.present_position[-1])
        )

    def leaving_collision(self) -> bool:
        last_req_goal_pos = self.user_requested_goal_position[-1]
        last_interp_goal_pos = self.interpolated_goal_position[-1]

        # This condition is triggered by a direct user command to release our grasp
        user_request_to_release = (
            last_req_goal_pos < last_interp_goal_pos
            if self.is_direct else
            last_req_goal_pos > last_interp_goal_pos
        )

        # This condition is triggered because we are moving again, due to either:
        #   - because the object was removed
        #   - because it was a false detection in the first place
        moving_again = (
            self.elapsed_dts_since_collision > self.present_position.maxlen and
            ((self.present_position[0] - self.present_position[-1]) < -MIN_MOVING_DIST
                if self.is_direct
                else (self.present_position[0] - self.present_position[-1]) > MIN_MOVING_DIST)
        )

        return user_request_to_release or moving_again

    def has_changed_direction(self, new_goal_pos: float) -> bool:
        present_position = self.present_position[-1]
        last_goal_pos = self.user_requested_goal_position[-1]

        return np.sign(last_goal_pos - present_position) != np.sign(new_goal_pos - present_position)

    def compute_close_smart_goal_position(self) -> float:
        last_req_goal_pos = self.user_requested_goal_position[-1]
        goal_offset = INC_PER_DT * np.sign(last_req_goal_pos - self.present_position[-1])

        next_goal_pos = self.interpolated_goal_position[-1] + goal_offset

        return (
            min(next_goal_pos, last_req_goal_pos)
            if goal_offset > 0 else
            max(next_goal_pos, last_req_goal_pos)
        )
        
    def compute_fake_error_goal_position(self) -> float:
        model_offset = np.deg2rad(MX28_A_GAIN * MAX_TORQUE + MX28_B_GAIN / P_SAFE_CLOSE)
        if not self.is_direct:
            model_offset = -model_offset

        return model_offset + self.present_position[-1]

    def set_pid(self, p, i, d) -> None:
        self.pid = p, i, d
