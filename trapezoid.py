import math


class MotionState:
    def __init__(self, pose, vel: float, accel: float):
        self.pose = pose
        self.vel = vel
        self.accel = accel


class Profile:
    def __init__(self, initial_state: MotionState, final_state: MotionState):
        self.initial_state = initial_state
        self.final_state = final_state
        self.pose_delta = final_state.pose - initial_state.pose


class Trapezoid(Profile):
    def calculate_distance(self, max_vel, max_accel, elapsed_time, velocity):
        init_pose = self.initial_state.pose
        d = self.pose_delta
        init_vel = self.initial_state.vel
        final_vel = self.final_state.vel

        # the farthest position to reach before deceleration is needed
        decel_point = d / 2 + (final_vel ** 2 - init_vel ** 2) / (4 * max_accel)

        accel_t = max_vel / max_accel - init_vel / max_accel
        accel_d = init_vel * accel_t + 0.5 * max_accel * accel_t ** 2

        if accel_d > decel_point:
            print('yes')
            accel_d = decel_point
            # watch out for edge case where init_vel > max_vel
            max_vel = math.sqrt(init_vel ** 2 + 2 * max_accel * decel_point)
            accel_t = max_vel / max_accel - init_vel / max_accel


        decel_t = max_vel / max_accel - final_vel / max_accel
        decel_d = final_vel * decel_t + 0.5 * max_accel * decel_t ** 2

        cruise_d = d - accel_d - decel_d
        cruise_t = cruise_d / max_vel

        t_sum = accel_t + cruise_t + decel_t

        decel_start = accel_t + cruise_t


        if elapsed_time > t_sum:
            if velocity:
                return final_vel
            return init_pose + d

        if elapsed_time < accel_t:
            if velocity:
                return init_vel + max_accel * elapsed_time
            return init_pose + init_vel * elapsed_time + 0.5 * max_accel * elapsed_time ** 2
        elif elapsed_time < decel_start:
            if velocity:
                return init_vel + max_accel * accel_t
            current_cruise_t = elapsed_time - accel_t
            return init_pose + accel_d + max_vel * current_cruise_t
        else:
            current_decel_t = elapsed_time - decel_start
            if velocity:
                return init_vel + max_accel * accel_t - max_accel * current_decel_t
            return (
                    init_pose
                    + accel_d
                    + cruise_d
                    + max_vel * current_decel_t
                    - 0.5 * max_accel * current_decel_t ** 2
            )
