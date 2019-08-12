from math import atan
import math

MIN_SPEED = 0.1
VELOCITY_ERROR_MIN_THRESHOLD = 0.0001
VELOCITY_ERROR_MAX_THRESHOLD = (math.pi * 0.5)
class YawController(object):
    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle):
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.min_speed = min_speed
        self.max_lat_accel = max_lat_accel

        self.min_angle = -max_steer_angle
        self.max_angle = max_steer_angle
        self.last_steering_angle = None
        pass

    def get_angle(self, radius):
        angle = atan(self.wheel_base / radius) * self.steer_ratio
        return max(self.min_angle, min(self.max_angle, angle))

    def get_steering(self, linear_velocity, angular_velocity, current_velocity, current_angular_velocity):
        angular_velocity = current_velocity * angular_velocity / linear_velocity if abs(linear_velocity) > 0. else 0.

        if abs(current_velocity) > MIN_SPEED:
            max_yaw_rate = abs(self.max_lat_accel / current_velocity);
            angular_velocity = max(-max_yaw_rate, min(max_yaw_rate, angular_velocity))

        angular_velocity_error = angular_velocity - current_angular_velocity
        if abs(angular_velocity_error) < VELOCITY_ERROR_MIN_THRESHOLD: 
            steering_angle = self.last_steering_angle
        else: 
            if abs(angular_velocity_error) > VELOCITY_ERROR_MAX_THRESHOLD:
                angular_velocity = angular_velocity - (angular_velocity_error * 0.5) # add dampening factoring by reducing the angular_velocity to make it closer to the current angular velocity
            steering_angle = self.get_angle(max(current_velocity, self.min_speed) / angular_velocity) if abs(angular_velocity) > 0. else 0.0;
            self.last_steering_angle = steering_angle
        return steering_angle
