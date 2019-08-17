from math import atan
import math

MIN_SPEED = 0.1
VELOCITY_ERROR_MIN_THRESHOLD = 0.00001
VELOCITY_ERROR_MAX_THRESHOLD = math.pi/24
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
        print("1.angular_velocity=", angular_velocity)
        print("2.current_angular_velocity=", current_angular_velocity)
        angular_velocity = current_velocity * angular_velocity / linear_velocity if abs(linear_velocity) > 0. else 0.
        if abs(current_velocity) > MIN_SPEED:
            max_yaw_rate = abs(self.max_lat_accel / current_velocity);
            angular_velocity = max(-max_yaw_rate, min(max_yaw_rate, angular_velocity))

       
        angular_velocity_error = angular_velocity - current_angular_velocity
        print("3.angular_velocity=", angular_velocity)
        print("4.current_angular_velocity=", current_angular_velocity)
        print("5.angular_velocity_error=", angular_velocity_error)
        steering_angle = 0
        if abs(angular_velocity_error) < VELOCITY_ERROR_MIN_THRESHOLD: 
            if not self.last_steering_angle is None:
                steering_angle = self.last_steering_angle
                print("6.steering_angle=", steering_angle)
            else:
                self.last_steering_angle = steering_angle    
                print("7.steering_angle=", steering_angle)
        else: 
            if abs(angular_velocity_error) > VELOCITY_ERROR_MAX_THRESHOLD:
                angular_velocity = angular_velocity - (angular_velocity_error * 0.8) # add dampening factoring by reducing the angular_velocity to make it closer to the current angular velocity
                print("8.angular_velocity=", angular_velocity)
            print("8a.current_velocity=", current_velocity)
            print("8b.self.min_speed=", self.min_speed)
            print("8c.angular_velocity=", angular_velocity)
            steering_angle = self.get_angle(max(current_velocity, self.min_speed) / angular_velocity) if abs(angular_velocity) > 0. else 0.0;
            print("9.steering_angle=", steering_angle)
            self.last_steering_angle = steering_angle
        
        #steering_angle = self.get_angle(max(current_velocity, self.min_speed) / angular_velocity) if abs(angular_velocity) > 0. else 0.0;
        #sself.last_steering_angle = steering_angle
    
        return steering_angle