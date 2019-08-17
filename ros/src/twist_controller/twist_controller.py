import rospy
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
MIN_SPEED =  0.1
PID_PARAMS = {
	'Kp': 0.3,
	'Ki': 0.1, 
	'Kd': 0.0, 
	'Min_Throttle': 0.0, 
	'Max_Throttle':0.4
}
LPF_PARAMS = {
	'tau': 0.5, # 1/(2pi*tau) is the cutoff frequency
	'Sample_Time': 0.02 # Sample Time 
}
MAX_BRAKE=400 #Nm
class Controller(object):
    def __init__(self, 
    			vehicle_mass, 
                fuel_capacity,
                brake_deadband, 
                decel_limit,
                accel_limit, 
                wheel_radius,
                wheel_base,
                steer_ratio,
                max_lat_accel,
                max_steer_angle):
    	# Instantiate steering angle controller - YawController
    	self.yaw_controller = YawController(wheel_base, steer_ratio, MIN_SPEED, max_lat_accel, max_steer_angle)

    	# Instantiate throttle controller - PIDController
    	self.throttle_controller = PID(PID_PARAMS.get('Kp'), PID_PARAMS.get('Ki'), PID_PARAMS.get('Kd'), PID_PARAMS.get('Min_Throttle'), PID_PARAMS.get('Max_Throttle'))

    	# Instantiate a low pass filter for averaging the velocity inputs 
    	self.velocity_lowPassFilter = LowPassFilter(LPF_PARAMS.get('tau'), LPF_PARAMS.get('Sample_Time'))

    	self.vehicle_mass = vehicle_mass
    	self.fuel_capacity = fuel_capacity
    	self.brake_deadband = brake_deadband
    	self.decel_limit = decel_limit
    	self.accel_limit = accel_limit
    	self.wheel_radius = wheel_radius

    	self.last_time = rospy.get_time()
        self.last_velocity = None
        pass

    def update_lastknowngood_states(self, current_velocity, current_time): 
    	self.last_velocity = current_velocity
    	self.last_time = current_time

    def compute_throttle_params(self, linear_velocity, current_velocity, current_time):
    	velocity_error  = linear_velocity - current_velocity
    	if not self.last_time is None: 
    		sample_time = current_time - self.last_time
    	else:
    		sample_time = current_time
        rospy.loginfo("Sample time: {}".format(sample_time))
        rospy.loginfo("Velocity error: {}".format(velocity_error))
    	throttle = self.throttle_controller.step(velocity_error, sample_time)
        rospy.loginfo("Updated throttle: {}".format(throttle))
    	
    	brake = 0

    	if linear_velocity == 0.0 and current_velocity < MIN_SPEED:
    		throttle = 0
    		brake = MAX_BRAKE # Nm- to hold the car in place if we are stopped at light, accel = 1 m/s^2
    	elif throttle < MIN_SPEED and velocity_error < 0: 
    		throttle = 0
    		decel = max(velocity_error, self.decel_limit)
    		brake = min(MAX_BRAKE, abs(decel) * self.vehicle_mass * self.wheel_radius) # torque Nm
    	return throttle, brake 

    def control(self, current_velocity, current_angular_velocity, dbw_enabled, linear_velocity, angular_velocity):
        if not dbw_enabled:
        	self.throttle_controller.reset()
        	return 0., 0., 0.

        # Smoothen the velocity input using low pass filter
        current_velocity = self.velocity_lowPassFilter.filter(current_velocity); 
        
        # Use yaw controller to get the desired steering angle given the linear and angular velocity 
        # from the car simulator (via dbw node) and the current step's velocity
        steering = self.yaw_controller.get_steering(linear_velocity, angular_velocity, current_velocity, current_angular_velocity)
        rospy.loginfo("Updated steering: {}".format(steering))


        current_time = rospy.get_time()
        
        # Use throttle controller to get the updated acceleration 
        throttle, brake = self.compute_throttle_params(linear_velocity, current_velocity, current_time)
        #rospy.loginfo("Updated throttle: {}".format(throttle))
        rospy.loginfo("Updated brake: {}".format(brake))

        self.update_lastknowngood_states(current_velocity, current_time)
        return throttle, brake, steering 
