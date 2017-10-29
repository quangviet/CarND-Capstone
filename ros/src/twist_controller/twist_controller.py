from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

class Controller(object):
    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle):
        self.wheel_base = wheel_base
	self.steer_ratio = steer_ratio
	self.min_speed = min_speed
	self.max_lat_accel = max_lat_accel
	self.max_steer_angle = max_steer_angle

	self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)


    def control(self, proposed_linear_velocity, proposed_angular_velocity, current_linear_velocity, dbw_state):
	steer = self.yaw_controller.get_steering(proposed_linear_velocity, proposed_angular_velocity, current_linear_velocity)
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        return 1., 0., steer
