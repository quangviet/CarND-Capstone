#import rospy
from pid import PID
from yaw_controller import YawController
from dbw_mkz_msgs.msg import BrakeCmd

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
MIN_THROTTLE = 0.0
MAX_THROTTLE = 1.0
MIN_BRAKE = 0.0
MAX_BRAKE = BrakeCmd.TORQUE_MAX # 3412
MIN_SPEED = 10.0
SAMPLE_TIME = 0.02

class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband,
                  decel_limit, accel_limit, wheel_radius, wheel_base,
                  steer_ratio, max_lat_accel, max_steer_angle):
        # TODO: Implement
        self.pid_th = PID(1.8, 0.001, 0.01, MIN_THROTTLE, MAX_THROTTLE)
        self.pid_th.reset()
        self.pid_br = PID(200, 0.0, 0.0, MIN_BRAKE, MAX_BRAKE)
        self.pid_br.reset()
        self.yawController = YawController(wheel_base, steer_ratio,
                              MIN_SPEED, max_lat_accel, max_steer_angle)

    def control(self, linear_velocity, angular_velocity, current_velocity):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        steer = self.yawController.get_steering(linear_velocity,
                                                angular_velocity,
                                                current_velocity)
        #rospy.logerr('steer %f', steer)
        if (linear_velocity < current_velocity):
            #rospy.logerr('linear_velocity %f : current_velocity %f', linear_velocity, current_velocity)
            throttle = 0.0
            brake = self.pid_br.step(current_velocity-linear_velocity,
                                    SAMPLE_TIME)
        else:
            linear_velocity *= (1.0 - abs(steer)) # decrease velocity in curve road
            throttle = self.pid_th.step(linear_velocity-current_velocity,
                                    SAMPLE_TIME)
            brake = 0.0
        return throttle, brake, steer

