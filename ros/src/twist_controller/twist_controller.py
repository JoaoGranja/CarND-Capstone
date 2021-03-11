
GAS_DENSITY = 2.858
ONE_MPH = 0.44704

import rospy
from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter

class Controller(object):
    def __init__(self, vehicle_mass, wheel_radius, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle,
                decel_limit, ):
        
        self.vehicle_mass = vehicle_mass
        self.wheel_radius = wheel_radius
        self.decel_limit = decel_limit
        
        # low pass filter for velocity values
        tau = 0.5
        ts = 0.02
        self.lpf = LowPassFilter(tau, ts)
        
        # PID for velocity control
        kp = 1
        ki = 0
        kd = 0
        self.pid_min = 0
        self.pid = PID(kp, ki, kd, mn=self.pid_min, mx=1)
        
        # yaw controller for angle control
        self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)
        
        
        self.last_time = rospy.Time.now().to_sec()

    def control(self, cur_linear_vel, cur_angular_vel, target_linear_vel, target_angular_vel, dbw_enabled ):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        throttle = 0.
        brake = 0.
        steering = 0.
        
        if not dbw_enabled:
            self.pid.reset()
            return throttle, brake, steering
        
        ## control the yaw angle
        steering = self.yaw_controller.get_steering(target_linear_vel, target_angular_vel, cur_linear_vel)
        
        ## control the velocity
        
        #Apply low pass filter to current velocity error to minimize high value changes
        #cur_linear_vel = self.lpf(cur_linear_vel)
        
        current_time = rospy.Time.now().to_sec()
        step_time = current_time - self.last_time

        # calculate velocity error
        error_vel = target_linear_vel - cur_linear_vel
        val = self.pid.step(error_vel, step_time)

        # Update last_time
        self.last_time = current_time
        
        # Calculate throttle or brake values based on pid control value
        if val > self.pid_min :
            throttle = val
        else:
            acc = abs(max(error_vel, self.decel_limit))
            brake = acc * self.vehicle_mass * self.wheel_radius # Torque N*m
           
        
        return throttle, brake, steering
