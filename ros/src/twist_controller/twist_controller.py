
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
        self.min_speed = min_speed
        
        # low pass filter for velocity values
        tau = 0.2
        ts = 0.02
        self.lpf_curr_vel = LowPassFilter(tau, ts)
        self.lpf_target_vel = LowPassFilter(tau, ts)
        
        # PID for throttle
        kp = 1
        ki = 0.3
        kd = 0
        self.throttle_min = 0
        self.throttle_pid = PID(kp, ki, kd, mn=self.throttle_min, mx=1)
        
        # yaw controller for angle control
        rospy.logdebug("Twist Controller - wheel_base %f, steer_ratio %f, min_speed %f, max_lat_accel %f, max_steer_angle %f", 
                      wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)
        self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)
        
 
        self.last_time = rospy.get_time()

    def control(self, cur_linear_vel, cur_angular_vel, target_linear_vel, target_angular_vel, dbw_enabled ):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        throttle = 0.
        brake = 0.
        steering = 0.
        
        rospy.logdebug("Twist Controller - cur_linear_vel {},  cur_angular_vel {}, target_linear_vel {} and target_angular_vel {} ".format(cur_linear_vel, cur_angular_vel, target_linear_vel, target_angular_vel))
        
        if not dbw_enabled:
            self.throttle_pid.reset()
            rospy.logdebug("Twist Controller - Manual Mode")
            return throttle, brake, steering
        
        rospy.logdebug("Twist Controller - Automatic Mode")
        
        # Calculate step time
        current_time = rospy.get_time()
        step_time = current_time - self.last_time
        
        ## --------------- control the yaw angle --------------------------- ##
        steering = self.yaw_controller.get_steering(target_linear_vel, target_angular_vel, cur_linear_vel)
        
        #rospy.logwarn("Twist Controller - steering {}".format(steering))

        ## --------------- control the Throttle/Brake --------------------------- ##
        #Apply low pass filter to current velocity error to minimize high value changes
        cur_linear_vel = self.lpf_curr_vel.filt(cur_linear_vel)
        target_linear_vel = self.lpf_target_vel.filt(target_linear_vel)
    
        # calculate velocity error
        error_vel = target_linear_vel - cur_linear_vel
        throttle = self.throttle_pid.step(error_vel, step_time)
        
        
        # Calculate throttle or brake values based on pid control value
        if cur_linear_vel < self.min_speed and target_linear_vel < self.min_speed:
            throttle = 0
            brake = 700
        elif throttle < 0.1 and target_linear_vel < cur_linear_vel:
            throttle = 0
            decel = abs(max(error_vel, self.decel_limit))
            brake = decel * self.vehicle_mass * self.wheel_radius # Torque N*m
         
        # Update last_time
        self.last_time = current_time
        
        return throttle, brake, steering
