#!/usr/bin/env python

"""
    A base controller class for the Arduino microcontroller
    
    Borrowed heavily from Mike Feguson's ArbotiX base_controller.py code.
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2010 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses
"""

import rclpy
from rclpy.node import Node
import sys, os

from ros_arduino_python.miscellaneous import declare_params
from math import sin, cos, pi
from geometry_msgs.msg import Quaternion, Twist, Pose
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from ros_arduino_python.diagnostics import DiagnosticsUpdater

 
""" Class to receive Twist commands and publish Odometry data """
class BaseController(Node):
    def __init__(self, arduino, base_frame, name='base_controller'):
        super().__init__(name)
        print("base_controller node init, name:", name)
        declare_params(self)

        self.arduino = arduino
        self.name = name
        self.base_frame = base_frame

        self.rate = float(self.get_parameter("base_controller_rate").get_parameter_value().integer_value)
        self.timeout = self.get_parameter("base_controller_timeout").get_parameter_value().double_value
        self.odom_linear_scale_correction = self.get_parameter("odom_linear_scale_correction").get_parameter_value().double_value
        self.odom_angular_scale_correction = self.get_parameter("odom_angular_scale_correction").get_parameter_value().double_value
        self.use_imu_heading = self.get_parameter("use_imu_heading").get_parameter_value().bool_value
        self.publish_odom_base_transform = self.get_parameter("publish_odom_base_transform").get_parameter_value().bool_value

        print("rate:", self.rate)
        print("timeout:", self.timeout)
        print("odom_linear_scale_correction:", self.odom_linear_scale_correction)
        print("odom_angular_scale_correction:", self.odom_angular_scale_correction)
        print("use_imu_heading:", self.use_imu_heading)
        print("publish_odom_base_transform:", self.publish_odom_base_transform)

        self.stopped = False
        self.current_speed = Twist()
                 
        pid_params = dict()
        pid_params['wheel_diameter'] = self.get_parameter("wheel_diameter").get_parameter_value().double_value
        pid_params['wheel_track'] = self.get_parameter("wheel_track").get_parameter_value().double_value
        pid_params['encoder_resolution'] = self.get_parameter("encoder_resolution").get_parameter_value().integer_value
        pid_params['gear_reduction'] = self.get_parameter("gear_reduction").get_parameter_value().double_value
        pid_params['Kp'] = self.get_parameter("Kp").get_parameter_value().integer_value
        pid_params['Kd'] = self.get_parameter("Kd").get_parameter_value().integer_value
        pid_params['Ki'] = self.get_parameter("Ki").get_parameter_value().integer_value
        pid_params['Ko'] = self.get_parameter("Ko").get_parameter_value().integer_value
        print("pid_params:", pid_params)

        self.accel_limit = self.get_parameter('accel_limit').get_parameter_value().double_value
        self.motors_reversed = self.get_parameter("motors_reversed").get_parameter_value().bool_value
        self.detect_enc_jump_error = self.get_parameter("detect_enc_jump_error").get_parameter_value().bool_value
        self.enc_jump_error_threshold = self.get_parameter("enc_jump_error_threshold").get_parameter_value().integer_value

        # Default error threshold (percent) before getting a diagnostics warning
        self.base_diagnotics_error_threshold = self.get_parameter("base_diagnotics_error_threshold").get_parameter_value().integer_value

        # Diagnostics update rate
        self.base_diagnotics_rate = self.get_parameter("base_diagnotics_rate").get_parameter_value().double_value
        print("accel_limit:", self.accel_limit)
        print("motors_reversed:", self.motors_reversed)
        print("detect_enc_jump_error:", self.detect_enc_jump_error)
        print("enc_jump_error_threshold:", self.enc_jump_error_threshold)
        print("base_diagnotics_error_threshold:", self.base_diagnotics_error_threshold)
        print("base_diagnotics_rate:", self.base_diagnotics_rate)

        # Create the diagnostics updater for the Arduino device
        self.diagnostics = DiagnosticsUpdater(self, 
                                              self.name,
                                              self.base_diagnotics_error_threshold, 
                                              self.base_diagnotics_rate)

        # Set up PID parameters and check for missing values
        self.setup_pid(pid_params)

        # How many encoder ticks are there per meter?
        self.ticks_per_meter = self.encoder_resolution * self.gear_reduction  / (self.wheel_diameter * pi)

        # What is the maximum acceleration we will tolerate when changing wheel speeds?
        self.max_accel = self.accel_limit * self.ticks_per_meter / self.rate

        # Track how often we get a bad encoder count (if any)
        self.bad_encoder_count = 0

        now = self.get_clock().now()
        self.then = now # time for determining dx/dy
        self.t_delta = rclpy.duration.Duration(seconds=1.0 / self.rate)
        self.t_next = now + self.t_delta
        print("now:", now)
        print("t_delta:", self.t_delta)
        print("t_next:", self.t_next)

        # Internal data        
        self.enc_left = None            # encoder readings
        self.enc_right = None
        self.x = 0.0                      # position in xy plane
        self.y = 0.0
        self.th = 0.0                     # rotation in radians
        self.v_left = 0.0
        self.v_right = 0.0
        self.v_des_left = 0.0             # cmd_vel setpoint
        self.v_des_right = 0.0
        self.last_cmd_vel = now

        # Subscriptions
        self.create_subscription(Twist, "cmd_vel", self.cmdVelCallback, 10)

        # Clear any old odometry info
        self.arduino.reset_encoders()

        # Set up the odometry broadcaster
        self.odomPub = self.create_publisher(Odometry, 'odom', 5)
        self.odomBroadcaster = TransformBroadcaster(self)

        self.get_logger().info("Started base controller for a base of " + str(self.wheel_track) + "m wide with " + str(self.encoder_resolution) + " ticks per rev")
        self.get_logger().info("Publishing odometry data at: " + str(self.rate) + " Hz using " + str(self.base_frame) + " as base frame")
        
    def setup_pid(self, pid_params):
        # Check to see if any PID parameters are missing
        missing_params = False
        for param in pid_params:
            if pid_params[param] is None or pid_params[param] == "":
                print("*** PID Parameter " + param + " is missing. ***")
                missing_params = True
        
        if missing_params:
            os._exit(1)
                
        self.wheel_diameter = pid_params['wheel_diameter']
        self.wheel_track = pid_params['wheel_track']
        self.encoder_resolution = pid_params['encoder_resolution']
        self.gear_reduction = pid_params['gear_reduction']
        
        self.Kp = pid_params['Kp']
        self.Kd = pid_params['Kd']
        self.Ki = pid_params['Ki']
        self.Ko = pid_params['Ko']
        
        if self.arduino.update_pid(self.Kp, self.Kd, self.Ki, self.Ko):
            self.get_logger().info("PID parameters update to: Kp=%d, Kd=%d, Ki=%d, Ko=%d" %(self.Kp, self.Kd, self.Ki, self.Ko))
        else:
            self.get_logger().error("Updating PID parameters failed!")

    def poll(self):
        #now = rospy.Time.now()
        now = self.get_clock().now()
        if now > self.t_next:
            # Read the encoders
            try:
                self.diagnostics.reads += 1
                self.diagnostics.total_reads += 1
                left_enc, right_enc = self.arduino.get_encoder_counts()
                self.diagnostics.freq_diag.tick()
            except:
                self.diagnostics.errors += 1
                self.bad_encoder_count += 1
                self.get_logger().error("Encoder exception count: " + str(self.bad_encoder_count))
                return

            # Check for jumps in encoder readings
            if self.detect_enc_jump_error:
                try:
                    self.get_logger().info("Left: %d LEFT: %d Right: %d RIGHT: %d", left_enc, self.enc_left, right_enc, self.enc_right)
                    enc_jump_error = False
                    if abs(right_enc - self.enc_right) > self.enc_jump_error_threshold:
                        self.diagnostics.errors += 1
                        self.bad_encoder_count += 1
                        self.get_logger().error("RIGHT encoder jump error from %d to %d", self.enc_right, right_enc)
                        self.enc_right = right_enc
                        enc_jump_error = True

                    if abs(left_enc - self.enc_left) > self.enc_jump_error_threshold:
                        self.diagnostics.errors += 1
                        self.bad_encoder_count += 1
                        self.get_logger().error("LEFT encoder jump error from %d to %d", self.enc_left, left_enc)
                        self.enc_left = left_enc
                        enc_jump_error = True

                    if enc_jump_error:
                        return
                except:
                    pass

            dt = now - self.then
            self.then = now
            dt = dt.nanoseconds / 1e9  # to seconds
            #print("base controller dt:", dt)
            
            # Calculate odometry
            if self.enc_left == None:
                dright = 0
                dleft = 0
            else:
                dright = (right_enc - self.enc_right) / self.ticks_per_meter
                dleft = (left_enc - self.enc_left) / self.ticks_per_meter

            self.enc_right = right_enc
            self.enc_left = left_enc
            
            dxy_ave = self.odom_linear_scale_correction * (dright + dleft) / 2.0
            dth = self.odom_angular_scale_correction * (dright - dleft) / float(self.wheel_track)
            vxy = dxy_ave / dt
            vth = dth / dt
                
            if (dxy_ave != 0):
                dx = cos(dth) * dxy_ave
                dy = -sin(dth) * dxy_ave
                self.x += (cos(self.th) * dx - sin(self.th) * dy)
                self.y += (sin(self.th) * dx + cos(self.th) * dy)
    
            if (dth != 0):
                self.th += dth 
    
            quaternion = Quaternion()
            quaternion.x = 0.0 
            quaternion.y = 0.0
            quaternion.z = sin(self.th / 2.0)
            quaternion.w = cos(self.th / 2.0)

            transform_stamped = TransformStamped()
            transform_stamped.header.stamp = self.get_clock().now().to_msg()
            transform_stamped.header.frame_id = "odom"
            transform_stamped.child_frame_id = self.base_frame
            transform_stamped.transform.translation.x = self.x
            transform_stamped.transform.translation.y = self.y
            transform_stamped.transform.translation.z = 0.0
            transform_stamped.transform.rotation.x = quaternion.x
            transform_stamped.transform.rotation.y = quaternion.y
            transform_stamped.transform.rotation.z = quaternion.z
            transform_stamped.transform.rotation.w = quaternion.w
            # Create the odometry transform frame broadcaster.
            if self.publish_odom_base_transform:
                self.odomBroadcaster.sendTransform(
                        # (self.x, self.y, 0), 
                        # (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                        # self.get_clock().now(),
                        # self.base_frame,
                        # "odom"
                        transform_stamped
                    )
    
            odom = Odometry()
            odom.header.frame_id = "odom"
            odom.child_frame_id = self.base_frame
            odom.header.stamp = now.to_msg()
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0.0
            odom.pose.pose.orientation = quaternion
            odom.twist.twist.linear.x = vxy
            odom.twist.twist.linear.y = 0.0
            odom.twist.twist.angular.z = vth
            
            self.current_speed = Twist()
            self.current_speed.linear.x = vxy
            self.current_speed.angular.z = vth

            """
            Covariance values taken from Kobuki node odometry.cpp at:
            https://github.com/yujinrobot/kobuki/blob/indigo/kobuki_node/src/library/odometry.cpp
            
            Pose covariance (required by robot_pose_ekf) TODO: publish realistic values
            Odometry yaw covariance must be much bigger than the covariance provided
            by the imu, as the later takes much better measures
            """
            odom.pose.covariance[0]  = 0.1
            odom.pose.covariance[7]  = 0.1
            if self.use_imu_heading:
                #odom.pose.covariance[35] = 0.2
                odom.pose.covariance[35] = 0.05
            else:
                odom.pose.covariance[35] = 0.05
            
            odom.pose.covariance[14] = sys.float_info.max  # set a non-zero covariance on unused
            odom.pose.covariance[21] = sys.float_info.max  # dimensions (z, pitch and roll); this
            odom.pose.covariance[28] = sys.float_info.max  # is a requirement of robot_pose_ekf

            self.odomPub.publish(odom)
            
            if now > (self.last_cmd_vel + rclpy.duration.Duration(seconds=self.timeout)):
                self.v_des_left = 0
                self.v_des_right = 0
                
            if self.v_left < self.v_des_left:
                self.v_left += self.max_accel
                if self.v_left > self.v_des_left:
                    self.v_left = self.v_des_left
            else:
                self.v_left -= self.max_accel
                if self.v_left < self.v_des_left:
                    self.v_left = self.v_des_left
            
            if self.v_right < self.v_des_right:
                self.v_right += self.max_accel
                if self.v_right > self.v_des_right:
                    self.v_right = self.v_des_right
            else:
                self.v_right -= self.max_accel
                if self.v_right < self.v_des_right:
                    self.v_right = self.v_des_right
            
            # Set motor speeds in encoder ticks per PID loop
            if not self.stopped:
                self.arduino.drive(self.v_left, self.v_right)
                
            self.t_next = now + self.t_delta

    def stop(self):
        self.stopped = True
        self.arduino.drive(0, 0)
            
    def cmdVelCallback(self, req):
        # Handle velocity-based movement requests
        #self.last_cmd_vel = rospy.Time.now()
        self.last_cmd_vel = self.get_clock().now()
        
        x = req.linear.x         # m/s
        th = req.angular.z       # rad/s
        print("base controller cmd vel x:", x, "th:", th)
        if x == 0:
            # Turn in place
            right = th * self.wheel_track  * self.gear_reduction / 2.0
            left = -right
        elif th == 0:
            # Pure forward/backward motion
            left = right = x
        else:
            # Rotation about a point in space
            left = x - th * self.wheel_track  * self.gear_reduction / 2.0
            right = x + th * self.wheel_track  * self.gear_reduction / 2.0
            
        self.v_des_left = int(left * self.ticks_per_meter / self.arduino.PID_RATE)
        self.v_des_right = int(right * self.ticks_per_meter / self.arduino.PID_RATE)
        
    def reset_odometry(self):
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        
