#!/usr/bin/env python

"""
    Sensor class for the arudino_python package
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""

import rclpy, time
from rclpy.node import Node
from ros_arduino_python.miscellaneous import rc_logger
from sensor_msgs.msg import Range, Imu
from geometry_msgs.msg import Twist, Quaternion, Vector3
from ros_arduino_python.arduino_driver import CommandErrorCode, CommandException
from ros_arduino_python.diagnostics import DiagnosticsUpdater

from ros_arduino_msgs.srv import DigitalSensorWrite, DigitalSensorRead
from ros_arduino_msgs.srv import AnalogSensorWrite, AnalogSensorRead
from ros_arduino_msgs.srv import AnalogFloatSensorWrite, AnalogFloatSensorRead
from ros_arduino_msgs.msg import Digital, Analog, AnalogFloat

from math import pow, radians
import tf2_ros
# https://github.com/DLu/tf_transformations/
from tf_transformations import quaternion_from_euler
import sys

LOW = 0
HIGH = 1

INPUT = 0
OUTPUT = 1

class MessageType:
    ANALOG = 0
    DIGITAL = 1
    RANGE = 2
    FLOAT = 3
    INT = 4
    BOOL = 5
    IMU = 6
    
class Sensor:
    def __init__(self, device, name, pin=None, node=None, rate=0, direction="input", frame_id="base_link", **kwargs):
        print("sensor node init, name:", name)
        self.device = device
        self.name = name
        self.pin = pin
        self.node = node
        self.rate = rate
        self.direction = direction
        self.frame_id = frame_id

        # Set min/max/offset/scale if specified
        self.min = self.get_kwargs(kwargs, 'min', 0)
        self.max = self.get_kwargs(kwargs, 'max', float('inf'))
        self.offset = self.get_kwargs(kwargs, 'offset', 0)
        self.scale = self.get_kwargs(kwargs, 'scale', 1)

        # Track diagnostics for this component
        diagnotics_error_threshold = self.get_kwargs(kwargs, 'diagnotics_error_threshold', 10)
        diagnostics_rate = float(self.get_kwargs(kwargs, 'diagnostics_rate', 1))
        print("sensor error_threshold:", diagnotics_error_threshold)
        print("sensor rate:", diagnostics_rate)
        # The DiagnosticsUpdater class is defined in the diagnostics.py module
        #self.diagnostics = DiagnosticsUpdater(self, 
        #                                      name + '_sensor', 
        #                                      diagnotics_error_threshold, 
        #                                      diagnostics_rate)    

        # Initialize the component's value
        self.value = None

        # Create the default publisher
        if self.rate != 0:
            self.custom_create_publisher()

        # Create any appropriate services
        self.custom_create_services()

        # Intialize the next polling time stamp
        if self.rate != 0:
            now = self.node.get_clock().now()
            self.t_delta = rclpy.duration.Duration(seconds=1.0 / self.rate)
            self.t_next = now + self.t_delta
            print("now:", now)
            print("t_delta:", self.t_delta)
            print("t_next:", self.t_next)

    def get_kwargs(self, kwargs, arg, default):
        try:
            return kwargs[arg]
        except:
            return default

    def custom_create_publisher(self):
        # Override per sensor type
        pass

    def custom_create_services(self):
        # Override per sensor type
        pass

    def read_value(self):
        pass

    def write_value(self):
        pass

    def publish_message(self):
        # Override this method if necessary for particular sensor types
        if self.direction == "input":
            self.value = self.read_value()
        else:
           self.write_value()

        self.msg.value = self.value
        self.msg.header.stamp = self.node.get_clock().now().to_msg()
        self.pub.publish(self.msg)
    
    def poll(self):
        now = self.node.get_clock().now()
        if now > self.t_next:
            # Update read counters
            #self.diagnostics.reads += 1
            #self.diagnostics.total_reads += 1
            # Add a successful poll to the frequency status diagnostic task
            #self.diagnostics.freq_diag.tick()
            try:
                self.publish_message()
            except CommandException as e:
                # Update error counter
                #self.diagnostics.errors += 1
                rc_logger.error('Command Exception: ' + CommandErrorCode.ErrorCodeStrings[e.code])
                rc_logger.error("Invalid value read from sensor: " + str(self.name))
            except TypeError as e:
                # Update error counter
                #self.diagnostics.errors += 1
                rc_logger.error('Type Error: ' + e.message)

            # Compute the next polling time stamp
            self.t_next = now + self.t_delta
    
class AnalogSensor(Sensor):
    def __init__(self, *args, **kwargs):
        super(AnalogSensor, self).__init__(*args, **kwargs)
                
        self.message_type = MessageType.ANALOG
        
        self.msg = Analog()
        self.msg.header.frame_id = self.frame_id

    def create_publisher(self):
        self.pub = self.node.create_publisher(Analog, "sensor/" + self.name, 5)

    def create_services(self):
        if self.direction == "output":
            self.device.analog_pin_mode(self.pin, OUTPUT)
            self.node.create_service(AnalogSensorWrite, self.name + '/write', self.sensor_write_handler)
        else:
            self.device.analog_pin_mode(self.pin, INPUT)
            self.node.create_service(AnalogSensorRead, self.name + '/read', self.sensor_read_handler)

    def read_value(self):
        return self.scale * (self.device.analog_read(self.pin) - self.offset)

    def write_value(self, value):
        return self.device.analog_write(self.pin, value)

    def sensor_read_handler(self, req, res):
        self.value = self.read_value()
        res.value = self.value
        return res

    def sensor_write_handler(self, req, res):
        self.write_value(req.value)
        self.value = req.value
        return res

class AnalogFloatSensor(AnalogSensor):
    def __init__(self, *args, **kwargs):
        super(AnalogFloatSensor, self).__init__(*args, **kwargs)
                
        self.message_type = MessageType.ANALOG
        
        self.msg = AnalogFloat()
        self.msg.header.frame_id = self.frame_id

    def create_publisher(self):
        self.pub = self.node.create_publisher(AnalogFloat, "sensor/" + self.name, 5)

    def create_services(self):
        if self.direction == "output":
            self.device.analog_pin_mode(self.pin, OUTPUT)
            self.node.create_service(AnalogFloatSensorWrite, self.name + '/write', self.sensor_write_handler)
        else:
            self.device.analog_pin_mode(self.pin, INPUT)
            self.node.create_service(AnalogFloatSensorRead, self.name + '/read', self.sensor_read_handler)

    def read_value(self):
        return self.scale * (self.device.analog_read(self.pin) - self.offset)
    
    def write_value(self, value):
        return self.device.analog_write(self.pin, value)
    
    def sensor_read_handler(self, req, res):
        self.value = self.read_value()
        res.value = self.value
        return res
    
    def sensor_write_handler(self, req, res):
        self.write_value(req.value)
        self.value = req.value
        return res
        
class DigitalSensor(Sensor):
    def __init__(self, *args, **kwargs):
        Sensor.__init__(self, *args, **kwargs)
        
        self.message_type = MessageType.BOOL
        
        self.msg = Digital()
        self.msg.header.frame_id = self.frame_id
        print("frame id:", self.frame_id)

        # Get the initial state
        self.value = self.read_value()
        #self.cmd_vel_sub = self.node.create_subscription(Twist, 'cmd_vel', self.cmdVelCallback, 1)

    def cmdVelCallback(self, req):
        self.last_cmd_vel = self.node.get_clock().now()
        x = req.linear.x         # m/s
        th = req.angular.z       # rad/s
        print("digital sensor cmd_vel x:", x, "th:", th)

    def custom_create_publisher(self):
        self.pub = self.node.create_publisher(Digital, "sensor/" + self.name, 5)

    def custom_create_services(self):
        if self.direction == "output":
            self.device.digital_pin_mode(self.pin, OUTPUT)
            self.node.create_service(DigitalSensorWrite, self.name + '/write', self.sensor_write_handler)
        else:
            self.device.digital_pin_mode(self.pin, INPUT)
            self.node.create_service(DigitalSensorRead, self.name + '/read', self.sensor_read_handler)

    def read_value(self):
        return self.device.digital_read(self.pin)
    
    def write_value(self, value=None):
        # Alternate HIGH/LOW when publishing at a fixed rate
        if self.rate != 0:
            self.value = not self.value
        else:
            self.value = value
        
        return self.device.digital_write(self.pin, self.value)
    
    def sensor_read_handler(self, req=None, res=None):
        self.value = self.read_value()
        res.value = self.value
        return res
    
    def sensor_write_handler(self, req, res):
        self.write_value(req.value)
        self.value = req.value
        print("req write value:", req.value)
        return res  
    
class RangeSensor(Sensor):
    def __init__(self, *args, **kwargs):
        super(RangeSensor, self).__init__(*args, **kwargs)
        
        self.message_type = MessageType.RANGE
        
        self.msg = Range()
        self.msg.header.frame_id = self.frame_id

    def create_publisher(self):
        self.pub = self.node.create_publisher(Range, "sensor/" + self.name, 5)
    
    def create_services(self):
        self.node.create_service(AnalogFloatSensorRead, self.name + '/read', self.sensor_read_handler)
        
    def publish_message(self):
        self.value = self.read_value()
        self.msg.range = self.value
        self.msg.header.stamp = self.node.get_clock().now()
        self.pub.publish(self.msg)

    def sensor_read_handler(self, req, res):
        self.value = self.read_value()
        res.value = self.value
        return res

class SonarSensor(RangeSensor):
    def __init__(self, *args, **kwargs):
        super(SonarSensor, self).__init__(*args, **kwargs)
        
        self.msg.radiation_type = Range.ULTRASOUND
        
class IRSensor(RangeSensor):
    def __init__(self, *args, **kwargs):
        super(IRSensor, self).__init__(*args, **kwargs)
        
        self.msg.radiation_type = Range.INFRARED
        
class Ping(SonarSensor):
    def __init__(self,*args, **kwargs):
        super(Ping, self).__init__(*args, **kwargs)
                
        self.msg.field_of_view = 0.7
        self.msg.min_range = 0.02
        self.msg.max_range = 3.0
        
    def read_value(self):
        # The Arduino Ping code returns the distance in centimeters
        cm = self.device.ping(self.pin)
        
        # Convert it to meters for ROS
        distance = cm / 100.0
        
        return distance
        
class GP2D12(IRSensor):
    # The GP2D12 has been replaced by the GP2Y0A21YK0F
    def __init__(self, *args, **kwargs):
        super(GP2D12, self).__init__(*args, **kwargs)
        
        self.msg.field_of_view = 0.09
        self.msg.min_range = 0.10
        self.msg.max_range = 0.80
        
    def read_value(self):
        value = self.device.analog_read(self.pin)
        
        # The GP2D12 cannot provide a meaning result closer than 3 cm.
        if value <= 3.0:
            return float('NaN')
        
        try:
            #distance = pow(4187.8 / value, 1.106)
            distance = (6787.0 / (float(value) - 3.0)) - 4.0
        except:
            return float('NaN')
            
        # Convert to meters
        distance /= 100.0
        
        # If we get a spurious reading, set it to the max_range
        if distance > self.msg.max_range: distance = float('NaN')
        if distance < self.msg.min_range: distance = float('NaN')
        
        return distance

class IMU(Sensor):
    def __init__(self, *args, **kwargs):
        super(IMU, self).__init__(*args, **kwargs)

        self.message_type = MessageType.IMU
        self.direction = "input"

        self.msg = Imu()
        self.msg.header.frame_id = self.frame_id

        self.msg.orientation_covariance = [1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6]
        self.msg.angular_velocity_covariance = [1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6]
        self.msg.linear_acceleration_covariance = [1e-6, 0, 0, 0, 1e6, 0, 0, 0, 1e6]

    def create_publisher(self):
        self.pub = self.node.create_publisher(Imu, "sensor/" + self.name, 5)

    def read_value(self):
        '''
        IMU data is assumed to be returned in the following order:

        [ax, ay, az, gx, gy, gz, mx, my, mz, roll, pitch, ch]

        where a stands for accelerometer, g for gyroscope and m for magnetometer.
        The last value ch stands for "compensated heading" that some IMU's can 
        compute to compensate magnetic heading for the current roll and pitch. 
        '''
        data  = self.device.get_imu_data()

        try:
            ax, ay, az, gx, gy, gz, mx, my, mz, roll, pitch, ch = data
        except:
            rc_logger.error("Invalid value read from sensor: " + str(self.name))
            return None

        roll = radians(roll)
        pitch = -radians(pitch)

        self.msg.linear_acceleration.x = ax
        self.msg.linear_acceleration.y = ay
        self.msg.linear_acceleration.z = az

        self.msg.angular_velocity.x = radians(gx)
        self.msg.angular_velocity.y = radians(gy)
        self.msg.angular_velocity.z = radians(gz)

        if ch != -999:
            yaw = -radians(ch)
        else:
            yaw = -radians(mz)

        (self.msg.orientation.x, self.msg.orientation.y, self.msg.orientation.z, self.msg.orientation.w) = quaternion_from_euler(roll, pitch, yaw)

        return data

    def publish_message(self):
        self.read_value()
        self.msg.header.stamp = self.node.get_clock().now()
        self.pub.publish(self.msg)

class Gyro(Sensor):
    def __init__(self, *args, **kwargs):
        super(Gyro, self).__init__(*args, **kwargs)

        try:
            self.base_controller = kwargs['base_controller']
        except:
            self.base_controller = None

        self.message_type = MessageType.IMU
        self.direction = "input"

        self.sensitivity =  self.get_parameter('sensors/' + self.name + '/sensitivity', None)
        self.voltage = self.get_parameter('sensors/' + self.name + '/voltage').get_parameter_value().double_value  # default 5.0
        self.gyro_scale_correction = self.get_parameter('sensors/' + self.name + '/gyro_scale_correction').get_parameter_value().double_value  # default 1.0

        # Time in seconds to collect initial calibration data at startup
        self.cal_start_interval = self.get_parameter('sensors/' + self.name + '/cal_start_interval').get_parameter_value().double_value  # default 5.0

        if self.sensitivity is None:
            rc_logger.error("Missing sensitivity parameter for gyro.")
            #rospy.signal_shutdown("Missing sensitivity parameter for gyro.")


        self.rad_per_sec_per_adc_unit = radians(self.voltage / 1023.0 / self.sensitivity)

        self.orientation = 0.0
        self.last_time = None

        self.cal_offset = None
        self.cal_drift_threshold = self.get_parameter('sensors/' + self.name + '/cal_drift_threshold').get_parameter_value().double_value  # default 0.1
        self.cal_buffer = []
        self.cal_drift_buffer = []
        self.cal_buffer_length = 1000
        self.cal_drift_buffer_length = 1000

        self.msg = Imu()

        self.msg.header.frame_id = self.frame_id

        self.msg.orientation_covariance = [sys.float_info.max, 0, 0, 0, sys.float_info.max, 0, 0, 0, 0.05]
        self.msg.angular_velocity_covariance = [sys.float_info.max, 0, 0, 0, sys.float_info.max, 0, 0, 0, 0.05]
        self.msg.linear_acceleration_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]

        print("\n*** DO NOT MOVE GYRO FOR", self.cal_start_interval, "SECONDS TO ALLOW OFFSET CALLIBRATION ***\n")
        update_interval = 1.0 / self.rate
        cal_time = 0.0
        while cal_time < self.cal_start_interval:
            gyro_data = self.device.analog_read(self.pin)
            self.update_calibration(gyro_data)
            #rospy.sleep(update_interval)
            time.sleep(update_interval)
            cal_time += update_interval

    def create_publisher(self):
        self.pub = self.node.create_publisher(Imu, "sensor/" + self.name, 5)

    def read_value(self):
        gyro_data = self.device.analog_read(self.pin)

        # If the robot is not moving, update the gyro calibration
        if self.base_controller is not None and self.base_controller.current_speed == Twist():
            self.update_calibration(gyro_data)
   
        # If this is the first measurement, just record the current time
        if self.last_time is None:
            self.last_time = self.node.get_clock().now()
            return

        # Store the current time
        current_time = self.node.get_clock().now()

        # Compute the time since the last measurement
        dt = (current_time - self.last_time).to_sec()

        self.last_time = current_time

        # Compute angular velocity from the raw gyro data
        angular_velocity = self.gyro_scale_correction * self.rad_per_sec_per_adc_unit * (gyro_data - self.cal_offset)

        # Right-hand coordinate system
        angular_velocity = -1.0 * angular_velocity

        # Ignore small values that are likely due to drift
        if abs(angular_velocity) < self.cal_drift_threshold:
            angular_velocity = 0

        # Update the orientation by integrating angular velocity over time
        self.orientation += angular_velocity * dt

        # Fill in the Imu message
        self.msg.header.stamp =  current_time
        self.msg.angular_velocity.z  = angular_velocity
        (self.msg.orientation.x, self.msg.orientation.y, self.msg.orientation.z, self.msg.orientation.w) = quaternion_from_euler(0, 0, self.orientation)

        return self.msg

    def publish_message(self):
        self.read_value()
        self.msg.header.stamp = self.node.get_clock().now()
        self.pub.publish(self.msg)

    def update_calibration(self, gyro_data):
        # Collect raw analog values when stoped so we can compute the ADC offset
        self.cal_buffer.append(gyro_data)

        if len(self.cal_buffer) > self.cal_buffer_length:
            del self.cal_buffer[:-self.cal_buffer_length]

        if len(self.cal_drift_buffer) > self.cal_drift_buffer_length:
            del self.cal_drift_buffer[:-self.cal_drift_buffer_length]

        try:
            # Collect angular velocity values when stopped to compute the drift estimate
            angular_velocity = self.gyro_scale_correction * self.rad_per_sec_per_adc_unit * (gyro_data - self.cal_offset)
            self.cal_drift_buffer.append(abs(angular_velocity))
        except:
            pass

        try:
            # Use the max absolute angular velocity when stopped as the drift estimated
            self.cal_drift_offset = max(self.cal_drift_buffer, key=lambda x: abs(x))
        except:
            pass

        try:
            self.cal_offset = sum(self.cal_buffer) / len(self.cal_buffer)
        except:
            pass

    def reset(self):
        self.orientation = 0
        self.msg.orientation = Quaternion()
        self.msg.orientation.w = 1.0
        self.msg.angular_velocity = Vector3()
        self.msg.linear_acceleration = Vector3()

class PololuMotorCurrent(AnalogFloatSensor):
    def __init__(self, *args, **kwargs):
        super(PololuMotorCurrent, self).__init__(*args, **kwargs)

    def read_value(self):
        # From the Pololu source code
        milliamps = self.device.analog_read(self.pin) * 34
        return milliamps / 1000.0

class PhidgetsVoltage(AnalogFloatSensor):
    def __init__(self, *args, **kwargs):
        super(PhidgetsVoltage, self).__init__(*args, **kwargs)

    def read_value(self):
        # From the Phidgets documentation
        voltage = 0.06 * (self.device.analog_read(self.pin) - 500.)
        return voltage

class PhidgetsCurrent(AnalogFloatSensor):
    def __init__(self, *args, **kwargs):
        super(PhidgetsCurrent, self).__init__(*args, **kwargs)
        
    def read_value(self):
        # From the Phidgets documentation for the 20 amp DC sensor
        current = 0.05 * (self.device.analog_read(self.pin) - 500.)
        return current

class MaxEZ1Sensor(SonarSensor):
    def __init__(self, *args, **kwargs):
        super(MaxEZ1Sensor, self).__init__(*args, **kwargs)

        self.trigger_pin = kwargs['trigger_pin']
        self.output_pin = kwargs['output_pin']

        self.msg.field_of_view = 0.785398163
        self.msg.min_range = 0.02
        self.msg.max_range = 3.0

    def read_value(self):
        return self.device.get_MaxEZ1(self.trigger_pin, self.output_pin)

            
