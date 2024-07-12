#!/usr/bin/env python

"""
    A ROS Node for the Arduino microcontroller
    
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

import rclpy
import rclpy.duration
from rclpy.node import Node
from ros_arduino_python.miscellaneous import rc_logger
from ros_arduino_python.arduino_driver import Arduino
# from ros_arduino_python.arduino_sensors import *
from ros_arduino_msgs.srv import ServoAttach, ServoDetach
from ros_arduino_msgs.msg import SensorState
# from ros_arduino_python.diagnostics import DiagnosticsUpdater, DiagnosticsPublisher
# from ros_arduino_python.cfg import ROSArduinoBridgeConfig
# from ros_arduino_python.base_controller import BaseController
# from ros_arduino_python.servo_controller import Servo, ServoController
# from ros_arduino_python.follow_controller import FollowController
# from ros_arduino_python.joint_state_publisher import JointStatePublisher
# import dynamic_reconfigure.server
# import dynamic_reconfigure.client
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty, SetBool
import os, time, threading
from math import radians
from serial.serialutil import SerialException

# controller_types = { "follow_controller" : FollowController }

class ArduinoROS(Node):
    def destroy_node(self):
        self.shutdown()
        super().destroy_node()

    def timer_callback(self):
        str_msg = String()
        str_msg.data = 'Hello World: %d' % self.inum
        self.strtopicPub.publish(str_msg)
        # self.get_logger().info('Publishing: str_msg "%s"' % str_msg.data)
        self.inum += 1

        sensor_state_msg = SensorState()
        sensor_state_msg.name = ["state1", "state2"]
        sensor_state_msg.value = [0.01, 2.57]
        self.sensorStatePub.publish(sensor_state_msg)
        # self.get_logger().info('Publishing: sensor_state_msg "%s"' % sensor_state_msg.name)

    
    def __init__(self):
        super().__init__('arduino')
        self.inum = 0
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 57600)
        self.declare_parameter('timeout', 0.1)
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('rate', 50)

        self.declare_parameter('sensorstate_rate', 10)
        self.declare_parameter('use_base_controller', True)
        self.declare_parameter('diagnotics_error_threshold', 10)
        self.declare_parameter('diagnotics_rate', 1.0)

        # Find the actual node name in case it is set in the launch file
        self.name = self.get_name()
        print("name:", self.name)

        self.port = self.get_parameter("port").get_parameter_value().string_value
        self.baud = self.get_parameter("baud").get_parameter_value().integer_value
        self.timeout = self.get_parameter("timeout").get_parameter_value().double_value
        self.base_frame = self.get_parameter("base_frame").get_parameter_value().string_value
        print("port:", self.port)
        print("baud:", self.baud)
        print("timeout:", self.timeout)
        print("base_frame:", self.base_frame)
        # Overall loop rate: should be faster than fastest sensor rate
        self.rate = self.get_parameter("rate").get_parameter_value().integer_value
        print("rate:", self.rate)
        self._loop_rate = self.create_rate(self.rate, self.get_clock())

        # Rate at which summary SensorState message is published. Individual sensors publish
        # at their own rates.        
        self.sensorstate_rate = self.get_parameter("sensorstate_rate").get_parameter_value().integer_value
        # Are we using the base_controller?
        self.use_base_controller = self.get_parameter("use_base_controller").get_parameter_value().bool_value
        # Default error threshold (percent) before getting a diagnostics warning
        self.diagnotics_error_threshold = self.get_parameter("diagnotics_error_threshold").get_parameter_value().integer_value
        # Diagnostics update rate
        self.diagnotics_rate = self.get_parameter("diagnotics_rate").get_parameter_value().double_value
        print("sensorstate_rate:", self.sensorstate_rate)
        print("use_base_controller:", self.use_base_controller)
        print("diagnotics_error_threshold:", self.diagnotics_error_threshold)
        print("diagnotics_rate:", self.diagnotics_rate)

        # Assume we don't have any joints by default
        self.have_joints = False
        
        # Set up the time for publishing the next SensorState message
        now = self.get_clock().now()
        self.t_delta_sensors = rclpy.duration.Duration(seconds=1.0 / self.sensorstate_rate)
        self.t_next_sensors = now + self.t_delta_sensors
        print("now:", now)
        print("t_delta_sensors:", self.t_delta_sensors)
        print("t_next_sensors:", self.t_next_sensors)
        # Initialize a Twist message
        self.cmd_vel = Twist()
  
        # A cmd_vel publisher so we can stop the robot when shutting down
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 5)
        
        # The SensorState publisher periodically publishes the values of all sensors on
        # a single topic.
        self.sensorStatePub = self.create_publisher(SensorState, 'sensor_state', 5)
        self.strtopicPub = self.create_publisher(String, 'strtopic', 10)
        self.timer = self.create_timer(1, self.timer_callback)

        # A service to attach a PWM servo to a specified pin
        self.create_service(ServoAttach, 'servo_attach', self.ServoAttachHandler)
        
        # A service to detach a PWM servo to a specified pin
        self.create_service(ServoDetach, 'servo_detach', self.ServoDetachHandler)
        
#         # A service to position a PWM servo
#         rospy.Service('~servo_write', ServoWrite, self.ServoWriteHandler)
        
#         # A service to read the position of a PWM servo
#         rospy.Service('~servo_read', ServoRead, self.ServoReadHandler)
        
#         # A service to turn set the direction of a digital pin (0 = input, 1 = output)
#         rospy.Service('~digital_pin_mode', DigitalPinMode, self.DigitalPinModeHandler)
        
#         # Obsolete: Use DigitalPinMode instead
#         rospy.Service('~digital_set_direction', DigitalSetDirection, self.DigitalSetDirectionHandler)
        
#         # A service to turn set the direction of an analog pin (0 = input, 1 = output)
#         rospy.Service('~analog_pin_mode', AnalogPinMode, self.AnalogPinModeHandler)
        
#         # A service to turn a digital sensor on or off
#         rospy.Service('~digital_write', DigitalWrite, self.DigitalWriteHandler)
        
#         # A service to read the value of a digital sensor
#         rospy.Service('~digital_read', DigitalRead, self.DigitalReadHandler) 

#         # A service to set pwm values for the pins
#         rospy.Service('~analog_write', AnalogWrite, self.AnalogWriteHandler)
        
#         # A service to read the value of an analog sensor
#         rospy.Service('~analog_read', AnalogRead, self.AnalogReadHandler)

#         # Initialize the device
#         self.device = Arduino(self.port, self.baud, self.timeout, debug=True)

#         # Make the connection
#         if self.device.connect():
#             rospy.loginfo("Connected to Arduino on port " + self.port + " at " + str(self.baud) + " baud")
#         else:
#             rospy.logerr("No serial connection found.")
#             os._exit(0)

#         # Initialize the base controller if used
#         if self.use_base_controller:
#             self.base_controller = BaseController(self.device, self.base_frame, self.name + "_base_controller")
            
#             # A service to reset the odometry values to 0
#             rospy.Service('~reset_odometry', Empty, self.ResetOdometryHandler)
    
#             # A service to update the PID parameters Kp, Kd, Ki, and Ko
#             rospy.Service('~update_pid', UpdatePID, self.UpdatePIDHandler)
            
#             # Fire up the dynamic_reconfigure server
#             dyn_server = dynamic_reconfigure.server.Server(ROSArduinoBridgeConfig, self.dynamic_reconfigure_server_callback, namespace=self.name)

#             # Connect to the dynamic_reconfigure client
#             dyn_client = dynamic_reconfigure.client.Client (self.name, timeout=5)
     
#         # Reserve a thread lock
#         mutex = thread.allocate_lock()  # mutex = threading.Lock()

#         # Initialize any sensors
#         self.device.sensors = list()

#         # Keep a list of IMUs or gyros used for odometry so they can be reset
#         self.imu_for_odom = list()

#         # Read in the sensors parameter dictionary
#         sensor_params = rospy.get_param("~sensors", dict({}))

#         # Initialize individual sensors appropriately
#         for name, params in sensor_params.iteritems():
#             if params['type'].lower() == 'Ping'.lower():
#                 sensor = Ping(self.device, name, **params)
#             elif params['type'].lower() == 'GP2D12'.lower() or params['type'].lower() == 'GP2Y0A21YK0F'.lower():
#                 sensor = GP2D12(self.device, name, **params)
#             elif params['type'].lower() == 'Digital'.lower():
#                 sensor = DigitalSensor(self.device, name, **params)
#             elif params['type'].lower() == 'Analog'.lower():
#                 sensor = AnalogSensor(self.device, name, **params)
#             elif params['type'].lower() == 'AnalogFloat'.lower():
#                 sensor = AnalogFloatSensor(self.device, name, **params)
#             elif params['type'].lower() == 'PololuMotorCurrent'.lower():
#                 sensor = PololuMotorCurrent(self.device, name, **params)
#             elif params['type'].lower() == 'PhidgetsVoltage'.lower():
#                 sensor = PhidgetsVoltage(self.device, name, **params)
#             elif params['type'].lower() == 'PhidgetsCurrent'.lower():
#                 sensor = PhidgetsCurrent(self.device, name, **params)
#             elif params['type'].lower() == 'IMU'.lower():
#                 sensor = IMU(self.device, name, **params)
#                 try:
#                     if params['use_for_odom']:
#                         self.imu_for_odom.append(sensor)
#                 except:
#                     pass
#             elif params['type'].lower() == 'Gyro'.lower():
#                 try:
#                     sensor = Gyro(self.device, name, base_controller=self.base_controller, **params)
#                 except:
#                     sensor = Gyro(self.device, name, **params)

#                 try:
#                     if params['use_for_odom']:
#                         self.imu_for_odom.append(sensor)
#                 except:
#                     pass
                
# #                if params['type'].lower() == 'MaxEZ1'.lower():
# #                    self.sensors[len(self.sensors)]['trigger_pin'] = params['trigger_pin']
# #                    self.sensors[len(self.sensors)]['output_pin'] = params['output_pin']

#             try:
#                 self.device.sensors.append(sensor)
            
#                 if params['rate'] != None and params['rate'] != 0:
#                     rospy.loginfo(name + " " + str(params) + " published on topic " + rospy.get_name() + "/sensor/" + name)
#                 else:
#                     if sensor.direction == "input":
#                         rospy.loginfo(name + " service ready at " + rospy.get_name() + "/" + name + "/read")
#                     else:
#                         rospy.loginfo(name + " service ready at " + rospy.get_name() + "/" + name + "/write")
#             except:
#                 rospy.logerr("Sensor type " + str(params['type'] + " not recognized."))

#         # If at least one IMU or gyro is used for odometry, set the use_imu_heading flag on the base controller
#         if self.use_base_controller and len(self.imu_for_odom) > 0:
#             self.base_controller.use_imu_heading = True
        
#         # Read in the joints (if any)    
#         joint_params = rospy.get_param("~joints", dict())
        
#         if len(joint_params) != 0:
#             self.have_joints = True
            
#             self.device.joints = dict()
            
#             self.device.joint_update_rate = float(rospy.get_param("~joint_update_rate", 10))
            
#             # Configure each servo
#             for name, params in joint_params.iteritems():
#                 try:
#                     if params['continuous'] == True:
#                         self.device.joints[name] = ContinuousServo(self.device, name)
#                     else:
#                         self.device.joints[name] = Servo(self.device, name)
#                 except:
#                     self.device.joints[name] = Servo(self.device, name)

#                 # Display the joint setup on the terminal
#                 rospy.loginfo(name + " " + str(params))
            
#             # The servo controller determines when to read and write position values to the servos
#             self.servo_controller = ServoController(self.device, "ServoController")
            
#             # The joint state publisher publishes the latest joint values on the /joint_states topic
#             self.joint_state_publisher = JointStatePublisher()
            
#         # Create the diagnostics updater for the Arduino device
#         self.device.diagnostics = DiagnosticsUpdater(self, self.name, self.diagnotics_error_threshold, self.diagnotics_rate, create_watchdog=True)
        
#         # Create the overall diagnostics publisher
#         self.diagnostics_publisher = DiagnosticsPublisher(self)
            
#         # Initialize any trajectory action follow controllers
#         controllers = rospy.get_param("~controllers", dict())
          
#         self.device.controllers = list()
          
#         for name, params in controllers.items():
#             try:
#                 controller = controller_types[params["type"]](self.device, name)
#                 self.device.controllers.append(controller)
#             except Exception as e:
#                 if type(e) == KeyError:
#                     rospy.logerr("Unrecognized controller: " + params["type"])
#                 else:  
#                     rospy.logerr(str(type(e)) + str(e))
  
#         for controller in self.device.controllers:
#             controller.startup()
            
#         print("\n==> ROS Arduino Bridge ready for action!")
    
#         # Start polling the sensors, base controller, and servo controller
#         while not rospy.is_shutdown(): # rclpy.ok()
#             # Heartbeat/watchdog test for the serial connection
#             try:
#                 # Update read counters
#                 self.device.diagnostics.reads += 1
#                 self.device.diagnostics.total_reads += 1
#                 self.device.serial_port.inWaiting()
#                 # Add this heartbeat to the frequency status diagnostic task
#                 self.device.diagnostics.freq_diag.tick()
#                 # Let the diagnostics updater know we're still alive
#                 self.device.diagnostics.watchdog = True
#             except IOError:
#                 # Update error counter
#                 self.device.diagnostics.errors += 1
#                 rospy.loginfo("Lost serial connection. Waiting to reconnect...")
#                 # Let the diagnostics updater know that we're down
#                 self.device.diagnostics.watchdog = False
#                 self.device.close()
#                 with mutex:
#                     while True:
#                         try:
#                             self.device.open()
#                             while True:
#                                 self.device.serial_port.write('\r')
#                                 test = self.device.serial_port.readline().strip('\n').strip('\r')
#                                 rospy.loginfo("Waking up serial port...")
#                                 if test == 'Invalid Command':
#                                     self.device.serial_port.flushInput()
#                                     self.device.serial_port.flushOutput()
#                                     break
#                             rospy.loginfo("Serial connection re-established.")
#                             break
#                         except:
#                             r.sleep()
#                             self.diagnostics_publisher.update()
#                             continue
            
#             # Poll any sensors
#             for sensor in self.device.sensors:
#                 if sensor.rate != 0:
#                     with mutex:
#                         sensor.poll()
                                     
#             # Poll the base controller
#             if self.use_base_controller:
#                 with mutex:
#                     self.base_controller.poll()
                
#             # Poll any joints
#             if self.have_joints:
#                 with mutex:
#                     self.servo_controller.poll()
#                     self.joint_state_publisher.poll(self.device.joints.values())
                                
#             # Publish all sensor values on a single topic for convenience
#             now = rospy.Time.now()
            
#             if now > self.t_next_sensors:
#                 msg = SensorState()
#                 msg.header.frame_id = self.base_frame
#                 msg.header.stamp = now
#                 for i in range(len(self.device.sensors)):
#                     if self.device.sensors[i].message_type != MessageType.IMU:
#                         msg.name.append(self.device.sensors[i].name)
#                         msg.value.append(self.device.sensors[i].value)
#                 try:
#                     self.sensorStatePub.publish(msg)
#                 except:
#                     pass
                
#                 self.t_next_sensors = now + self.t_delta_sensors
                
#             # Update diagnostics and publish
#             self.diagnostics_publisher.update()
            
#             r.sleep()
    
    # Service callback functions
    def ServoAttachHandler(self, req, res):
        #self.device.attach_servo(req.id)
        print("handler ServoAttach req.id:", req.id)
        return res
    
    def ServoDetachHandler(self, req, res):
        #self.device.detach_servo(req.id)
        print("handler ServoDetach req.id:", req.id)
        return res
    
    def ServoWriteHandler(self, req, res):
        #self.device.servo_write(req.id, req.position)
        print("handler ServoWrite req.id:", req.id)
        return res
    
#     def ServoSpeedWriteHandler(self, req):
#         delay = 
#         self.device.servo_delay(req.id, delay)
#         return ServoSpeedResponse()
# 
#         # Convert servo speed in deg/s to a step delay in milliseconds
#         step_delay = self.device.joints[name].get_step_delay(req.value)
# 
#         # Update the servo speed
#         self.device.config_servo(pin, step_delay)
#         
#         return SetServoSpeedResponse()
    
#     def ServoReadHandler(self, req):
#         position = self.device.servo_read(req.id)
#         return ServoReadResponse(position)
    
#     def DigitalPinModeHandler(self, req):
#         self.device.digital_pin_mode(req.pin, req.direction)
#         return DigitalPinModeResponse()
    
#     # Obsolete: use DigitalPinMode instead
#     def DigitalSetDirectionHandler(self, req):
#         self.device.digital_pin_mode(req.pin, req.direction)
#         return DigitalSetDirectionResponse()
    
#     def DigitalWriteHandler(self, req):
#         self.device.digital_write(req.pin, req.value)
#         return DigitalWriteResponse()
    
#     def DigitalReadHandler(self, req):
#         value = self.device.digital_read(req.pin)
#         return DigitalReadResponse(value)
    
#     def AnalogPinModeHandler(self, req):
#         self.device.analog_pin_mode(req.pin, req.direction)
#         return AnalogPinModeResponse()
              
#     def AnalogWriteHandler(self, req):
#         self.device.analog_write(req.pin, req.value)
#         return AnalogWriteResponse()
    
#     def AnalogReadHandler(self, req):
#         value = self.device.analog_read(req.pin)
#         return AnalogReadResponse(value)

#     def ResetOdometryHandler(self, req):
#         if self.use_base_controller:
#             self.base_controller.reset_odometry()
#             for imu in self.imu_for_odom:
#                 imu.reset()
#         else:
#             rospy.logwarn("Not using base controller!")
#         return EmptyResponse()  # SetBool.Request() SetBool.Response()

#     def UpdatePIDHandler(self, req):
#         if self.use_base_controller:
#            self.device.update_pid(req.Kp, req.Kd, req.Ki, req.Ko)
#            rospy.loginfo("Updating PID parameters: Kp=%0.3f, Kd=%0.3f, Ki=%0.3f, Ko=%0.3f" %(req.Kp, req.Kd, req.Ki, req.Ko))
#         else:
#             rospy.logwarn("Not using base controller!")
#         return UpdatePIDResponse()

#     def dynamic_reconfigure_server_callback(self, config, level):
#         if self.use_base_controller:
#             try:
#                 if self.base_controller.Kp != config['Kp']:
#                     self.base_controller.Kp = config['Kp']

#                 if self.base_controller.Kd != config['Kd']:
#                     self.base_controller.Kd = config['Kd']

#                 if self.base_controller.Ki != config['Ki']:
#                     self.base_controller.Ki = config['Ki']

#                 if self.base_controller.Ko != config['Ko']:
#                     self.base_controller.Ko = config['Ko']

#                 if self.base_controller.accel_limit != config['accel_limit']:
#                     self.base_controller.accel_limit = config['accel_limit']
#                     self.base_controller.max_accel = self.base_controller.accel_limit * self.base_controller.ticks_per_meter / self.base_controller.rate

#                 if self.base_controller.odom_linear_scale_correction != config['odom_linear_scale_correction']:
#                     self.base_controller.odom_linear_scale_correction = config['odom_linear_scale_correction']
                    
#                 if self.base_controller.odom_angular_scale_correction != config['odom_angular_scale_correction']:
#                     self.base_controller.odom_angular_scale_correction = config['odom_angular_scale_correction']

#                 self.device.update_pid(self.base_controller.Kp, self.base_controller.Kd, self.base_controller.Ki, self.base_controller.Ko)

#                 rospy.loginfo("Updating PID parameters: Kp=%0.2f, Kd=%0.2f, Ki=%0.2f, Ko=%0.2f, accel_limit=%0.2f" %(self.base_controller.Kp, self.base_controller.Kd, self.base_controller.Ki, self.base_controller.Ko, self.base_controller.accel_limit))
#             except Exception as e:
#                 print(e)

#         return config
        
#     def shutdown(self):
#         rospy.loginfo("Shutting down Arduino node...")
        
#         # Stop the robot
#         if self.use_base_controller:
#             rospy.loginfo("Stopping the robot...")
#             self.cmd_vel_pub.publish(Twist())
#             rospy.sleep(2)

#         # Detach any servos
#         if self.have_joints:
#             rospy.loginfo("Detaching servos...")
#             for joint in self.device.joints.values():
#                 self.device.detach_servo(joint.pin)
#                 rospy.sleep(0.2)
                
#         # Close the serial port
#         self.device.close()
        
def main(args=None):
    try:
        rclpy.init(args=args)
        myArduino = ArduinoROS()

        rclpy.spin(myArduino)

        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        myArduino.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        try:
            myArduino.device.serial_port.close()
        except:
            rc_logger.warn("Serial exception trying to close port.")
            os._exit(0)
    except SerialException:
        rc_logger.warn("Serial exception trying to open port.")
        os._exit(0)
        
if __name__ == '__main__':
    main()