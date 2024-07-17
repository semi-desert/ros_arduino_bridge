#!/usr/bin/env python

"""
  follow_controller.py - controller for a kinematic chain
  Copyright (c) 2011 Vanadium Labs LLC.  All right reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of Vanadium Labs LLC nor the names of its 
        contributors may be used to endorse or promote products derived 
        from this software without specific prior written permission.
  
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

import rclpy
import rclpy.duration
from rclpy.action import ActionServer
from ros_arduino_python.miscellaneous import rc_logger
from ros_arduino_python.controllers import Controller

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue


class FollowController(Controller):
    """ A controller for joint chains, exposing a FollowJointTrajectory action. """

    def __init__(self, device, name, node=None):
        #Controller.__init__(self, device, name)
        super().__init__(device, name, node)
        
        # Parameters: rates and joints
        self.rate = self.node.get_parameter('controllers/' + name + '/rate').get_parameter_value().double_value  # default 50.0
        self.joints = self.node.get_parameter('controllers/' + name + '/joints').get_parameter_value()
        self.index = self.node.get_parameter('controllers/'+ name + '/index', ).get_parameter_value().integer_value  # default len(device.controllers)
        for joint in self.joints:
            self.device.joints[joint].controller = self

        # Action server
        name = self.node.get_parameter('controllers/' + name + '/action_name').get_parameter_value().string_value  # default follow_joint_trajectory
        #self.server = actionlib.SimpleActionServer(name, FollowJointTrajectoryAction, execute_cb=self.execute_cb, auto_start=False)
        self.server = ActionServer(self.node, 
                                   FollowJointTrajectory, 
                                   name, 
                                   self.execute_cb
                                )

        # Good old trajectory
        self.node.create_subscription(JointTrajectory, self.name + '/command', self.command_cb, 10)
        self.executing = False

        self.node.get_logger.info("Started FollowController ("+self.name+"). Joints: " + str(self.joints) + " on C" + str(self.index))

    def startup(self):
        self.server.start()

    def execute_cb(self, goal):
        self.node.get_logger().info(self.name + ": Action goal recieved.")
        traj = goal.trajectory
        
        if set(self.joints) != set(traj.joint_names):
            for j in self.joints:
                if j not in traj.joint_names:
                    msg = "Trajectory joint names does not match action controlled joints." + str(traj.joint_names)
                    self.node.get_logger().error(msg)
                    self.server.set_aborted(text=msg)
                    return
            self.node.get_logger().warn("Extra joints in trajectory")

        if not traj.points:
            msg = "Trajectory empy."
            self.node.get_logger().error(msg)
            self.server.set_aborted(text=msg)
            return

        try:
            indexes = [traj.joint_names.index(joint) for joint in self.joints]
        except ValueError as val:
            msg = "Trajectory invalid."
            self.node.get_logger().error(msg)
            self.server.set_aborted(text=msg)
            return

        if self.execute_trajectory(traj):   
            self.server.set_succeeded()
        else:
            self.server.set_aborted(text="Execution failed.")

        self.node.get_logger().info(self.name + ": Done.")
    
    def command_cb(self, msg):
        # Don't execute if executing an action
        if self.server.is_active():
            self.node.get_logger().info(self.name + ": Received trajectory, but action is active")
            return
        self.executing = True
        self.execute_trajectory(msg)
        self.executing = False    

    def execute_trajectory(self, traj):
        self.node.get_logger().info("Executing trajectory")
        self.node.get_logger().debug(traj)
        # carry out trajectory
        try:
            indexes = [traj.joint_names.index(joint) for joint in self.joints]
        except ValueError as val:
            self.node.get_logger().error("Invalid joint in trajectory.")
            return False

        # Get starting timestamp, MoveIt uses 0, need to fill in
        start = traj.header.stamp
        if start.secs == 0 and start.nsecs == 0:
            start = self.node.get_clock().now()
        
        # Set the start time for each joint
        for i in range(len(self.joints)):
            self.device.joints[self.joints[i]].time_move_started = start

        r = self.create_rate(self.rate)
        last = [ self.device.joints[joint].position for joint in self.joints ]
        for point in traj.points:
            while self.node.get_clock().now() + rclpy.duration.Duration(seconds=0.01) < start:
                r.sleep()
            desired = [ point.positions[k] for k in indexes ]
            endtime = start + point.time_from_start
            while self.node.get_clock().now() + rclpy.duration.Duration(seconds=0.01) < endtime:
                err = [ (d-c) for d,c in zip(desired,last) ]
                velocity = [ abs(x / (self.rate * (endtime - self.node.get_clock().now()).to_sec())) for x in err ]
                self.node.get_logger().debug(err)
                for i in range(len(self.joints)):
                    if err[i] > 0.001 or err[i] < -0.001:
                        cmd = err[i] 
                        top = velocity[i]
                        if cmd > top:
                            cmd = top
                        elif cmd < -top:
                            cmd = -top
                        last[i] += cmd
                        self.device.joints[self.joints[i]].time_move_started = self.node.get_clock().now()
                        self.device.joints[self.joints[i]].in_trajectory = True
                        self.device.joints[self.joints[i]].is_moving = True
                        self.device.joints[self.joints[i]].desired = last[i]
                    else:
                        self.device.joints[self.joints[i]].speed = 0.0
                        self.device.joints[self.joints[i]].in_trajectory = True
                        self.device.joints[self.joints[i]].is_moving = False
                        velocity[i] = 0
                r.sleep()
        return True

    def active(self):
        """ Is controller overriding servo internal control? """
        return self.server.is_active() or self.executing

    def getDiagnostics(self):
        """ Get a diagnostics status. """
        msg = DiagnosticStatus()
        msg.name = self.name
        msg.level = DiagnosticStatus.OK
        msg.message = "OK"
        if self.active():
            msg.values.append(KeyValue("State", "Active"))
        else:
            msg.values.append(KeyValue("State", "Not Active"))
        return msg
