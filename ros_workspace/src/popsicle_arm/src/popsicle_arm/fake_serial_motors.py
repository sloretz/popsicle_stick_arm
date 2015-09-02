# Popsicle stick robot
# Copyright (C) 2015  Shane Loretz
# 
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import rospy
from popsicle_arm.motor_message import MotorMessage
import time
import math


class FakeMotor(object):
    """Fakes position, velocity, and acceleration data for visualization in rviz without having to hook up the real arm
    """
    def __init__(self):
        self.last_update_time = None
        self.position = 0.0
        self.velocity = 0.0
        self.acceleration = 0.0
        self.command = None
    
    def get_update(self):
        new_update_time = time.time()
        if self.command is not None:
            if self.last_update_time is None:
                self.last_update_time = new_update_time
            else:
                delta_t = new_update_time - self.last_update_time
                self.last_update_time = new_update_time
                #Determine target velocity starting this period
                if self.command['type'] == 'move_position':
                    #target velocity gets smaller as goal gets closer
                    distance_to_goal = self.command['target_pos'] - self.position
                    ideal_velocity = math.sqrt(2.0 * self.command['acceleration'] * abs(distance_to_goal))
                    target_velocity = min(self.command['velocity'], ideal_velocity)
                    
                    #determine acceleration using starting and ending velocity last period
                    actual_acceleration = (target_velocity - self.velocity) / delta_t
                    #determine distance moved
                    movement = 0.5 * actual_acceleration * delta_t * delta_t
                    rospy.loginfo("Position: %f, movement %f, actual acceleration %f, target_velocity %f", self.position, movement, actual_acceleration, target_velocity)
                    
                    if distance_to_goal > 0:
                        self.position += movement
                    else:
                        self.position -= movement
                    self.velocity = target_velocity
                    self.acceleration = actual_acceleration
                else:
                    rospy.logwarn("Unknown motor command %r", self.command)
        return int(self.position), int(self.velocity), int(self.acceleration)
    
    def moveRelative(self, steps, velocity, acceleration):
        self.command = {'type': 'move_position', 'target_pos': self.position + steps, 'velocity': abs(float(velocity)), 'acceleration': float(acceleration)}
    
    def moveAbsolute(self, position, velocity, acceleration):
        rospy.loginfo("Absolute command pos %d, vel %d, accel %d", position, velocity, acceleration)
        self.command = {'type': 'move_position', 'target_pos': float(position), 'velocity': abs(float(velocity)), 'acceleration': float(acceleration)}
    
    def moveVelocity(self, velocity, acceleration):
        self.command = {'type': 'move_velocity', 'velocity': float(velocity), 'acceleration': float(acceleration)}


class FakeMotorInterface(object):
    """Class behaves like a serial device, but the motors
    aren't real
    """
    def __init__(self, num_motors):
        self.fake_motors = list()
        for i in range(num_motors):
            self.fake_motors.append(FakeMotor())
        self._feedback_num = 0
        self._last_update_time = time.time()
    
    def readline(self):
        """Returns feedback if enough time has elapsed
        """
        delta_t = time.time() - self._last_update_time
        if delta_t >=  1.0/20.0:
            position, velocity, acceleration = self.fake_motors[self._feedback_num].get_update()
            msg = MotorMessage.feedback(self._feedback_num+1, position, velocity, acceleration)
            
            self._feedback_num += 1
            if self._feedback_num >= len(self.fake_motors):
                #gotten feedback from all motors, go back to waiting
                self._last_update_time = time.time()
                self._feedback_num = 0
            
            return str(msg)
        return ""
    
    def write(self, msg):
        if msg.msgtype == MotorMessage.MOVE_RELATIVE:
            self.fake_motors[msg.motor-1].moveRelative(msg.steps, msg.velocity, msg.acceleration)
        elif msg.msgtype == MotorMessage.MOVE_ABSOLUTE:
            self.fake_motors[msg.motor-1].moveAbsolute(msg.position, msg.velocity, msg.acceleration)
        elif msg.msgtype == MotorMessage.MOVE_VELOCITY:
            self.fake_motors[msg.motor-1].moveVelocity(msg.velocity, msg.acceleration)
