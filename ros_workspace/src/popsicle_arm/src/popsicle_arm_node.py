#!/usr/bin/env python
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
from sensor_msgs.msg import JointState
from popsicle_arm.msg import NudgeJoint
import re
import serial #pyserial
import traceback
import select


class MotorMessage(object):
    TERMINATOR = '\n'
    FEEDBACK = 'F'
    _feedback_re = re.compile("^(?P<motor>[0-9])F(?P<position>([-+])?[0-9]+),(?P<velocity>([-+])?[0-9]+),(?P<acceleration>([-+])?[0-9]+)$")
    
    def __init__(self, motor, msgtype, **msgparams):
        #Put message params directly onto object
        for k,v in msgparams.items():
            setattr(self, k, v)
        
        self.motor = motor
        self.msgtype = msgtype
    
    @classmethod
    def parse(cls, raw):
        #Should only be feedback message
        #1F123,456,789 motorFposition,vel,accel
        obj = re.match(cls._feedback_re, raw)
        if obj:
            return cls(int(obj.group('motor')),
                    cls.FEEDBACK,
                    position=int(obj.group('position')),
                    velocity=int(obj.group('velocity')),
                    acceleration=int(obj.group('acceleration'))
                    )
        return None



class MotorInterface(object):
    def __init__(self):
        self._pub = rospy.Publisher('joint_state', JointState, queue_size=10)
        self._dev = serial.Serial(port='/dev/ttyUSB0', baudrate=115200)
        self.motors = ['motor_1_to_plastic_1', 'motor_2_to_plastic_2']
        
    def readOnce(self):
        #Get feedback from the motors, or send a motor command?
        msg = MotorMessage.parse(self._dev.readline())
        
        if msg:
            #take action
            if msg.msgtype == MotorMessage.FEEDBACK:
                print "Got feedback", msg.motor, msg.position, msg.velocity, msg.acceleration
                pass
        
    #TODO methods for sending motor commands from nudge joint messages

if __name__ == '__main__':
    rospy.init_node('popsicle_arm_node')
    mi = MotorInterface()
    
    rate = rospy.Rate(100) #Hz
    try:
        while not rospy.is_shutdown():
            mi.readOnce()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
