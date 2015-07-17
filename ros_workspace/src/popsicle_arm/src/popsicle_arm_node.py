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
import Queue
import math


STEPS_PER_REVOLUTION=4076


class MotorMessage(object):
    TERMINATOR = '\n'
    FEEDBACK = 'F'
    MOVE_RELATIVE = 'R'
    MOVE_ABSOLUTE = 'A'
    _feedback_re = re.compile("^(?P<motor>[0-9])F(?P<position>([-+])?[0-9]+),(?P<velocity>([-+])?[0-9]+),(?P<acceleration>([-+])?[0-9]+)$")
    
    def __init__(self, raw, motor, msgtype, **msgparams):
        #Put message params directly onto object
        for k,v in msgparams.items():
            setattr(self, k, v)
        
        self.motor = motor
        self.msgtype = msgtype
        self.raw = raw
    
    @classmethod
    def parse(cls, raw):
        #Should only be feedback message
        #1F123,456,789 motorFposition,vel,accel
        obj = re.match(cls._feedback_re, raw)
        if obj:
            return cls(raw,
                    int(obj.group('motor')),
                    cls.FEEDBACK,
                    position=int(obj.group('position')),
                    velocity=int(obj.group('velocity')),
                    acceleration=int(obj.group('acceleration'))
                    )
        return None
    
    def __str__(self):
        return self.raw
    
    @classmethod
    def moveRelative(cls, motor, steps, velocity, acceleration):
        motor = int(motor)
        steps = int(steps)
        velocity = int(velocity)
        acceleration = int(acceleration)
        raw = "{motor}R{steps},{velocity},{acceleration}\n".format(
                motor=motor, steps=steps, velocity=velocity,
                acceleration=acceleration)
        return cls(raw, motor, cls.MOVE_RELATIVE, steps=steps,
                velocity=velocity, acceleration=acceleration)
    
    @classmethod
    def moveAbsolute(cls, motor, position, velocity, acceleration):
        motor = int(motor)
        position = int(position)
        velocity = int(velocity)
        acceleration = int(acceleration)
        raw = "{motor}A{position},{velocity},{acceleration}\n".format(
                motor=motor, position=position, velocity=velocity,
                acceleration=acceleration)
        return cls(raw, motor, cls.MOVE_RELATIVE, position=position, 
                velocity=velocity, acceleration=acceleration)


class MotorInterface(object):
    def __init__(self):
        self._pub = rospy.Publisher('joint_state', JointState, queue_size=10)
        self._dev = serial.Serial(port='/dev/ttyUSB0', baudrate=115200,
                timeout=0, writeTimeout=0)
        self.motors = ['motor_1_to_plastic_1', 'motor_2_to_plastic_2']
        self._write_Q = Queue.Queue()
        
    def readOnce(self):
        #Get feedback from the motors, or send a motor command?
        try:
            msg = MotorMessage.parse(self._dev.readline())
        except serial.SerialTimeoutException:
            return False
        
        if msg:
            #take action
            if msg.msgtype == MotorMessage.FEEDBACK:
                print "Got feedback", msg.motor, msg.position, msg.velocity, msg.acceleration
            return True
        return False
    
    def writeOnce(self):
        try:
            msg = self._write_Q.get_nowait()
            self._dev.write(str(msg))
            return True
        except serial.SerialTimeoutException:
            pass
        except Queue.Empty:
            pass
        return False
    
    def moveRelative(self, motor, steps, velocity, acceleration):
        msg = MotorMessage.moveRelative(motor, steps, velocity, acceleration)
        self._write_Q.put(msg)
    
    def moveAbsolute(self, motor, position, velocity, acceleration):
        msg = MotorMessage.moveAbsolute(motor, position, velocity, acceleration)
        self._write_Q.put(msg)


class JointNudger(object):
    VELOCITY = 100
    ACCELERATION = 200
    
    def __init__(self, motor_interface):
        self._mi = motor_interface
        self._motors = {'motor_1_to_plastic_1': 1,
                'motor_2_to_plastic_2': 2}
    
    def subscriberCallback(self, msg):
        #convert radians to steps
        #and do a move relative
        steps = STEPS_PER_REVOLUTION * msg.amount / (2*math.pi)
        self._mi.moveRelative(self._motors[msg.joint_name], steps,
                self.VELOCITY, self.ACCELERATION)



if __name__ == '__main__':
    import code
    rospy.init_node('popsicle_arm_node')
    mi = MotorInterface()
    jn = JointNudger(mi)
    
    rospy.Subscriber('/popsicle_arm/nudge_joint/', NudgeJoint, jn.subscriberCallback)
    
    rate = rospy.Rate(100) #Hz
    try:
        while not rospy.is_shutdown():
            #TODO do joint state publishing,
            #and NudgeJoint support
            while mi.readOnce():
                pass
            while mi.writeOnce():
                pass
            #code.interact(local=locals())
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
