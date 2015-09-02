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
from popsicle_arm.msg import JointTarget
from popsicle_arm.fake_serial_motors import FakeMotorInterface
from popsicle_arm.motor_message import MotorMessage
import serial #pyserial
import traceback
import select
import Queue
import math


STEPS_PER_REVOLUTION=4096.0


class MotorInterface(object):
    def __init__(self):
        self._pub = rospy.Publisher('/popsicle_arm/joint_state', JointState, queue_size=10)
        if rospy.has_param("~fake") and rospy.get_param("~fake"):
            self._fake = True
            self._dev = FakeMotorInterface(2)
        else:
            self._fake = False
            self._dev = serial.Serial(port='/dev/ttyUSB0', baudrate=115200, timeout=0, writeTimeout=0)
        self.motors = ['motor_1_to_plastic_1', 'motor_2_to_plastic_2']
        self._write_Q = Queue.Queue()
        self._feedback = dict()

    def update(self):
        while mi.readOnce():
            pass
        while mi.writeOnce():
            pass
    
    def readOnce(self):
        #Get feedback from the motors
        try:
            msg = MotorMessage.parse(self._dev.readline())
        except serial.SerialTimeoutException:
            return False
        
        if msg:
            if msg.msgtype == MotorMessage.FEEDBACK:
                self._feedback[self.motors[msg.motor-1]] = msg
        
            if len(self._feedback) == len(self.motors):
                pubmsg = JointState()
                #put all joints into same joint state message
                for name in self.motors:
                    msg = self._feedback[name]
                    pubmsg.name.append(name)
                    pubmsg.position.append(-(2.0*math.pi) * msg.position / STEPS_PER_REVOLUTION)
                    pubmsg.velocity.append(-(2.0*math.pi) * msg.velocity / STEPS_PER_REVOLUTION)
                    self._pub.publish(pubmsg)
                self._feedback = dict()
            return True
        return False
    
    def writeOnce(self):
        try:
            msg = self._write_Q.get_nowait()
            if not self._fake:
                msg = str(msg)
            self._dev.write(msg)
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
    
    def moveVelocity(self, motor, velocity, acceleration):
        msg = MotorMessage.moveVelocity(motor, velocity, acceleration)
        self._write_Q.put(msg)


class JointNudger(object):
    VELOCITY = 300
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


class JointTargetMover(object):
    """Class that moves motors based on target speeds and accelerations"""
    def __init__(self, motor_interface):
        self._mi = motor_interface
        self._motors = {'motor_1_to_plastic_1': 1,
                'motor_2_to_plastic_2': 2}
    
    def subscriberCallback(self, msg):
        #convert radians to steps
        for joint_name, position, velocity, acceleration in zip(msg.joint_names, 
                msg.positions, msg.velocities, msg.accelerations):
            position = -1.0 * STEPS_PER_REVOLUTION * position / (2*math.pi)
            velocity = 1.0 * STEPS_PER_REVOLUTION * velocity / (2*math.pi)
            acceleration = STEPS_PER_REVOLUTION * acceleration / (2*math.pi)
            self._mi.moveAbsolute(self._motors[joint_name], position,
                velocity, acceleration)



if __name__ == '__main__':
    import code
    rospy.init_node('popsicle_arm_node')
    mi = MotorInterface()
    jn = JointNudger(mi)
    jtm = JointTargetMover(mi)
    
    rospy.Subscriber('/popsicle_arm/nudge_joint', NudgeJoint, jn.subscriberCallback)
    rospy.Subscriber('/popsicle_arm/joint_target', JointTarget, jtm.subscriberCallback)
    
    rate = rospy.Rate(100) #Hz
    try:
        while not rospy.is_shutdown():
            mi.update()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
