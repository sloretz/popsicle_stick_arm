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


"""Node for manually moving the joints on the robot arm using a keyboard.
"""

import rospy
import time
from popsicle_arm.msg import NudgeJoint


class RobotArmNudger(object):
    def __init__(self, joint_name, publisher):
        self._pub = publisher
        self._joint = joint_name
    
    def nudgeClockwise(self):
        """Nudges the arm clockwise. Nudges further the more often this method is called
        """
        amount = -0.05
        self._nudgeJoint(amount)
    
    def nudgeCounterClockwise(self):
        """Nudges the arm clockwise. Nudges further the more often this method is called
        """
        amount = 0.05
        self._nudgeJoint(amount)
    
    def _nudgeJoint(self, amount):
        msg = NudgeJoint()
        msg.joint_name = self._joint
        msg.amount = amount
        self._pub.publish(msg)


if __name__ == '__main__':
    
    rospy.init_node('joint_teleop_node')
    pub = rospy.Publisher('/popsicle_arm/nudge_joint', NudgeJoint, queue_size=5)
    ran1 = RobotArmNudger('motor_1_to_plastic_1', pub)
    ran2 = RobotArmNudger('motor_2_to_plastic_2', pub)
    
    
    def getch():
        #http://stackoverflow.com/questions/510357/python-read-a-single-character-from-the-user/510364#510364
        import sys, tty, termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch
    
    print "Controls: j1: q,w j2: o,p (t to quit)"
    while not rospy.is_shutdown():
        ch = getch().lower()
        if ch == 'q':
            ran1.nudgeCounterClockwise()
        elif ch == 'w':
            ran1.nudgeClockwise()
        elif ch == 'o':
            ran2.nudgeCounterClockwise()
        elif ch == 'p':
            ran2.nudgeClockwise()
        elif ch == 't':
            print "Quitting!"
            break;



