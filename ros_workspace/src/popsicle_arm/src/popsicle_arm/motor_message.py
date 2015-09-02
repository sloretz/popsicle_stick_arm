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
import re


class MotorMessage(object):
    TERMINATOR = '\n'
    FEEDBACK = 'F'
    MOVE_RELATIVE = 'R'
    MOVE_ABSOLUTE = 'A'
    MOVE_VELOCITY = 'T'
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
    
    def __repr__(self):
        return self.raw
    
    def __len__(self):
        return len(self.raw)
    
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
        return cls(raw, motor, cls.MOVE_ABSOLUTE, position=position, 
                velocity=velocity, acceleration=acceleration)
    
    @classmethod
    def moveVelocity(cls, motor, velocity, acceleration):
        """Tell the motor to move up to a target velocity"""
        motor = int(motor)
        velocity = int(velocity)
        acceleration = int(acceleration)
        raw = "{motor}T{velocity},{acceleration}\n".format(
                motor=motor, velocity=velocity,
                acceleration=acceleration)
        return cls(raw, motor, cls.MOVE_VELOCITY, velocity=velocity, 
                acceleration=acceleration)
    
    @classmethod
    def feedback(cls, motor, position, velocity, acceleration):
        """Generate a fake feedback message"""
        motor = int(motor)
        position = int(position)
        velocity = int(velocity)
        acceleration = int(acceleration)
        raw = "{motor}F{position},{velocity},{acceleration}\n".format(
                motor=motor, position=position, velocity=velocity,
                acceleration=acceleration)
        return cls(raw,
                motor,
                cls.FEEDBACK,
                position=position,
                velocity=velocity,
                acceleration=acceleration)
