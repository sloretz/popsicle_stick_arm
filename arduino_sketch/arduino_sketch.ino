/*
Popsicle stick robot
Copyright (C) 2015  Shane Loretz

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "AccelStepper.h"
#include "message_parser.h"

AccelStepper stepper_1(AccelStepper::HALF4WIRE, 8,10,9,11);
AccelStepper stepper_2(AccelStepper::HALF4WIRE, 2,4,3,5);
MessageParser messager;

const unsigned int STEPS_PER_REVOLUTION = 4076;

void setup()
{
    Serial.begin(115200);
}

void applyMessage(AccelStepper &motor, const MessageParser::Message &msg)
{
    switch(msg.type)
    {
        case MessageParser::MAX_SPEED:
            motor.setMaxSpeed(msg.max_speed.speed);
            break;
        case MessageParser::MOVE_RELATIVE:
            motor.setSpeed(msg.move_relative.speed);
            motor.setAcceleration(msg.move_relative.speed);
            motor.move(msg.move_relative.steps);
            break;
        default:
            Serial.println("Unknown message");
            break;
    }
}

void loop()
{
    MessageParser::Message msg;
    if (messager.getMessage(msg))
    {
        if (msg.motor == MessageParser::MOTOR_1)
        {
            applyMessage(stepper_1, msg);
        }
        else if (msg.motor == MessageParser::MOTOR_2)
        {
            applyMessage(stepper_2, msg);
        }
    }
    stepper_1.run();
    stepper_2.run();
}
