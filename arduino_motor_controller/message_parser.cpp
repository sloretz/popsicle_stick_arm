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
#include "message_parser.h"

#include <stdlib.h>

MessageParser::MessageParser()
{
    buffer_.reserve(64);
}

MessageParser::~MessageParser()
{
}

bool MessageParser::getMessage(Message &msg)
{
    while(Serial.available())
    {
        char ch = Serial.read();
        if (ch == '\n')
        {
            bool found_msg = parseMsg(msg);
            buffer_.clear();
            return found_msg;
        }
        else
        {
            buffer_.push_back(ch);
        }
    }
}

bool MessageParser::parseMsg(Message &msg)
{
    int idx = 0;
    if (!buffer_.size())
    {
        return false;
    }
    //Precondition:: buffer_has one, and only one message in it
    //Which motor is this command for
    if (buffer_[idx] == '1')
    {
        msg.motor = MOTOR_1;
    }
    else if (buffer_[idx] == '2')
    {
        msg.motor = MOTOR_2;
    }
    else
    {
        //bad message, skip
        Serial.println("Unknown motor");
        return false;
    }
    idx++;
    if (buffer_.size() <= idx)
    {
        return false;
    }
    
    //What command is this?
    switch (buffer_[idx])
    {
        case 'A':
            msg.type = MOVE_ABSOLUTE;
            idx++;
            msg.move_absolute.position = parseInt(idx);
            msg.move_absolute.speed = parseInt(idx);
            msg.move_absolute.acceleration = parseInt(idx);
            break;
        case 'R':
            msg.type = MOVE_RELATIVE;
            idx++;
            msg.move_relative.steps = parseInt(idx);
            msg.move_relative.speed = parseInt(idx);
            msg.move_relative.acceleration = parseInt(idx);
            break;
        case 'T':
            msg.type = MOVE_VELOCITY;
            idx++;
            msg.move_velocity.speed = parseInt(idx);
            msg.move_velocity.acceleration = parseInt(idx);
            break;
        case 'F':
            msg.type = MOTOR_FEEDBACK;
            idx++;
            //Arduino sends this command, it does not receive it
            return false;
        default:
            //invalid command
            return false;
    }
    return true;
}

void MessageParser::sendFeedback(MotorNum motor, int position, int speed, int acceleration)
{
    Serial.print(motor);
    Serial.print('F');
    Serial.print(position);
    Serial.print(',');
    Serial.print(speed);
    Serial.print(',');
    Serial.print(acceleration);
    Serial.print('\n');
}

int MessageParser::parseInt(int &idx)
{
    char integer[16];
    int num = 0;
    int val = 0;
    while (idx < buffer_.size() && num < 15)
    {
        char ch = buffer_[idx];
        idx++;
        if (( ch == '-' || ch == '+' ) && num == 0)
        {
            integer[num] = ch;
            num++;
        }
        else if (ch == '0' || ch == '1' || ch == '2' || ch == '3' || ch == '4' || ch == '5' || ch == '6' || ch == '7' || ch == '8' || ch == '9')
        {
            integer[num] = ch;
            num++;
        }
        else
        {
            break;
        }
    }
    integer[num] = '\0';
    if (num)
    {
        val = atoi(integer);
    }
    return val;
}
