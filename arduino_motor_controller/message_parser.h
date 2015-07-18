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

#ifndef MESSAGE_PARSER_HPP
#define MESSAGE_PARSER_HPP

#include <Arduino.h>
#include <stdint.h>
#include "variable_array.h"

/*
 * Class parses motor controller messages on 
 */

class MessageParser
{
    
    protected:
    
    struct MoveAbsoluteMsg{
        int16_t position;
        int16_t speed;
        int16_t acceleration;
    };
    
    struct MoveRelativeMsg{
        int16_t steps;
        int16_t speed;
        int16_t acceleration;
    };
    
    struct MotorStateMsg{
        int16_t position;
        int16_t speed;
        int16_t acceleration;
    };
    
    public:
    
    typedef enum {
        MOVE_ABSOLUTE = 2,
        MOVE_RELATIVE = 3,
        MOTOR_FEEDBACK = 4
    }MessageType;
    
    typedef enum {
        MOTOR_1 = 1,
        MOTOR_2 = 2
    }MotorNum;
    
    typedef struct {
        MessageType type;
        MotorNum motor;
        union
        {
            MoveAbsoluteMsg move_absolute;
            MoveRelativeMsg move_relative;
            MotorStateMsg motor_state;
        };
    } Message;
    
    MessageParser();
    ~MessageParser();
    
    //Parses message from Serial1 and puts it into referenced message
    //returns true iff a message is parsed
    bool getMessage(Message &msg);
    
    void sendFeedback(MotorNum, int position, int speed, int acceleration);
    
    protected:
    VariableArray<char> buffer_;
    
    bool parseMsg(Message &msg);
    int parseInt(int &index);
    
};

#endif
