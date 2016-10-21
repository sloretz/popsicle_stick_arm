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

AccelStepper stepper_1(AccelStepper::HALF4WIRE, 9, 11, 10, 12); //D, B, C, A
AccelStepper stepper_2(AccelStepper::HALF4WIRE, 7, 5, 6, 4); // A, C, B, D
MessageParser messager;

//const unsigned int STEPS_PER_REVOLUTION = 4076;
typedef enum {
    POSITION_MODE,
    SPEED_MODE,
} RunMode;

RunMode current_mode = POSITION_MODE;

void setup()
{
    Serial.begin(115200);
}

unsigned long last_command_time = 0;
const unsigned long command_period = 100;

void applyMessage(AccelStepper &motor, const MessageParser::Message &msg)
{
    float vel;
    switch(msg.type)
    {
        case MessageParser::MOVE_RELATIVE:
            motor.setMaxSpeed(msg.move_relative.speed);
            motor.setAcceleration(msg.move_relative.acceleration);
            motor.move(msg.move_relative.steps);
            current_mode = POSITION_MODE;
            break;
        case MessageParser::MOVE_ABSOLUTE:
            motor.setMaxSpeed(msg.move_absolute.speed);
            motor.setAcceleration(msg.move_absolute.acceleration);
            motor.moveTo(msg.move_absolute.position);
            current_mode = POSITION_MODE;
            break;
        case MessageParser::MOVE_VELOCITY:
            vel = fabs(msg.move_velocity.speed);
            motor.setMaxSpeed(vel);
            motor.setSpeed(vel);
            motor.setAcceleration(msg.move_velocity.acceleration);
            current_mode = SPEED_MODE;
            break;
        default:
            Serial.println("Unknown message");
            break;
    }
    last_command_time = millis();
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

    if (current_mode == POSITION_MODE)
    {
        stepper_1.run();
        stepper_2.run();
    }
    else if (current_mode == SPEED_MODE)
    {
        if (last_command_time + command_period > millis())
        {
            stepper_1.runSpeed();
            stepper_2.runSpeed();
        }
        else
        {
            //Only run if the command is recent
            stepper_1.stop();
            stepper_2.stop();
        }
    }

    static unsigned long last_feedback_time = 0;
    static int m1_last_speed = 0;
    static int m2_last_speed = 0;
    const unsigned long feedback_period = 50;
    unsigned long now = millis();
    if (last_feedback_time + feedback_period <= now)
    {
        int m1_new_speed = stepper_1.speed();
        int m2_new_speed = stepper_2.speed();
        float delta_t = now-last_feedback_time;
        delta_t /= 1000.0;
        messager.sendFeedback(MessageParser::MOTOR_1, stepper_1.currentPosition(), m1_new_speed, (m1_new_speed - m1_last_speed)/delta_t);
        messager.sendFeedback(MessageParser::MOTOR_2, stepper_2.currentPosition(), m2_new_speed, (m2_new_speed - m2_last_speed)/delta_t);
        m1_last_speed = m1_new_speed;
        m2_last_speed = m2_new_speed;
        last_feedback_time = now;

        //Serial.print("mode:");
        //Serial.println(current_mode == POSITION_MODE ? "position" : "speed");
    }
}
