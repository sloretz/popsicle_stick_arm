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

AccelStepper stepper_1(AccelStepper::HALF4WIRE, 8,10,9,11);
AccelStepper stepper_2(AccelStepper::HALF4WIRE, 2,4,3,5);

const unsigned int MAX_STEPS_PER_SECOND = 500;
const unsigned int STEPS_PER_REVOLUTION = 4076;
const unsigned int STEP_ACCELERATION = 100;

void setup()
{
    stepper_1.setMaxSpeed(MAX_STEPS_PER_SECOND);
    stepper_2.setMaxSpeed(MAX_STEPS_PER_SECOND);
    stepper_1.setAcceleration(STEP_ACCELERATION);
    stepper_2.setAcceleration(STEP_ACCELERATION);
    stepper_1.setSpeed(200);
    stepper_2.setSpeed(200);
    stepper_1.moveTo(500);
    stepper_2.moveTo(500);
    Serial.begin(115200);
}

void loop()
{
    if (stepper_1.distanceToGo() == 0)
    {
    stepper_1.moveTo(-stepper_1.currentPosition());
    }
    if (stepper_2.distanceToGo() == 0)
    {
    stepper_2.moveTo(-stepper_2.currentPosition());
    }
    stepper_1.run();
    stepper_2.run();
}
