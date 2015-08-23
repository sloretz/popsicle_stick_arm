// Popsicle stick robot
// Copyright (C) 2015  Shane Loretz
// 
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include "arm_ros_control/position_hi.hpp"


PopsicleArmPositionHI::PopsicleArmPositionHI()
{
    hardware_interface::JointStateHandle state_handle_m1("motor_1_to_plastic_1", &pos[0], &vel[0], &eff[0]);
    jnt_state_interface.registerHandle(state_handle_m1);
    hardware_interface::JointStateHandle state_handle_m2("motor_2_to_plastic_2", &pos[1], &vel[1], &eff[1]);
    jnt_state_interface.registerHandle(state_handle_m2);
    registerInterface(&jnt_state_interface);

    hardware_interface::JointHandle pos_handle_m1(jnt_state_interface.getHandle("motor_1_to_plastic_1"), &cmd[0]);
    jnt_pos_interface.registerHandle(pos_handle_m1);
    hardware_interface::JointHandle pos_handle_m2(jnt_state_interface.getHandle("motor_2_to_plastic_2"), &cmd[1]);
    jnt_pos_interface.registerHandle(pos_handle_m2);
    registerInterface(&jnt_pos_interface);
}

PopsicleArmPositionHI::~PopsicleArmPositionHI()
{
}

void PopsicleArmPositionHI::update_state(int joint, double position, double velocity, double effort)
{
    //TODO is effort different from acceleration?
    pos[joint] = position;
    vel[joint] = velocity;
    eff[joint] = effort;
}

double PopsicleArmPositionHI::get_command(int joint)
{
    //todo bounds checking
    return cmd[joint];
}
