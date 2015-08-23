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

#ifndef POSITION_HI_HPP
#define POSITION_HI_HPP

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

class PopsicleArmPositionHI : public hardware_interface::RobotHW
{
    public:
        PopsicleArmPositionHI();
        ~PopsicleArmPositionHI();
        
        void update_state(int joint, double position, double velocity, double effort);
        double get_command(int joint);
    
    private:
        hardware_interface::JointStateInterface jnt_state_interface;
        hardware_interface::PositionJointInterface jnt_pos_interface;
        double cmd[2];
        double pos[2];
        double vel[2];
        double eff[2];
};

#endif
