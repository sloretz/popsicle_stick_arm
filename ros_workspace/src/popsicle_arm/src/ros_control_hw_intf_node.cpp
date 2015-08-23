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

#include <ros/ros.h>
#include "popsicle_arm/position_hi.hpp"
#include "popsicle_arm/JointTarget.h"
#include <sensor_msgs/JointState.h>
#include <controller_manager/controller_manager.h>

const std::string joint_1 = "motor_1_to_plastic_1";
const std::string joint_2 = "motor_2_to_plastic_2";
const double VELOCITY = 0.8; // rad/s
const double ACCELERATION = 1.0;// rad/s^2



class ArmUpdater
{
    public:
        ArmUpdater()
        {
            command_pub_ = nh_.advertise<popsicle_arm::JointTarget>("/popsicle_arm/joint_target", 1000);
            state_sub_ = nh_.subscribe("/popsicle_arm/joint_state", 1, &ArmUpdater::joint_state_callback, this);
            controller_manager_.reset(new controller_manager::ControllerManager(&arm_hw_intf, nh_));
            ros::Duration update_period( 1.0 / 10.0 );
            timer_ = nh_.createTimer( update_period, &ArmUpdater::update_arm, this );
        }
        ~ArmUpdater()
        {
        }
        
        PopsicleArmPositionHI arm_hw_intf;
        boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
        ros::Publisher command_pub_;
        ros::Subscriber state_sub_;
        ros::NodeHandle nh_;
        ros::Timer timer_;
        
        void update_arm(const ros::TimerEvent &e)
        {
            //ROS_INFO("updating arm");
            controller_manager_->update(ros::Time::now(), e.current_real - e.last_real);
            
            //TODO just use joint state msg for target msg
            popsicle_arm::JointTarget out_msg;
            out_msg.joint_names.push_back(joint_1);
            out_msg.positions.push_back(arm_hw_intf.get_command(0));
            out_msg.velocities.push_back(VELOCITY);
            out_msg.accelerations.push_back(ACCELERATION);
            out_msg.joint_names.push_back(joint_2);
            out_msg.positions.push_back(arm_hw_intf.get_command(1));
            out_msg.velocities.push_back(VELOCITY);
            out_msg.accelerations.push_back(ACCELERATION);
            command_pub_.publish(out_msg);
            //ROS_INFO("After publishing commands"); 
        }
        
        void joint_state_callback(const sensor_msgs::JointState::ConstPtr& msg)
        {
            //ROS_INFO("Got state");
            //Populate the joint values
            for (int j = 0; j < msg->name.size(); j++)
            {
                std::string name = msg->name[j];
                double pos = msg->position[j];
                double vel = msg->velocity[j];
                double eff = 0.0;
                if (joint_1 == name)
                {
                    //motor 1
                    arm_hw_intf.update_state(0, pos, vel, eff);
                }
                else if (joint_2 == name)
                {
                    //motor 2
                    arm_hw_intf.update_state(1, pos, vel, eff);
                }
                else
                {
                    ROS_WARN("Unknown joint name %s", msg->name[j].c_str());
                }
            }
            //ROS_INFO("Finished processing state");
        }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_controller_node");
    ArmUpdater au;
    ros::MultiThreadedSpinner spinner(4); // Use 4 threads
    spinner.spin(); // spin() will not return until the node has been shutdown
    return 0;
}
