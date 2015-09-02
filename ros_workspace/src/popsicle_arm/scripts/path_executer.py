#!/usr/bin/env python

"""Path executer uses moveit commander to move an end effector through a path
"""

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg as moveit_msgs
import geometry_msgs.msg as geometry_msgs
import std_msgs.msg as  std_msgs

class PathExecuterNode(object):
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("path_executer_node")
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("arm_group")
        self.group.set_goal_tolerance(0.001)
        print "============ Printing robot state"
        print self.robot.get_current_state()
        print "============"
        
        rospy.Subscriber('/popsicle_arm/draw_path', std_msgs.String, self._draw_path_callback)
    
    def _draw_path_callback(self, msg):
        """Draw a path given in a msg
        """
        #TODO draw path in message
        waypoints = list()
        # start with the current pose
        waypoints.append(self.group.get_current_pose().pose)
        
        #First, move down
        wpose = geometry_msgs.Pose()
        wpose.orientation.w = 1.0
        wpose.position.x = waypoints[0].position.x
        wpose.position.y = waypoints[0].position.y
        wpose.position.z = waypoints[0].position.z - 0.1
        waypoints.append(copy.deepcopy(wpose))
        
        #second, move across
        wpose.position.y -= 0.075
        waypoints.append(copy.deepcopy(wpose))
        
        #third, up
        wpose.position.z += 0.05
        waypoints.append(copy.deepcopy(wpose))
        
        #fourth, across
        wpose.position.y += 0.075
        waypoints.append(copy.deepcopy(wpose))
        
        #fifth, back to home
        wpose.position.z = waypoints[0].position.z
        wpose.position.y = waypoints[0].position.y
        waypoints.append(copy.deepcopy(wpose))
        
        plan, fraction = self.group.compute_cartesian_path(
                             waypoints,   # waypoints to follow
                             0.005,        # eef_step
                             0.0)         # jump_threshold
        self.group.execute(plan)

if __name__ == '__main__':
    node = PathExecuterNode()
    rospy.spin()
