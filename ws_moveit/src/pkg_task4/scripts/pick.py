#! /usr/bin/env python

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import rospkg

import yaml
import os
import math
import time
import sys
import copy

from std_srvs.srv import Empty

import rosservice
from pkg_vb_sim.srv import vacuumGripper


class Ur5Moveit:

    # Constructor
    def __init__(self, arg_robot_name):

        rospy.init_node('play_ur5_1', anonymous=True)

        self._robot_ns = '/'  + 'ur5_1'
        self._planning_group = "manipulator"
        
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander(robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._scene = moveit_commander.PlanningSceneInterface(ns=self._robot_ns)
        self._group = moveit_commander.MoveGroupCommander(self._planning_group, robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._display_trajectory_publisher = rospy.Publisher( self._robot_ns + '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        self._exectute_trajectory_client = actionlib.SimpleActionClient( self._robot_ns + '/execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self.box_name = 'package'
        # Attribute to store computed trajectory by the planner	
        self._computed_plan = ''
        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()
        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rp = rospkg.RosPack()
        self._pkg_path = rp.get_path('pkg_task4')
        self._file_path = self._pkg_path + '/config/saved_trajectories_ur5_1/'
        rospy.loginfo( "Package Path: {}".format(self._file_path) )

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def clear_octomap(self):
        clear_octomap_service_proxy = rospy.ServiceProxy(self._robot_ns + "/clear_octomap", Empty)
        return clear_octomap_service_proxy()

    def set_joint_angles(self, arg_list_joint_angles):

        list_joint_values = self._group.get_current_joint_values()
        self._group.set_joint_value_target(arg_list_joint_angles)
        self._computed_plan = self._group.plan()
        flag_plan = self._group.go(wait=True)
        list_joint_values = self._group.get_current_joint_values()
        pose_values = self._group.get_current_pose().pose
        if (flag_plan == True):
            pass
           
        else:
            pass
           
        return flag_plan

    def hard_set_joint_angles(self, arg_list_joint_angles, arg_max_attempts):

        number_attempts = 0
        flag_success = False
        
        while ( (number_attempts <= arg_max_attempts) and  (flag_success is False) ):
            number_attempts += 1
            flag_success = self.set_joint_angles(arg_list_joint_angles)
            rospy.logwarn("attempts: {}".format(number_attempts) )
           

    def ee_cartesian_translation(self, trans_x, trans_y, trans_z):
        # 1. Create a empty list to hold waypoints
        waypoints = []
        # 2. Add Current Pose to the list of waypoints
        waypoints.append(self._group.get_current_pose().pose)
        # 3. Create a New waypoint
        wpose = geometry_msgs.msg.Pose()
        wpose.position.x = waypoints[0].position.x + (trans_x)
        wpose.position.y = waypoints[0].position.y + (trans_y)
        wpose.position.z = waypoints[0].position.z + (trans_z)
        # This to keep EE parallel to Ground Plane
        wpose.orientation.x = -0.5
        wpose.orientation.y = -0.5
        wpose.orientation.z = 0.5
        wpose.orientation.w = 0.5
        # 4. Add the new waypoint to the list of waypoints
        waypoints.append(copy.deepcopy(wpose))
        # 5. Compute Cartesian Path connecting the waypoints in the list of waypoints
        (plan, fraction) = self._group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # Step Size, distance between two adjacent computed waypoints will be 1 cm
            0.0)         # Jump Threshold
        rospy.loginfo("Path computed successfully. Moving the arm.")
        num_pts = len(plan.joint_trajectory.points)
        if (num_pts >= 3):
            del plan.joint_trajectory.points[0]
            del plan.joint_trajectory.points[1]

        # 6. Make the arm follow the Computed Cartesian Path
        self._group.execute(plan)


    def moveit_play_planned_path_from_file(self, arg_file_path, arg_file_name):
        file_path = arg_file_path + arg_file_name
        
        with open(file_path, 'r') as file_open:
            loaded_plan = yaml.load(file_open)
        
        ret = self._group.execute(loaded_plan)
        # rospy.logerr(ret)
        return ret

    
    def moveit_hard_play_planned_path_from_file(self, arg_file_path, arg_file_name, arg_max_attempts):
        number_attempts = 0
        flag_success = False

        while ( (number_attempts <= arg_max_attempts) and (flag_success is False) ):
            number_attempts += 1
            flag_success = self.moveit_play_planned_path_from_file(arg_file_path, arg_file_name)
            rospy.logwarn("attempts: {}".format(number_attempts) )
            # # self.clear_octomap()
        
        return True

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')

def main():

    def activate_vacuum_gripper(result):

        rospy.wait_for_service("/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1")
        try:
            s=rospy.ServiceProxy("/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1",vacuumGripper)
               
            result = s(result)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    lst_home_pose = [math.radians(7.8433489314),
                    math.radians(-139.942784159),
                    math.radians(-58.2911345789),   
                    math.radians (-71.7204516851),
                    math.radians (89.9713177297),
                    math.radians (7.90736427846)]

    go_to_pose_packagen20 = [math.radians(-54.3268126815),
                          math.radians(-96.37609067552),
                          math.radians(81.91620049067552),
                          math.radians(14.46398399),
                          math.radians(124.884660925),
                          math.radians(0.002313864)]

    go_to_pose_packagen30 = [math.radians(-65.456772158),
                          math.radians(-100.159754377),
                          math.radians(117.078639371),
                          math.radians(-16.9851453469),
                          math.radians(100.369630363),
                          math.radians(-0.0270974077971)]

    go_to_pose_packagen22 = [math.radians(-159.17403431),
                          math.radians(-96.766141291),
                          math.radians(88.5505579158),
                          math.radians(8.17732613835),
 
                          math.radians(23.5726159102),
                          math.radians( 0.0788962387792)]

    ur5 = Ur5Moveit(sys.argv[0])
    

    while not rospy.is_shutdown():

       

        
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'zero_to_packagen20.yaml', 5)
        activate_vacuum_gripper(True)
        ur5.ee_cartesian_translation(0,25,0)
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'packagen20_to_zero.yaml', 5)
        activate_vacuum_gripper(False)

        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'zero_to_packagen30.yaml', 5)
        activate_vacuum_gripper(True)
        ur5.ee_cartesian_translation(0,35,0)
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'packagen30_to_zero.yaml', 5)
        activate_vacuum_gripper(False)


        rospy.signal_shutdown("picking done")

    del ur5

if __name__ == '__main__':
    main()

