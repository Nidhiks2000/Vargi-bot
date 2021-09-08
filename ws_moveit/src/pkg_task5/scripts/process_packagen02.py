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
from hrwros_gazebo.msg import LogicalCameraImage  #to use logical camera feed
from pkg_vb_sim.srv import vacuumGripper  #to activate vaccum gripper
from pkg_vb_sim.srv import conveyorBeltPowerMsg  #to activate conveyor belt




class Ur5Moveit:

    # Constructor
    def __init__(self, arg_robot_name):

        rospy.init_node('process_packagen02', anonymous=True)

        self._robot_ns = '/'  + 'ur5_2'
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
        self._box_name = ''


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
        self._file_path = self._pkg_path + '/config/saved_trajectories_ur5_2/'
        rospy.loginfo( "Package Path: {}".format(self._file_path) )


        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def clear_octomap(self):
        clear_octomap_service_proxy = rospy.ServiceProxy(self._robot_ns + "/clear_octomap", Empty)
        return clear_octomap_service_proxy()

    def set_joint_angles(self, arg_list_joint_angles):

        list_joint_values = self._group.get_current_joint_values()
        # rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        # rospy.loginfo(list_joint_values)

        self._group.set_joint_value_target(arg_list_joint_angles)
        self._computed_plan = self._group.plan()
        flag_plan = self._group.go(wait=True)

        list_joint_values = self._group.get_current_joint_values()
        # rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        # rospy.loginfo(list_joint_values)

        pose_values = self._group.get_current_pose().pose
        # rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        # rospy.loginfo(pose_values)

        if (flag_plan == True):
            pass
            # rospy.loginfo(
            #     '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            pass
            # rospy.logerr(
            #     '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan

    def hard_set_joint_angles(self, arg_list_joint_angles, arg_max_attempts):

        number_attempts = 0
        flag_success = False
        
        while ( (number_attempts <= arg_max_attempts) and  (flag_success is False) ):
            number_attempts += 1
            flag_success = self.set_joint_angles(arg_list_joint_angles)
            rospy.logwarn("attempts: {}".format(number_attempts) )
            # self.clear_octomap()


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

    def update_google_sheets(self,*args):

        # defining our sheet name in the 'id' variable and the the column where we want to update the value
        parameters = {"id":"OrdersShipped","Team Id":"VB_1299","Unique Id":"MnOpqRsT","Order Id":args[0][0],"City": args[0][3],"Item":args[0][1],"Priority": args[0][6],"Shipped Quantity":args[0][2],"Latitude": args[0][4] ,"Longitude": args[0][5] ,"Cost":args[0][7],"Shipped Status and time": args[0][8],"Estimated Time of Delivery":args[0][9] ,"Shipped Status": args[0][10]} 

        URL = "https://script.google.com/macros/s/AKfycbyuc519rp6UbeLF7lE8wyPT32MlU6zkllLapdqMhzBEohqH33u_RLA/exec"
    

        response = requests.get(URL, params=parameters)

        print(response.content)
    
        
    # Destructor

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


def main():

    ur5 = Ur5Moveit(sys.argv[1])


    def activate_vacuum_gripper(result):

        rospy.wait_for_service("/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2")
        try:
            s=rospy.ServiceProxy("/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2",vacuumGripper)
               
            result = s(result)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def activate_conveyor_belt(power):

        rospy.wait_for_service("/eyrc/vb/conveyor/set_power")
        try:
            s = rospy.ServiceProxy("/eyrc/vb/conveyor/set_power",conveyorBeltPowerMsg)
            result = s(power)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    

    

    def callback(msg):

        if msg.models[1].type == "packagen02":

            activate_conveyor_belt(11)
            rospy.sleep(2.5)
            activate_conveyor_belt(0)
            activate_vacuum_gripper(True)
            activate_conveyor_belt(75)
            ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'home_to_greenbin.yaml', 1)
            activate_vacuum_gripper(False)
            ur5.moveit_hard_play_planned_path_from_file(ur5._file_path,'greenbin_to_home.yaml',1)
            activate_conveyor_belt(100)
            rospy.signal_shutdown("packagen02 is done")

    rospy.Subscriber("/eyrc/vb/logical_camera_2",LogicalCameraImage,callback)
    rospy.spin()
                   
if __name__ == '__main__':
    main()

