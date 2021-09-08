#! /usr/bin/env python

import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math


class Ur5Moveit:

    # Constructor
    def __init__(self):

        rospy.init_node('node_eg3_set_joint_angles', anonymous=True)

        self._robot_ns = '/'  + "ur5_1"
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
        self.box_name = 'packagen1'


        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def go_to_pose(self, arg_pose):

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


def main():

    ur5 = Ur5Moveit()


    ur5_2_home_pose = geometry_msgs.msg.Pose()
    ur5_2_home_pose.position.x = -0.8
    ur5_2_home_pose.position.y = 0.0
    ur5_2_home_pose.position.z = 1.19
    # This to keep EE parallel to Ground Plane
    ur5_2_home_pose.orientation.x = -0.5
    ur5_2_home_pose.orientation.y = -0.5
    ur5_2_home_pose.orientation.z = 0.5
    ur5_2_home_pose.orientation.w = 0.5

    ur5_1_bin_pose = geometry_msgs.msg.Pose()
    ur5_1_bin_pose.position.x = 0.0278622697832
    ur5_1_bin_pose.position.y = 0.799518958978
    ur5_1_bin_pose.position.z = 1.18997053774
    ur5_1_bin_pose.orientation.x = -0.707020548495
    ur5_1_bin_pose.orientation.y = 0.0127553692576
    ur5_1_bin_pose.orientation.z = -0.0123208155081
    ur5_1_bin_pose.orientation.w = 0.706970609054

    ur5_pose_1 = geometry_msgs.msg.Pose()
    ur5_pose_1.position.x = -7.67076563146e-06
    ur5_pose_1.position.y = 0.416441513974 
    ur5_pose_1.position.z =  1.19004725387
    ur5_pose_1.orientation.x = -1.57079630711
    ur5_pose_1.orientation.y = 2.56716853331e-09
    ur5_pose_1.orientation.z = 1.57079632678
    ur5_pose_1.orientation.w = 0.5

    ur5_pose_2 = geometry_msgs.msg.Pose()
    ur5_pose_2.position.x = 0.517188193775
    ur5_pose_2.position.y =  0.000356788486
    ur5_pose_2.position.z =  0.994456061846
    ur5_pose_2.orientation.x = 3.14159265244
    ur5_pose_2.orientation.y = -9.81340784475e-05
    ur5_pose_2.orientation.z =  -3.14110970089
    ur5_pose_2.orientation.w = 0.5

    ur5_pose_3 = geometry_msgs.msg.Pose()
    ur5_pose_3.position.x = -2.95847754653e-05



    ur5_pose_3.position.y = -0.41334668668682
    ur5_pose_3.position.z =  1.19001885622
    ur5_pose_3.orientation.x = -1.570794646808
    ur5_pose_3.orientation.y = -5.84644161972e-08
    ur5_pose_3.orientation.z =1.5707963268
    ur5_pose_3.orientation.w =   0.5

    ur5_pose_5 = geometry_msgs.msg.Pose()
    ur5_pose_5.position.x = 0.002648
    ur5_pose_5.position.y = -0.2501
    ur5_pose_5.position.z =  1.917855
    ur5_pose_5.orientation.x = 0.000796
    ur5_pose_5.orientation.y = 0.9999999
    ur5_pose_5.orientation.z = 0.00024155
    ur5_pose_5.orientation.w = -3.049716


    ur5_pose_6 = geometry_msgs.msg.Pose()
    ur5_pose_6.position.x = 0.0066819962314  
    ur5_pose_6.position.y = 0.244867529747
    ur5_pose_6.position.z = 1.89486492323
    ur5_pose_6.orientation.x = 5.19707730382e-05
    ur5_pose_6.orientation.y = -0.0175198826474
    ur5_pose_6.orientation.z = -9.00214763909e-06
    ur5_pose_6.orientation.w = 0.999846513686
    
    while not rospy.is_shutdown():
        ur5.go_to_pose(ur5_pose_6)
        rospy.sleep(2)
        #ur5.go_to_pose(ur5_2_home_pose)
        #rospy.sleep(2)
        #ur5.go_to_pose(ur5_pose_3)
        #rospy.sleep(2)

    


if __name__ == '__main__':
    main()
