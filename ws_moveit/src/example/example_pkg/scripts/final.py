#! /usr/bin/env python

import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math
import rosservice
from pkg_vb_sim.srv import vacuumGripper


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
        self.box_name = 'packagen00'


        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    #go to given joint values
    def set_joint_angles(self, arg_list_joint_angles):

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        self._group.set_joint_value_target(arg_list_joint_angles)
        self._group.plan()
        flag_plan = self._group.go(wait=True)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan
    #go to predefined pose 
    def go_to_predefined_pose(self, arg_pose_name):
        rospy.loginfo('\033[94m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
        self._group.set_named_target(arg_pose_name)
        plan = self._group.plan()
        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        goal.trajectory = plan
        self._exectute_trajectory_client.send_goal(goal)
        self._exectute_trajectory_client.wait_for_result()
        rospy.loginfo('\033[94m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m')

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

    def add_box(self, timeout=4):
    
        box_name = self.box_name
        scene = self._scene
        
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose.position.x=  -0.26
        box_pose.pose.position.y= -0.45
        box_pose.pose.position.z= 1.91
        scene.add_box(box_name, box_pose, size=(0.15, 0.15, 0.15))

    def attach_box(self, timeout=4):
    
        box_name = self.box_name
        robot = self._robot
        scene = self._scene
        eef_link = self._eef_link
        group_names = self._group_names
        grasping_group = self._group_names    
        touch_links = robot.get_link_names(group = "manipulator")
        scene.attach_box(self._eef_link, box_name,touch_links=touch_links)

    def detach_box(self, timeout=4):
    
        box_name = self.box_name
        scene = self._scene
        eef_link = self._eef_link
        scene.remove_attached_object(eef_link, name=box_name)

    def remove_box(self, timeout=4):
   
        box_name = self.box_name
        scene = self._scene
        scene.remove_world_object(box_name)


    
    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


def main():
    
    #activating the vaccum gripper by calling a service
    def activate_vacuum_gripper(result):

        rospy.wait_for_service("/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1")
        try:
            s=rospy.ServiceProxy("/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1",vacuumGripper)
               
            result = s(result)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


    ur5 = Ur5Moveit()

    #attaching box to the planning screen
    

    ur5_2_home_pose = geometry_msgs.msg.Pose()
    ur5_2_home_pose.position.x = -0.8
    ur5_2_home_pose.position.y = 0.0
    ur5_2_home_pose.position.z = 1.19
    # This to keep EE parallel to Ground Plane
    ur5_2_home_pose.orientation.x = -0.5
    ur5_2_home_pose.orientation.y = -0.5
    ur5_2_home_pose.orientation.z = 0.5
    ur5_2_home_pose.orientation.w = 0.5

    lst_joint_angles_homepose = [math.radians(7.84642573914),
                          math.radians(-139.934864592),
                          math.radians (-58.27588025),
                          math.radians (-71.8459202532),
                          math.radians(90.0018929695),
                          math.radians( 7.82696229335)]

    lst_joint_angles_packagen00 = [math.radians(-56.447387),
                          math.radians(-75.6653413),
                          math.radians (16.0108),
                          math.radians (-108.905),
                          math.radians(-105.561996),
                          math.radians(26.3767469)]

    lst_joint_angles_packagen012 = [math.radians(-164.7769),
                          math.radians(-41.3004244),
                          math.radians (-23.7363),
                          math.radians (153.24131),
                          math.radians(-9.2317807),
                          math.radians(-109.378170)]


    lst_joint_angles_packagen02 = [math.radians(73.5445220714),
                          math.radians(-104.144457977),
                          math.radians(-7.14937919),
                          math.radians(-68.8000644),
                          math.radians(81.9998315616),
                          math.radians(-179.77173994)]

    go_to_pose_packagen12 = [math.radians(55.4029451495),
                          math.radians(-85.8893316073),
                          math.radians(-82.3285254182),
                          math.radians(-8.575581),
                          math.radians(124.55727),
                          math.radians(1.77630224393)]

    push_packagen12 = [math.radians(136.212063291),
                          math.radians(-90.1018718),
                          math.radians(50.1513826),
                          math.radians(43.729127),
                          math.radians(-43.85003104),
                          math.radians(177.235722332)]

    go_to_pose_packagen11 = [math.radians(-115.708781802),
                          math.radians(-102.68120),
                          math.radians(57.5895058),
                          math.radians(42.181883),
                          math.radians(64.3058200),
                          math.radians(-178.7811)]

    push_packagen11 = [math.radians(65.0055197),
                          math.radians(15.5177135),
                          math.radians(-95.419628),
                          math.radians(-66.18283444),
                          math.radians(114.89522289),
                          math.radians(1.18205633206)]

    go_to_pose_packagen10 = [math.radians(-58.679607),
                          math.radians(-95.60067648),
                          math.radians(83.69978386),
                          math.radians(-171.8818031),
                          math.radians(-121.246780),
                          math.radians(-1.9997)]

    push_packagen10 = [math.radians(-131.508549235),
                          math.radians(-145.98286759),
                          math.radians(60.9678766047),
                          math.radians(-71.9610095965),
                          math.radians(-50.8663801255),
                          
                          math.radians(-15.041878575)]

    go_to_pose_packagen20 = [math.radians(-54.3268126815),
                          math.radians(-96.37609067552),
                          math.radians(81.91620049067552),
                          math.radians(14.46398399),
                          math.radians(124.884660925),
                          math.radians(0.002313864)]

    push_packagen20 = [math.radians(-122.51536),
                          math.radians(-98.748036),
                          math.radians(-66.44081),
                          math.radians(165.18129),
                          math.radians(56.69731984),
                          math.radians(0.00819289)]

    

    lst_joint_angles_packagen01 = [math.radians(107.64492134),
                          math.radians(-102.89620351),
                          math.radians (2.8311536),
                          math.radians (-85.1181331),
                          math.radians(76.094295),

    
                          math.radians(-164.96778355)]
    

    lst_joint_angles_reach = [math.radians(-20.55346),
                          math.radians(-165.0076128),
                          math.radians (-9.00420),
                          math.radians (-95.947766),
                          math.radians(90.0001110),
                          math.radians(-20.508508)]







    
    


    

    while not rospy.is_shutdown():

        #ur5.add_box()
       
        ur5.go_to_pose(ur5_2_home_pose)
        #to go the package
        #ur5.set_joint_angles(lst_joint_angles_packagen02)
        #activate_vacuum_gripper(True)
        #ur5.add_box()
        #ur5.attach_box()
        #ur5.set_joint_angles(push_packagen10)
        #ur5.set_joint_angles(lst_joint_angles_reach)
        #ur5.detach_box()

        #activate_vacuum_gripper(False)
        
        
        
        

    #del ur5


if __name__ == '__main__':
    main()

