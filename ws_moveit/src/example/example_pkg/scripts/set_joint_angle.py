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
        self.box_name = 'packagen1'


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
        box_pose.pose.position.x=  0.01
        box_pose.pose.position.y= 0.49
        box_pose.pose.position.z= 1.88
        scene.add_box(box_name, box_pose, size=(0.15, 0.15, 0.15))

    def attach_box(self, timeout=4):
    
        box_name = self.box_name
        robot = self._robot
        scene = self._scene
        eef_link = self._eef_link
        group_names = self._group_names
        grasping_group = self._group_names    
        touch_links = robot.get_link_names(group = "ur5_1_planning_group")
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

        rospy.wait_for_service("/eyrc/vb/ur5_1/activate_vacuum_gripper")
        try:
            s=rospy.ServiceProxy('/eyrc/vb/ur5_1/activate_vacuum_gripper', vacuumGripper)
               
            result = s(result)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


    ur5 = Ur5Moveit()

    #attaching box to the planning screen
    ur5.add_box()

    ur5_2_home_pose = geometry_msgs.msg.Pose()
    ur5_2_home_pose.position.x = -0.8
    ur5_2_home_pose.position.y = 0.0
    ur5_2_home_pose.position.z = 1.19
    # This to keep EE parallel to Ground Plane
    ur5_2_home_pose.orientation.x = -0.5
    ur5_2_home_pose.orientation.y = -0.5
    ur5_2_home_pose.orientation.z = 0.5
    ur5_2_home_pose.orientation.w = 0.5


    ur5_1_packagen10 = geometry_msgs.msg.Pose()
    ur5_1_packagen10.position.x=1.19026905871
    
    ur5_1_packagen10.position.y = 0.279052248601
    
    ur5_1_packagen10.position.z = 0.147499445589
     
    ur5_1_packagen10.orientation.x = -9.12192977411e-07
    ur5_1_packagen10.orientation.y = -9.1291943938e-07
    ur5_1_packagen10.orientation.z= 0.706825251234
    ur5_1_packagen10.orientation.w= 0.707388199093

     

    #joint values to go to the package
    lst_joint_angles_packagen00= [math.radians(160.00902361522047),
                          math.radians(-120.9549528739),
                          math.radians(-0.00490865213606),
                          math.radians(-45.00097016028425),
                          math.radians(5.00402124269617),
                          math.radians(0.00183613766813)]
   
    #joint values to go to the bin
    lst_joint_angles_packagen11 = [math.radians(160.00902361522047),
                          math.radians(-89.9549528739),
                          math.radians(-0.00490865213606),
                          math.radians(0.00097016028425),
                          math.radians(0.00402124269617),
                          math.radians(0.00183613766813)]
   
    lst_joint_angles_2 = [math.radians(-115.27432931),
                          math.radians(-70.4918426768),
                          math.radians(10.28683920169),
                          math.radians(-100.2096635),
                          math.radians(115.327502377),
                          math.radians(5.0303493904877)]
   
    lst_joint_angles_packagen01 = [math.radians(110.0857588764),
                          math.radians( -89.3707505846),
                          math.radians(-45.5867402481),
                          math.radians(-58.9830815994),
                          math.radians(100.031250447),
                          math.radians(-1.98454116811)]
   
    
    while not rospy.is_shutdown():
       
        #to go the package
        #ur5.set_joint_angles(lst_joint_angles_packagen11)
        
        
        
        #ur5.go_to_pose(ur5_1_packagen11)
        
        #to go to the bin
        
        #ur5.set_joint_angles(lst_joint_angles_packagen00)
        #ur5.go_to_pose(ur5_2_home_pose)

        #ur5.set_joint_angles(lst_joint_angles_packagen02)
        ur5.go_to_pose(ur5_2_home_pose) 
        #deattaching the package
        #ur5.detach_box()
        #deactivating vaccum_gripper
        
        #remove the box from the screen
        #ur5.remove_box()
        #make all the joints back to zero
        #ur5.go_to_predefined_pose("allZeroes")

    #del ur5


if __name__ == '__main__':
    main()

