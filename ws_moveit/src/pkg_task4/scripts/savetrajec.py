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

import rosservice
from pkg_vb_sim.srv import vacuumGripper

from std_srvs.srv import Empty

class Ur5Moveit:

    # Constructor
    def __init__(self, arg_robot_name):

        rospy.init_node('node_moveit_eg6', anonymous=True)

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

    

    def add_box(self, timeout=4):
    
        box_name = self.box_name
        scene = self._scene
        
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose.position.x= -0.02
        box_pose.pose.position.y= -0.42
        box_pose.pose.position.z= 1.88
        
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

    def activate_vacuum_gripper(result):

        rospy.wait_for_service("/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1")

        try:
            s=rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1',vacuumGripper)
               
            result = s(result)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    ur5 = Ur5Moveit(sys.argv[0])

    lst_joint_angles_packagen02 = [math.radians(73.5445220714),
                          math.radians(-104.144457977),
                          math.radians(-7.14937919),
                          math.radians(-68.8000644),
                          math.radians(81.9998315616),
                          math.radians(-179.77173994)]

    lst_joint_angles_packagen00 = [math.radians(-56.447387),
                          math.radians(-75.6653413),
                          math.radians (16.0108),
                          math.radians (-108.905),
                          math.radians(-105.561996),
                          math.radians(26.3767469)]

    lst_joint_angles_packagen01 = [math.radians(107.64492134),
                          math.radians(-102.89620),
                          math.radians (2.83115),
                          math.radians (-85.1181331),
                          math.radians(76.09429),
                          math.radians(-164.96778335)]
   
    lst_joint_angles_homepose = [math.radians(7.84642573914),
                          math.radians(-139.934864592),
                          math.radians (-58.27588025),
                          math.radians (-71.8459202532),
                          math.radians(90.0018929695),
                          math.radians( 7.82696229335)]

    go_to_pose_packagen12 = [math.radians(55.4029451495),
                          math.radians(-85.8893316073),
                          math.radians(-82.3285254182),
                          math.radians(-8.575581),
                          math.radians(124.55727),
                          math.radians(1.77630224393)]

    go_to_pose_packagen11 = [math.radians(-115.708781802),
                          math.radians(-102.68120),
                          math.radians(57.5895058),
                          math.radians(42.181883),
                          math.radians(64.3058200),
                          math.radians(-178.7811)]

    go_to_pose_packagen10 = [math.radians(-58.679607),
                          math.radians(-95.60067648),
                          math.radians(83.69978386),
                          math.radians(-171.8818031),
                          math.radians(-121.246780),
                          math.radians(-1.9997)]

    go_to_pose_packagen20 = [math.radians(-54.3268126815),
                          math.radians(-96.37609067552),
                          math.radians(81.91620049067552),
                          math.radians(14.46398399),
                          math.radians(124.884660925),
                          math.radians(0.002313864)]

    push_packagen20 = [math.radians(-39.0305570771),
                          math.radians(-101.1785),
                          math.radians(125.86187),
                          math.radians(150.97325),
                          math.radians(-130.029187),
                          math.radians(63.98421924)]

    push_packagen12 = [math.radians(136.212063291),
                          math.radians(-90.1018718),
                          math.radians(50.1513826),
                          math.radians(43.729127),
                          math.radians(-43.85003104),
                          math.radians(177.235722332)]

    go_to_pose_packagen21 = [math.radians(127.95260),
                          math.radians(-54.977861),
                          math.radians(-124.4956229),
                          math.radians(-0.5373557),
                          math.radians(52.8076089),
                          math.radians(170.69291)]

    push_packagen21 = [math.radians(-65.3653918),
                          math.radians(-179.214691269),
                          math.radians(117.355851388),
                          math.radians(-118.13495),
                          math.radians(-113.86972),
                          math.radians(170.6889038)]

    

    lst_joint_angles_reach = [math.radians(-20.55346),
                          math.radians(-165.0076128),
                          math.radians (-9.00420),
                          math.radians (-95.947766),
                          math.radians(90.0001110),
                          math.radians(-20.508508)]

    lst_joint_angles_redbin = [math.radians(83.553465715),
                          math.radians(-34.70828499),
                          math.radians (-32.5385204),
                          math.radians (-50.582340),
                          math.radians(-80.61970908),
                          math.radians(163.300184637)]

    lst_joint_angles_yellowbin = [math.radians(-8.30940),
                          math.radians(-44.53558),
                          math.radians (-4.77929),
                          math.radians (-40.82826),
                          math.radians(-90.97929),
                          math.radians(81.71730)]

    lst_joint_angles_greenbin = [math.radians(-93.820091),
                          math.radians(-72.39555),
                          math.radians (41.897387),
                          math.radians (-60.46697),
                          math.radians(-89.948869),
                          math.radians(-16.4130)]

    go_to_pose_packagen22 = [math.radians( -160.384109323),
                          math.radians(-89.3079681314),
                          math.radians(107.878749987),
                          math.radians(161.434837185),
 
                          math.radians(-12.7961297535),
                          math.radians(-100.515004874)]

    go_to_pose_packagen30 = [math.radians(-65.456772158),
                          math.radians(-100.159754377),
                          math.radians(117.078639371),
                          math.radians(-16.9851453469),
                          math.radians(100.369630363),
                          math.radians(-0.0270974077971)]



  


    # 1. Save AllZeros to Pose#1 Trajectory stored in "_computed_plan" attribute in a File
    # The "_computed_plan" attribute is set in method hard_set_joint_angles()
   

    #zero to packagen
    ur5.hard_set_joint_angles(go_to_pose_packagen22, 50)
    file_name = 'zero_to_packagen22.yaml'
    file_path = ur5._file_path + file_name
    
    with open(file_path, 'w') as file_save:
        yaml.dump(ur5._computed_plan, file_save, default_flow_style=True)
    
    rospy.loginfo( "File saved at: {}".format(file_path) )

    rospy.sleep(0.2)
    ur5.ee_cartesian_translation(0,100,0)

    ur5.hard_set_joint_angles(lst_joint_angles_homepose, 50)
    file_name = 'packagen22_to_zero.yaml'
    file_path = ur5._file_path + file_name
    
    with open(file_path, 'w') as file_save:
        yaml.dump(ur5._computed_plan, file_save, default_flow_style=True)
    
    rospy.loginfo( "File saved at: {}".format(file_path) )

   
    
    

    
    
    

    
   
    

    



if __name__ == '__main__':
    main()

