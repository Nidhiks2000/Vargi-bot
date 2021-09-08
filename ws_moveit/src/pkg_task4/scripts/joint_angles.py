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
        box_pose.pose.position.x= 0.07
        box_pose.pose.position.y= -0.40
        box_pose.pose.position.z= 1.46
        
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
            s=rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1',vacuumGripper)
               
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

    lst_joint_angles_packagen02 = [math.radians(73.5445220714),
                          math.radians(-104.144457977),
                          math.radians(-7.14937919),
                          math.radians(-68.8000644),
                          math.radians(81.9998315616),
                          math.radians(-179.77173994)]

    lst_joint_angles_homepose = [math.radians(7.84642573914),
                          math.radians(-139.934864592),
                          math.radians (-58.27588025),
                          math.radians (-71.8459202532),
                          math.radians(90.0018929695),
                          math.radians( 7.82696229335)]

    lst_joint_angles_packagen12 = [math.radians(-164.776948),
                          math.radians(-41.3004244),
                          math.radians (-23.73636177),
                          math.radians (153.2413197),
                          math.radians(-9.2317807),
                          math.radians(-109.378170)]


    lst_joint_angles_packagen22 = [math.radians(-116.846808641),
                          math.radians(-119.894948171),
                          math.radians (119.565003199 ),
                          math.radians (-179.672156096),
                          math.radians(-59.0369665481),
                          math.radians(179.997350924)]

    lst_joint_angles_packagen21 =[math.radians(-75.7793219118),
                          math.radians(-110.445104705),
                          math.radians(95.0290341282),
                          math.radians(-10.8846947649),
                          math.radians(141.951238221),
                          math.radians( -31.8319539389)]

    lst_joint_angles_push_packagen21 = [math.radians(-145.201771447),
                          math.radians( -138.474814122),
                          math.radians(125.364893979),
                          math.radians(-166.913055227),
                          math.radians(-30.6812117227),
                          math.radians(179.999623799)]
                         

 

 



    




    

    while not rospy.is_shutdown():
       
       
        #to go the package
        ur5.set_joint_angles(lst_joint_angles_packagen22)
        ur5.add_box()
        ur5.attach_box()
        #activate_vacuum_gripper(True)
        #ur5.set_joint_angles(lst_joint_angles_push_packagen21)
        #activate_vacuum_gripper(True)
        
        #ur5.ee_cartesian_translation(0,5,0)
        #ur5.ee_cartesian_translation(0,50,0)
        #ur5.set_joint_angles(lst_joint_angles_push_packagen21)
        #rospy.sleep(0.5)

        ur5.set_joint_angles(lst_joint_angles_homepose)
        ur5.detach_box()
        #activate_vacuum_gripper(False)

        
        
        
        
        

    #del ur5


if __name__ == '__main__':
    main()

