import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import tf2_ros
import tf2_msgs.msg
import std_msgs.msg
from std_msgs.msg import String
import math
from hrwros_gazebo.msg import LogicalCameraImage  #to use logical camera feed
from pkg_vb_sim.srv import vacuumGripper  #to activate vaccum gripper
from pkg_vb_sim.srv import conveyorBeltPowerMsg  #to activate conveyor belt


class CartesianPath:

    #Constructor
    def __init__(self):

        rospy.init_node('process_packagen1', anonymous=True)

        self._planning_group = "ur5_1_planning_group"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._tfBuffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tfBuffer)
        self.box_name = "packagen1"

        rospy.loginfo('\033[94m' + " >>> Init done." + '\033[0m')

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

    def func_tf(self, arg_frame_1, arg_frame_2):
        try:
            trans = self._tfBuffer.lookup_transform(arg_frame_1, arg_frame_2, rospy.Time())

            #creating a list to add all the transform points
            res = []
            res.append(trans.transform.translation.x)
            res.append(trans.transform.translation.y)
            res.append(trans.transform.translation.z)

            return res
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("TF error")

    #functions for planning scene

    def add_box(self,timeout=4):
    
        box_name = self.box_name
        scene = self._scene
        
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose.position.x = 0.29
        box_pose.pose.position.y = -0.45
        box_pose.pose.position.z = 1.41
        scene.add_box(box_name, box_pose, size=(0.15, 0.15, 0.15))

    def attach_box(self, timeout=4):
    
        box_name = self.box_name
        robot = self._robot
        scene = self._scene
        eef_link = self._eef_link
        group_names = self._group_names
        grasping_group = self._group_names    
        touch_links = robot.get_link_names(group="ur5_1_planning_group")
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
            '\033[94m' + "Object of class CartesianPath Deleted." + '\033[0m')

class Services:

    def activate_conveyor_belt(self,power):

        rospy.wait_for_service("/eyrc/vb/conveyor/set_power")
        try:
            s = rospy.ServiceProxy("/eyrc/vb/conveyor/set_power",conveyorBeltPowerMsg)
            result = s(power)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    #activating the vaccum gripper by calling a service
    def activate_vacuum_gripper(self,result):

        rospy.wait_for_service("/eyrc/vb/ur5_1/activate_vacuum_gripper")
        try:
            s = rospy.ServiceProxy('/eyrc/vb/ur5_1/activate_vacuum_gripper', vacuumGripper)
            result = s(result)
            
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

def main():
        
    ur5 = CartesianPath()
    service = Services()
    
    reference_frame = "world"
    target_frame = "ur5_wrist_3_link"

    box_length = 0.15               # Length of the Package
    vacuum_gripper_width = 0.115    # Vacuum Gripper Width
    delta = vacuum_gripper_width + (box_length/2)  # 0.19
    # Teams may use this info in Tasks

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

    go_to_pose_packagen22 = [math.radians(-159.17403431),
                          math.radians(-96.766141291),
                          math.radians(88.5505579158),
                          math.radians(8.17732613835),
 
                          math.radians(23.5726159102),
                          math.radians( 0.0788962387792)]

 #packagen20 pick
    ur5.set_joint_angles(go_to_pose_packagen20)
    activate_vacuum_gripper(True)
    rospy.sleep(0.05)
    ur5.ee_cartesian_translation(0,25,0)
    ur5.set_joint_angles(lst_home_pose)
    activate_vacuum_gripper(False)
        

    rospy.sleep(2)

        #packagen30 pick
    ur5.set_joint_angles(go_to_pose_packagen30)
    activate_vacuum_gripper(True)
    ur5.ee_cartesian_translation(0,30,0)
    rospy.sleep(0.5)
    ur5.set_joint_angles(lst_home_pose)
    activate_vacuum_gripper(False)

    rospy.sleep(2)

if __name__ == '__main__':
    main()

    
    