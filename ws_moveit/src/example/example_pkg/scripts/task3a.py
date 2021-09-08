#!/usr/bin/env python
import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import tf2_ros
import tf2_msgs.msg
import std_msgs


from std_msgs.msg import String
from hrwros_gazebo.msg import LogicalCameraImage  #to use logical camera feed
from pkg_vb_sim.srv import vacuumGripper  #to activate vaccum gripper
from pkg_vb_sim.srv import conveyorBeltPowerMsg  #to activate conveyor belt


class CartesianPath:

    #Constructor
    def __init__(self):

       
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

    def go_to_pose(self, arg_pose):

        pose_values = self._group.get_current_pose().pose
        #rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        #rospy.loginfo(pose_values)

        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move

        pose_values = self._group.get_current_pose().pose
        #rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        #rospy.loginfo(pose_values)

        list_joint_values = self._group.get_current_joint_values()
        #rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        #rospy.loginfo(list_joint_values)

        if (flag_plan == True):
            rospy.loginfo('\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan

    def go_to_predefined_pose(self, arg_pose_name):
        rospy.loginfo('\033[94m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
        self._group.set_named_target(arg_pose_name)
        plan = self._group.plan()
        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        goal.trajectory = plan
        self._exectute_trajectory_client.send_goal(goal)
        self._exectute_trajectory_client.wait_for_result()
        rospy.loginfo('\033[94m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m')


    def func_tf(self, arg_frame_1, arg_frame_2):
        try:
            trans = self._tfBuffer.lookup_transform(arg_frame_1, arg_frame_2, rospy.Time())

            

            #creating a list to add all the transform points
            res = []
            res.append(trans.transform.translation.x)
            res.append(trans.transform.translation.y)
            res.append(trans.transform.translation.z)

            print(res)
            return res

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("TF error")

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



    ur5_2_home_pose = geometry_msgs.msg.Pose()
    ur5_2_home_pose.position.x = -0.8
    ur5_2_home_pose.position.y = 0.0
    ur5_2_home_pose.position.z = 1 + vacuum_gripper_width + (box_length/2)
    # This to keep EE parallel to Ground Plane
    ur5_2_home_pose.orientation.x = -0.5
    ur5_2_home_pose.orientation.y = -0.5
    ur5_2_home_pose.orientation.z = 0.5
    ur5_2_home_pose.orientation.w = 0.5


    ur5_2_packagen2_pose = geometry_msgs.msg.Pose()
    ur5_2_packagen2_pose.position.x = 0.817
    ur5_2_packagen2_pose.position.y = 0.10
    ur5_2_packagen2_pose.position.z = 0.944



    
  
   
  


        

        

            
            
    print("found the model")

    service.activate_conveyor_belt(5)
    ur5.go_to_pose(ur5_2_home_pose)
                                                                
    res = ur5.func_tf(reference_frame, target_frame)
    rospy.loginfo('\033[94m' + "Translating EE to package from current position." + '\033[0m')
                                                            
    ur5.ee_cartesian_translation(res[0]+1.0,0,-0.19)
    service.activate_vacuum_gripper(True)
                                
    service.activate_conveyor_belt(20)

                
    ur5.ee_cartesian_translation(0.817,0.10,0.944)
                
    service.activate_vacuum_gripper(False)
                                
                                    
    ur5.go_to_pose(ur5_2_home_pose)
    rospy.signal_shutdown("process done")

       
                

if __name__ == '__main__':

    rospy.init_node("hello")

    def callback(msg):

        
        if msg.models[0].type == "ur5" and msg.models[1].type == "packagen2" and msg.models[0].pose.position.y == 0.0:
            
            rospy.sleep(1)

       
            main()



    rospy.Subscriber("/eyrc/vb/logical_camera_2",LogicalCameraImage,callback)
    rospy.spin()

   


