#!/usr/bin/env python
import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import gazebo

import geometry_msgs.msg
import actionlib
import tf2_ros
import tf2_msgs.msg
from task3 import CartesianPath
from task3 import Services
from hrwros_gazebo.msg import LogicalCameraImage

def main():

    rospy.init_node("Hello")

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

  


    def callback(msg):



        if msg.models[0].type == "ur5" and msg.models[1].type == "packagen2":

            

            task3.Services.activate_conveyor_belt(service,3)
                                                    
            res = task3.CartesianPath.func_tf(ur5,reference_frame, target_frame)
            rospy.loginfo('\033[94m' + "Translating EE to package from current position." + '\033[0m')
                                            
            task3.CartesianPath.ee_cartesian_translation(ur5,res[1],0,0)
            task3.Services.activate_vacuum_gripper(service,True)
                
            task3.Services.activate_conveyor_belt(service,20)

            task3.CartesianPath.ee_cartesian_translation(ur5,0.7,0,0)
            task3.Services.activate_vacuum_gripper(False)
                
                    
            task3.CartesianPath.go_to_pose(ur5,ur5_2_home_pose)
            rospy.signal_shutdown("process is done")

    rospy.Subscriber("/eyrc/vb/logical_camera_2",LogicalCameraImage,callback)
    rospy.spin()            

if __name__ == '__main__':
    main()




