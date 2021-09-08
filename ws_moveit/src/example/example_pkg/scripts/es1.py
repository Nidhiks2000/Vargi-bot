#! /usr/bin/env python

import rospy
import sys
import actionlib
from task3 import Message


from hrwros_gazebo.msg import LogicalCameraImage
from pkg_vb_sim.srv import conveyorBeltPowerMsg

def main():
    
    rospy.init_node("hello")


    def func_callback_topic_my_topic(msg):
        print(msg.models[0].pose.position.y)
        if msg.models[0].pose.position.y == 0.0:
        

            if msg.models[0].type == "ur5" and msg.models[1].type == "packagen2" :
                print ("found !!!!")
        
 

    rospy.Subscriber("eyrc/vb/logical_camera_2",LogicalCameraImage,func_callback_topic_my_topic)

    rospy.spin()
    

    



if __name__ == '__main__':

    
    if Message.message.msg == "done with packagen1":

    
        main()