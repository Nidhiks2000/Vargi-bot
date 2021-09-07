#!/usr/bin/env python

import rospy
import cv2
import sys
import actionlib
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import math
import time
import requests
from pyzbar.pyzbar import decode
from pkg_ros_iot_bridge.msg import msgMqttSub       #importing the message 
from pkg_ros_iot_bridge.msg import msgRosIotAction  # Message Class that is used by ROS Actions internally
from pkg_ros_iot_bridge.msg import msgRosIotGoal    # Message Class that is used for Goal Messages
from pkg_ros_iot_bridge.msg import msgRosIotResult 
import numpy as np

#This class is used to send goals to node_action_server_ros_iot_bridge      
class IotBridgeActionClient():
    def __init__(self):

        # Initialize Action Client
        self._ac = actionlib.ActionClient('/action_ros_iot', msgRosIotAction)
        
        # Dictionary to Store all the goal handels
        self._goal_handles = {}

        # Store the MQTT Topic on which to Publish in a variable
        param_config_iot = rospy.get_param('config_iot')
        self._config_mqtt_pub_topic = param_config_iot['mqtt']['topic_pub']

        
        
        rospy.loginfo("Action server up, we can send goals to server.")


    def on_transition(self, goal_handle):
        
        # from on_goal() to on_transition(). goal_handle generated by send_goal() is used here.
        
        result = msgRosIotResult()

        index = 0
        for i in self._goal_handles:
            if self._goal_handles[i] == goal_handle:
                index = i
                break

        rospy.loginfo("Transition Callback. Client Goal Handle #: " + str(index))
        rospy.loginfo("Comm. State: " + str(goal_handle.get_comm_state()))
        rospy.loginfo("Goal Status: " + str(goal_handle.get_goal_status()))
        
        # if (Comm State == ACTIVE)
        if goal_handle.get_comm_state() == 2:
            rospy.loginfo(str(index) + ": Goal just went active.")
        
        # if (Comm State == DONE)
        if goal_handle.get_comm_state() == 7:
            rospy.loginfo(str(index) + ": Goal is DONE")
            rospy.loginfo(goal_handle.get_terminal_state())
            
            # get_result() gets the result produced by the Action Server
            result = goal_handle.get_result()
            rospy.loginfo(result.flag_success)

            if result.flag_success == True:
                rospy.loginfo("Goal successfully completed. Client Goal Handle #: " + str(index))
            else:
                rospy.loginfo("Goal failed!!! Client Goal Handle #: " + str(index))

    # sending goals 
    def send_goal(self, arg_protocol, arg_mode, arg_topic, arg_message):
        # Create a Goal Message object
        goal = msgRosIotGoal()

        goal.protocol = arg_protocol
        goal.mode = arg_mode
        goal.topic = arg_topic
        goal.message = arg_message

        rospy.loginfo("Send goal.")
        goal_handle = self._ac.send_goal(goal, self.on_transition, None)
                                         
        return goal_handle

class Camera1:

  def __init__(self):

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/eyrc/vb/camera_1/image_raw", Image,self.callback)

  
  def get_qr_data(self, arg_image):
    qr_result = decode(arg_image)

    print("packagen 02 :", qr_result[0].data)
    print("packagen 01 : " ,qr_result[1].data)
    print("packagen 11 : ",qr_result[2].data)
    print("packagen 10 : ",qr_result[3].data)
    print("packagen 20 : ",qr_result[4].data)
    print("packagen 00 : ",qr_result[5].data)
    print("packagen 12 : ",qr_result[6].data)
    print("packagen 32 : ",qr_result[7].data)
    rospy.sleep(5)
    self.sub.unregister()
  
  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      rospy.logerr(e)

    (rows,cols,channels) = cv_image.shape
    
    image = cv_image

    # Resize a 720x1280 image to 360x640 to fit it on the screen
    resized_image = cv2.resize(image, (720/2, 1280/2)) 

    cv2.imshow("/eyrc/vb/camera_1/image_raw", resized_image)
    gray_img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    self.get_qr_data(gray_img) 
    
def main(args):
  
  rospy.init_node('node_eg3_qr_decode', anonymous=True)

  ic = Camera1()
  action_client1 = IotBridgeActionClient()

  rospy.sleep(2)

  try:
    rospy.spin()
  except KeyboardInterrupt:
    rospy.loginfo("Shutting down")
  
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
