#!/usr/bin/env python

# ROS Node - Action Server - IoT ROS Bridge
import threading
import rospy
import actionlib
import json
from pkg_ros_iot_bridge.msg import msgRosIotAction  # Message Class used by ROS Actions internally
from pkg_ros_iot_bridge.msg import msgRosIotGoal    # Message Class used for Goal Messages
from pkg_ros_iot_bridge.msg import msgRosIotResult  # Message Class used for Result Messages
from pkg_ros_iot_bridge.msg import msgRosIotFeedback # Message Class used for Feedback Messages    
from pkg_ros_iot_bridge.msg import msgMqttSub        # Message Class for MQTT Subscription Messages
from pyiot import iot                                # Custom Python Module to perfrom MQTT Tasks
                                                                     
class IotRosBridgeActionServer:

    # Constructor
    def __init__(self):

        
        # Initialize the Action Server
        self._as = actionlib.ActionServer('/action_ros_iot',
                                          msgRosIotAction,
                                          self.on_goal,
                                          self.on_cancel,
                                          auto_start=False)

       

        # Read and Store IoT Configuration data from Parameter Server
        param_config_iot = rospy.get_param('config_iot')
        self._config_mqtt_server_url = param_config_iot['mqtt']['server_url']
        self._config_mqtt_server_port = param_config_iot['mqtt']['server_port']
        self._config_mqtt_sub_topic = param_config_iot['mqtt']['topic_sub']
        self._config_mqtt_pub_topic = param_config_iot['mqtt']['topic_pub']
        self._config_mqtt_qos = param_config_iot['mqtt']['qos']
        self._config_mqtt_sub_cb_ros_topic = param_config_iot['mqtt']['sub_cb_ros_topic']
        self._config_mqtt_google_webapp_id = param_config_iot['google_apps']['spread_sheet_id']
        print param_config_iot

        #publishing messages to /ros_iot_bridge/mqtt/sub
        self._handle_ros_pub = rospy.Publisher(self._config_mqtt_sub_cb_ros_topic, msgMqttSub,
                                               queue_size=10)

        # self.mqtt_sub_callback() function will be called when there is a message from MQTT Subscription.
        ret = iot.mqtt_subscribe_thread_start(self.mqtt_sub_callback, self._config_mqtt_server_url, self._config_mqtt_server_port, self._config_mqtt_sub_topic, self._config_mqtt_qos)

        if ret == 0:
            rospy.loginfo("MQTT Subscribe Thread Started")
        else:
            rospy.logerr("Failed to start MQTT Subscribe Thread")

        # Start the Action Server
        self._as.start()
        rospy.loginfo("Started ROS-IoT Bridge Action Server.")

    
    # This is a callback function for MQTT Subscriptions
    def mqtt_sub_callback(self, client, userdata, message):
        payload = str(message.payload.decode("utf-8"))
    
        print("[MQTT SUB CB] Message: ", payload)
        print("[MQTT SUB CB] Topic: ", message.topic)

        msg_mqtt_sub = msgMqttSub()
        msg_mqtt_sub.timestamp = rospy.Time.now()
        msg_mqtt_sub.topic = message.topic
        msg_mqtt_sub.message = payload
        
        self._handle_ros_pub.publish(msg_mqtt_sub)
    
    
    # This function will be called when Action Server receives a Goal
    def on_goal(self, goal_handle):
        goal = goal_handle.get_goal()

        rospy.loginfo("Received new goal from Client")
        rospy.loginfo(goal)

        # Validate incoming goal parameters
        if(goal.protocol == "mqtt") or (goal.protocol == "http"):
            
            if((goal.mode == "pub") or (goal.mode == "sub") or (goal.mode == "push")):
                goal_handle.set_accepted()
                
                # Start a new thread to process new goal from the client 
                thread = threading.Thread(name="worker", target=self.process_goal,
                                          args=(goal_handle,))
                thread.start()

            else:
                goal_handle.set_rejected()
                return
        
        else:
            goal_handle.set_rejected()
            return
    
    # This function is called is a separate thread to process Goal.
    def process_goal(self, goal_handle):
        
        result = msgRosIotResult()

        goal_id = goal_handle.get_goal_id()
        rospy.loginfo("Processing goal : " + str(goal_id.id))

        goal = goal_handle.get_goal()
        
        # Goal Processing
        if goal.protocol == "mqtt":
            result.flag_success == True
            rospy.logwarn("MQTT")

            if goal.mode == "pub":
                rospy.logwarn("MQTT PUB Goal ID: " + str(goal_id.id))

                rospy.logwarn(goal.topic + " > " + goal.message)

                ret = iot.mqtt_publish(self._config_mqtt_server_url, 
                                       self._config_mqtt_server_port,
                                       goal.topic, 
                                       goal.message, 
                                       self._config_mqtt_qos)

                if ret == 0:
                    rospy.loginfo("MQTT Publish Successful.")
                    result.flag_success = True
                    goal_handle.set_succeeded(result)
                else:
                    rospy.logerr("MQTT Failed to Publish")
                   
            elif goal.mode == "sub":
                rospy.logwarn("MQTT SUB Goal ID: " + str(goal_id.id))
                rospy.logwarn(goal.topic)

                ret = iot.mqtt_subscribe_thread_start(self.mqtt_sub_callback, 
                                                      self._config_mqtt_server_url, 
                                                      self._config_mqtt_server_port, 
                                                      goal.topic, 
                                                      self._config_mqtt_qos)
                if ret == 0:
                    rospy.loginfo("MQTT Subscribe Thread Started")
                    result.flag_success = True
                    goal_handle.set_succeeded(result)
                else:
                    rospy.logerr("Failed to start MQTT Subscribe Thread")
                    
        # handle google sheets requests    
        if goal.protocol == "http" and goal.mode == "push":
            result.flag_success == True

            rospy.logwarn("Pushing results to google sheets with goal ID"+str(goal_id.id))
            
            res = goal.message.split(" ")

            iot.push_to_google_sheets(res)
            
            #mes0 = goal.message.replace("\n", "")
            #mes1 = mes0.split(",")
            
            goal_handle.set_succeeded(result)

        rospy.loginfo("Send goal result to client")
        rospy.loginfo("Goal ID: " + str(goal_id.id) + " Goal Processing Done.")

        if goal.protocol == "http" and goal.mode == "push_orders":
            result.flag_success == True

            res1 = json.loads(data.message)
            res2 = list(res1.values())

            if res2[5] == "Medicine":

             
              res2.append("HP")
              res2.append(450)

            if res2[5] == "Food":

            
               res2.append("MP")
               res2.append(250)

            if res2[5] == "Clothes":
  
               res2.append("LP")
               res2.append(150)
            
            iot.push_to_google_sheets_incoming_order(res2)
        

    # This function will be called when Goal Cancel request is send to the Action Server
    def on_cancel(self, goal_handle):
        rospy.loginfo("Received cancel request.")
        goal_id = goal_handle.get_goal_id()

# Main
def main():

    #initialising the node
    rospy.init_node('node_iot_ros_bridge_action_server')

    #creating the instance of class
    action_server = IotRosBridgeActionServer()

    #loop forever
    rospy.spin()

if __name__ == '__main__':
    main()
