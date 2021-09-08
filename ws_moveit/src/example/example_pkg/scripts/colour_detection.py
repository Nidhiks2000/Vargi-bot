#!/usr/bin/env python

import rospy
import cv2
import sys
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


class Camera1:

  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/eyrc/vb/camera_1/image_raw", Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      rospy.logerr(e)

    (rows,cols,channels) = cv_image.shape
    
    image = cv_image

    # Resize a 720x1280 image to 360x640 to fit it on the screen
    resized_image = cv2.resize(image, (720/2, 1280/2)) 

    green = np.uint8([[[0, 255, 0]]])  #green color
    hsvGreen = cv2.cvtColor(green, cv2.COLOR_BGR2HSV) #hsv value of green color 
    

    lowerLimit = hsvGreen[0][0][0] - 10, 100, 100  # range of green color lower limit and upper limit
    upperLimit = hsvGreen[0][0][0] + 10, 255, 255

    
    red = np.uint8([[[0, 0, 255]]]) #red color
    hsvred = cv2.cvtColor(red, cv2.COLOR_BGR2HSV) #hsv value of red color
    

    lower = hsvred[0][0][0] - 10, 100, 100 # range of red color lower limit and upper limit
    upper = hsvred[0][0][0] + 10, 255, 255

    
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) #convert the image into hsv

    lg = np.array(lowerLimit) #range of green color
    ug = np.array(upperLimit)

    green_mask = cv2.inRange(hsv, lg, ug)
    
    cv2.imshow('green_packages', green_mask) #show the image 

    lr = np.array(lower) #range of red color
    ur = np.array(upper)

    red_mask = cv2.inRange(hsv, lr, ur) #red masked image
    cv2.imshow('red_packages', red_mask)  #show the image 


    original = image.copy()
    
    image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower = np.array([22, 93, 0], dtype="uint8") #yellow colour lower range and upper range
    upper = np.array([45, 255, 255], dtype="uint8")
    mask = cv2.inRange(image, lower, upper)

    cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]

    for c in cnts:
      x,y,w,h = cv2.boundingRect(c)
      cv2.rectangle(original, (x, y), (x + w, y + h), (36,255,12), 2)
      cv2.putText(image, "Blue Colour", (x, y), 
              cv2.FONT_HERSHEY_SIMPLEX, 
              1.0, (255, 0, 0)) 

    cv2.imshow('yellow_packages', mask)
    cv2.imshow("/eyrc/vb/camera_1/image_raw", resized_image)
    cv2.waitKey(3)


def main(args):
  
  rospy.init_node('node_eg1_read_camera', anonymous=True)

  ic = Camera1()
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    rospy.loginfo("Shutting down")
  
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
