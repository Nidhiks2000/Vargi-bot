#!/usr/bin/env python

import rospy
import cv2
import sys
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from pyzbar.pyzbar import decode

class Camera1:

  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/eyrc/vb/camera_1/image_raw", Image,self.callback)

  
  def get_qr_data(self, arg_image):
    qr_result = decode(arg_image)

    if (qr_result[0].rect.left == 129) and (qr.result[0].rect.top == 643) and (qr_result[0].rect.width == 89) and (qr_result[0].rect.height == 90):
      print("packagen02 = ",qr_result[0].data])
    elif (qr_result[0].rect.left == 130) and (qr.result[0].rect.top == 796) and (qr_result[0].rect.width == 90) and (qr_result[0].rect.height == 90):
      print("packagen00 = ",qr_result[1].data])
    elif (qr_result[0].rect.left == 501) and (qr.result[0].rect.top == 796) and (qr_result[0].rect.width == 91) and (qr_result[0].rect.height == 91):
      print("packagen10 = ",qr_result[2].data])
    elif (qr_result[0].rect.left == 129) and (qr.result[0].rect.top == 643) and (qr_result[0].rect.width == 89) and (qr_result[0].rect.height == 90):
      print("packagen11 = ",qr_result[3].data])
    elif (qr_result[0].rect.left == 129) and (qr.result[0].rect.top == 643) and (qr_result[0].rect.width == 89) and (qr_result[0].rect.height == 90):
      print("packagen12 = ",qr_result[4].data])
    elif (qr_result[0].rect.left == 129) and (qr.result[0].rect.top == 643) and (qr_result[0].rect.width == 89) and (qr_result[0].rect.height == 90):
      print("packagen20 = ",qr_result[5].data])
    elif (qr_result[0].rect.left == 129) and (qr.result[0].rect.top == 643) and (qr_result[0].rect.width == 89) and (qr_result[0].rect.height == 90):
      print("packagen21 = ",qr_result[6].data])
    elif (qr_result[0].rect.left == 129) and (qr.result[0].rect.top == 643) and (qr_result[0].rect.width == 89) and (qr_result[0].rect.height == 90):
      print("packagen22 = ",qr_result[7].data])
    elif (qr_result[0].rect.left == 502) and (qr.result[0].rect.top == 315) and (qr_result[0].rect.width == 90) and (qr_result[0].rect.height == 91):
      print("packagen30 = ",qr_result[8].data])
    elif (qr_result[0].rect.left == 318) and (qr.result[0].rect.top == 797) and (qr_result[0].rect.width == 89) and (qr_result[0].rect.height == 89):
      print("packagen31 = ",qr_result[9].data])
    elif (qr_result[0].rect.left == 128) and (qr.result[0].rect.top == 496) and (qr_result[0].rect.width == 90) and (qr_result[0].rect.height == 91):
      print("packagen32 = ",qr_result[10].data])
    
    
    
    


  
    



    


  
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
    
    rospy.loginfo(self.get_qr_data(image))
    
    cv2.waitKey(3)


def main(args):
  
  rospy.init_node('node_eg3_qr_decode', anonymous=True)

  ic = Camera1()
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    rospy.loginfo("Shutting down")
  
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
