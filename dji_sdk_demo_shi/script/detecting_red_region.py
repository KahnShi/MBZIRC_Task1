#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from geometry_msgs.msg import Twist

class image_converter:

  def __init__(self):
    #rospy.init_node('relative_pixel_position', anonymous=True)
    self.image_pub = rospy.Publisher("image_with_target",Image)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/left/image_rect_color",Image,self.callback)
    self.position_pub = rospy.Publisher('relative_pixel_position', Twist, queue_size = 10)
    
  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape

    hsv_lower_bound = np.array([101,30,165])
    hsv_upper_bound = np.array([255,255,255])
    image_hsv = cv2.cvtColor(cv_image, cv2.COLOR_RGB2HSV)
    image_mask = cv2.inRange(image_hsv, hsv_lower_bound, hsv_upper_bound)
    _, contours, _ = cv2.findContours(image_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


    large_contours = []
    center_set = []
    contour_area_set = []
    for i in range(0, len(contours)):
      if cv2.contourArea(contours[i]) > 1500:
        contour_area_set.append(cv2.contourArea(contours[i]))
        large_contours.append(contours[i])
        moments = cv2.moments(contours[i])
        center_set.append((int(moments['m10']/moments['m00']), int(moments['m01']/moments['m00'])))
        cv2.circle(cv_image, center_set[-1], 3, (255, 0, 0), 5)
  
    #print cv_image.shape[0] #720 #y_max
    #print cv_image.shape[1] #1280 #x_max

    pubTwist = Twist()
    if len(large_contours) == 0:
      pubTwist.linear.x = 0
      pubTwist.linear.y = 0
      pubTwist.linear.z = 0
    else:
      select_contour_index = contour_area_set.index(max(contour_area_set))
      #print center_set[select_contour_index][0] #x
      #print center_set[select_contour_index][1] #y
      #print "***"
      pixel_bias_x = center_set[select_contour_index][0] - cv_image.shape[1] / 2
      pixel_bias_y = center_set[select_contour_index][1] - cv_image.shape[0] / 2
      pubTwist.linear.x = pixel_bias_x
      pubTwist.linear.y = pixel_bias_y
      pubTwist.linear.z = 1
      
            
    #print len(large_contours)
    cv2.drawContours(cv_image, large_contours, -1, (0,255,0),3)
    
    #cv2.imshow("Image window", cv_image)
    #cv2.waitKey(3)
    
    self.position_pub.publish(pubTwist)
    self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))


def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
  main(sys.argv)

