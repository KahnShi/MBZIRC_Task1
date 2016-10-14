#!/usr/bin/env python

from dji_sdk.dji_drone import DJIDrone
import dji_sdk.msg 
import time
import sys
import math
import rospy
from geometry_msgs.msg import Twist

class tracker():
    def init(self):
        self.relative_position = Twist()
        self.relative_position.linear.x = 0
        self.relative_position.linear.y = 0
        self.sub = rospy.Subscriber("/ros_cmt_tracker/output/twist", Twist, self.relative_position_subscriber_callback)
        self.drone = DJIDrone()
        self.drone.request_sdk_permission_control()
	self.previous_position = Twist()
	self.isFirstFrame = True
        time.sleep(1)
        print "[INFO] Starting to tracking"
        print "[INFO] Taking off after 5 seconds"
        self.drone.takeoff()
        time.sleep(5)
        print "[INFO] Taking off task is finished"
        time.sleep(6)
        print "[INFO] Moving upwards to 10 meters"

        self.drone.local_position_navigation_send_request(0,0,5)

        while 1:
            if not self.drone.local_position_navigation_action_client.get_result():
                time.sleep(0.02)
		continue
  	    else:
                break
        print "[INFO] Arriving 10 meters high"
        time.sleep(2)
        print "[INFO] Preparing to tracking"

    def relative_position_subscriber_callback(self, relativePosition):
        self.relative_position = relativePosition

    def main(self):
        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
#	    self.drone.request_sdk_permission_control()
	    if not self.isFirstFrame and self.previous_position.linear.x == self.relative_position.linear.x and self.previous_position.linear.y == self.relative_position.linear.y:
		self.drone.attitude_control(0x4A, 0, 0, 0, 0)
		print "[Warning] Same with previous frame. Maybe data transmassion too slow, or object failed to detect."
		rate.sleep()
		continue

	    if self.isFirstFrame:
		self.isFirstFrame = False
	    self.previous_position = self.relative_position

	    myScale = 80.0
	    relative_pos_x = -self.relative_position.linear.y / myScale
	    relative_pos_y = self.relative_position.linear.x / myScale
            vel_x = 0
            vel_y = 0
            if relative_pos_x > 3:
	        vel_x = 3
            elif relative_pos_x < -3:
	        vel_x = -3
	    else:
		vel_x = relative_pos_x
            if relative_pos_y > 3:
	        vel_y = 3
            elif relative_pos_y < -3:
	        vel_y = -3
	    else:
		vel_y = relative_pos_y
            self.drone.attitude_control(0x4A, vel_x, vel_y, 0, 0)
            rate.sleep()



if __name__ == '__main__':
    try:
        myTracker = tracker()
        myTracker.init()
        myTracker.main()
    except rospy.ROSInterruptException:
        pass


