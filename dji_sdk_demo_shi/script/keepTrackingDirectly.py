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
        self.sub = rospy.Subscriber("relativePosition", Twist, self.relative_position_subscriber_callback)
        self.drone = DJIDrone()
        self.drone.request_sdk_permission_control()
        time.sleep(1)

    def relative_position_subscriber_callback(self, relativePosition):
        self.relative_position = relativePosition

    def run(self):
        rate = rospy.Rate(50)

        while not rospy.is_shutdown():
	    #self.drone.request_sdk_permission_control()
            vel_x = 0
            vel_y = 0
            if self.relative_position.linear.x > 4:
	        vel_x = 4
            elif self.relative_position.linear.x < -4:
	        vel_x = -4
	    else:
		vel_x = self.relative_position.linear.x
            if self.relative_position.linear.y > 4:
	        vel_y = 4
            elif self.relative_position.linear.y < -4:
	        vel_y = -4
	    else:
		vel_y = self.relative_position.linear.y
            self.drone.attitude_control(0x4A, vel_x, vel_y, 0, 0)
            rate.sleep()



if __name__ == '__main__':
    try:
        myTracker = tracker()
        myTracker.init()
        myTracker.run()
    except rospy.ROSInterruptException:
        pass

