#!/usr/bin/env python

from dji_sdk.dji_drone import DJIDrone
import dji_sdk.msg
import time
import sys
import math
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan

class RPYClass:
    def __init__(self):
        self.roll_ = 0.0
        self.pitch_ = 0.0
        self.yaw_ = 0.0
    def change_quaternion_to_rpy(self, q):
        self.roll  = math.atan2(2.0 * (q.q3 * q.q2 + q.q0 * q.q1) , 1.0 - 2.0 * (q.q1 * q.q1 + q.q2 * q.q2))
        self.pitch = math.asin(2.0 * (q.q2 * q.q0 - q.q3 * q.q1))
        self.yaw = math.atan2(2.0 * (q.q3 * q.q0 + q.q1 * q.q2) , - 1.0 + 2.0 * (q.q0 * q.q0 + q.q1 * q.q1))



class tracker():
    def init(self):
        rospy.init_node('relative_velocity', anonymous=True)
        self.relative_position = Twist()
        self.relative_position.linear.x = 0
        self.relative_position.linear.y = 0
        self.ultrasonic_value = LaserScan()
        self.ultrasonic_value.intensities = [0]

        ## Krish's output
        #self.sub = rospy.Subscriber("/ros_cmt_tracker/output/twist", Twist, self.relative_position_subscriber_callback)
        
        ## Red region's output
        self.subscriber_relative_pixel_position = rospy.Subscriber("/relative_pixel_position", Twist, self.relative_position_subscriber_callback)
        self.camera_f_value = 680.8

        
        self.subscriber_ultrasonic = rospy.Subscriber("/guidance/ultrasonic", LaserScan, self.ultrasonic_subscriber_callback)

        self.publisher_velocity = rospy.Publisher("control_velocity", Twist, queue_size = 10)
        
        #self.drone = DJIDrone()
        #self.drone.request_sdk_permission_control()
        time.sleep(1)

    def ultrasonic_subscriber_callback(self, sonicValue):
        self.ultrasonic_value = sonicValue
        
    def relative_position_subscriber_callback(self, relativePosition):
        self.relative_position = relativePosition

    def run(self):
        rate = rospy.Rate(20)

        control_velocity = Twist()
        control_velocity.linear.x = 0
        control_velocity.linear.y = 0

        print "test"
        while not rospy.is_shutdown():
            if self.ultrasonic_value.intensities[0] < 0.5:
                print "ultrasonic sensor is not reliable."
                self.publisher_velocity.publish(control_velocity)
                rate.sleep()
                continue
            
	    relative_pos_x = -self.relative_position.linear.y / self.camera_f_value * self.ultrasonic_value.ranges[0]
	    relative_pos_y = self.relative_position.linear.x / self.camera_f_value * self.ultrasonic_value.ranges[0]
            vel_x = 0
            vel_y = 0
            if relative_pos_x > 1:
	        vel_x = 1
            elif relative_pos_x < -1:
	        vel_x = -1
	    else:
		vel_x = relative_pos_x
            if relative_pos_y > 1:
	        vel_y = 1
            elif relative_pos_y < -1:
	        vel_y = -1
	    else:
		vel_y = relative_pos_y
            #self.drone.attitude_control(0x4A, vel_x, vel_y, 0, 0)
            #self.drone.velocity_control(0, vel_x, vel_y, 0, 0)
            control_velocity.linear.x = vel_x
            control_velocity.linear.y = vel_y
            self.publisher_velocity.publish(control_velocity)
            rate.sleep()


if __name__ == '__main__':
    try:
        myTracker = tracker()
        myTracker.init()
        myTracker.run()
    except rospy.ROSInterruptException:
        pass
