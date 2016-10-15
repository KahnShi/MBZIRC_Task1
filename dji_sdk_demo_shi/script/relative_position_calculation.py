#!/usr/bin/env python

from dji_sdk.dji_drone import DJIDrone
import dji_sdk.msg
import time
import sys
import math
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class RPYClass:
    def __init__(self):
        self.roll_ = 0.0
        self.pitch_ = 0.0
        self.yaw_ = 0.0
    def change_quaternion_to_rpy(self, q):
        self.roll  = math.atan2(2.0 * (q.q3 * q.q2 + q.q0 * q.q1) , 1.0 - 2.0 * (q.q1 * q.q1 + q.q2 * q.q2))
        self.pitch = math.asin(2.0 * (q.q2 * q.q0 - q.q3 * q.q1))
        self.yaw = math.atan2(2.0 * (q.q3 * q.q0 + q.q1 * q.q2) , - 1.0 + 2.0 * (q.q0 * q.q0 + q.q1 * q.q1))

class drone_information_class:
    def __init__(self):
        self.drone = DJIDrone()
        self.rpy_publisher = rospy.Publisher('rpy_angles', Twist, queue_size = 10)
        self.target_distance_publisher = rospy.Publisher('target_distance', Float64, queue_size = 10)
#        local_position_subscriber
    def get_request(self):
        self.drone.request_sdk_permission_control()
        time.sleep(1)
        print "Get control permission."
    
        
    def main(self):
        self.get_request()
#        rospy.init_node('dji_sdk_connector')
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            rpy = RPYClass()
            rpy.change_quaternion_to_rpy(self.drone.attitude_quaternion)
            twist_msg = Twist()
            twist_msg.angular.x = rpy.roll
            twist_msg.angular.y = rpy.pitch
            twist_msg.angular.z = rpy.yaw
            self.rpy_publisher.publish(twist_msg)
            target_distance_msg = Float64()
            target_distance_msg.data = self.drone.local_position.z / (math.cos(rpy.roll) * math.cos(rpy.pitch))
            self.target_distance_publisher.publish(target_distance_msg)
            rate.sleep()
    
if __name__ == '__main__':
    try:
        my_drone_info = drone_information_class()
        my_drone_info.main()
    except rospy.ROSInterruptException:
        pass

