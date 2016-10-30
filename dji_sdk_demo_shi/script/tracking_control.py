#!/usr/bin/env python

# Copyright (c) 2016, JSK(University of Tokyo)
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Open Source Robotics Foundation, Inc.
#       nor the names of its contributors may be used to endorse or promote
#       products derived from this software without specific prior
#       written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Authors: Fan Shi, Moju Zhao
# Maintainer: Fan Shi <shifan@jsk.imi.i.u-tokyo.ac.jp> 

import time
import sys
import math
import rospy
import tf
import json
import numpy as np
from numpy.linalg import inv
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from dji_sdk.dji_drone import DJIDrone
import dji_sdk.msg
        
class tracker:

    def init(self):
        ### Flag for DJI M100
        if len(sys.argv) != 4:
            self.__use_dji = False
        else:
            if sys.argv[1] == 'True':
                self.__use_dji = True
            else:
                self.__use_dji = False

        if self.__use_dji == False:
            rospy.init_node('aerial_tracking', anonymous=True)

        ## Basic Variables
        self.__object_image_center = PointStamped() # the position of target object in terms of the camera coord
        self.__level_camera_coord_target_image_center = PointStamped() # the position of target object in terms of the horizontal camera coord
        
        self.__euler = np.array([0.0, 0.0, 0.0]) # the orientation of the uav: [roll,pitch,yaw]

        self.__target_pos = np.array([0.0, 0.0, 0.0]) # the target(reference) position of uav to follow the object
        self.__object_pos = np.array([0.0, 0.0, 0.0]) # the current postion of the object
        self.__control_vel = np.array([0.0, 0.0, 0.0]) # the velocity command (input) to cotrol the uav for tracking
        
        self.__object_image_center_update = False # the flag to check the update of the object visual tracking
        
        ## ROS Param
        ### Topic Name
        self.__object_image_center_sub_topic_name = rospy.get_param("~object_image_center_sub_topic_name", "/object_image_center")
        self.__level_camera_coord_object_image_center_pub_topic_name = rospy.get_param("~level_camera_coord_object_image_center_pub_topic_name", "/level_camera_coord_object_image_center")
        if self.__use_dji:
            self.__uav_odom_sub_topic_name = rospy.get_param("~uav_odom_sub_topic_name", "/dji_sdk/odometry")
        else:
            self.__uav_odom_sub_topic_name = rospy.get_param("~uav_odom_sub_topic_name", "/uav_odom")
            
        self.__ultrasonic_sub_topic_name = rospy.get_param("~ultrasonic_sub_topic_name", "/guidance/ultrasonic")
        self.__control_velocity_pub_topic_name = rospy.get_param("~control_velocity_pub_topic_name", "/control_velocity")
        self.__object_pos_topic_name = rospy.get_param("~object_pos_pub_topic_name", "/object_pos")

        ### Height of the object
        self.__object_z_offset = rospy.get_param("~object_z_offset", 0.0)
    
        ## Camera Intrinsic Paramter
        self.__camera_f_x = rospy.get_param("~camera_f_x", 680.8)
        self.__camera_f_y = rospy.get_param("~camera_f_y", 680.8)
        self.__camera_c_x = rospy.get_param("~camera_c_x", 619.2)
        self.__camera_c_y = rospy.get_param("~camera_c_x", 349.84)

        self.__c = np.array([[self.__camera_f_x, 0.0, self.__camera_c_x],[0.0, self.__camera_f_y, self.__camera_c_y],[0.0, 0.0, 1.0]])
        self.__inv_c = inv(np.matrix(self.__c))

        ### Tracking Control Gains
        self.__p_gain = rospy.get_param("~p_gain", 0.5)
        self.__i_gain = rospy.get_param("~i_gain", 0.0)
        self.__d_gain = rospy.get_param("~d_gain", 1.0)
        self.__max_control_vel = rospy.get_param("~max_control_vel", 5.0)
        
        rospy.loginfo("Control gain, p: %f, i: %f, d: %f", self.__p_gain, self.__i_gain, self.__d_gain)
        
        ## Subscriber
        self.__subscriber_odometry = rospy.Subscriber(self.__uav_odom_sub_topic_name, Odometry, self.__uav_odom_callback)
        self.__subscriber_object_image_center = rospy.Subscriber(self.__object_image_center_sub_topic_name, PointStamped, self.__object_image_center_callback)
        self.__subscriber_ultrasonic = rospy.Subscriber(self.__ultrasonic_sub_topic_name, LaserScan, self.__ultrasonic_callback)
        ### TODO: please use subscribe to receive the camera info

        ## Publisher
        self.__publisher_control_velocity = rospy.Publisher(self.__control_velocity_pub_topic_name, TwistStamped, queue_size = 10)
        self.__publisher_object_pos = rospy.Publisher(self.__object_pos_topic_name, PointStamped, queue_size = 10)
        self.__publisher_level_camera_coord_object_image_center = rospy.Publisher(self.__level_camera_coord_object_image_center_pub_topic_name, PointStamped, queue_size = 10)

        ## Initial Process
        
        ### DJI bringup
        if self.__use_dji == True:
            print "Use DJI"
            self.__drone = DJIDrone()
            self.__drone.request_sdk_permission_control()
        time.sleep(1)

    # Callback Func
    def __ultrasonic_callback(self, msg):
        self.__range_sensor_value = msg
        
    def __object_image_center_callback(self, msg):
        self.__object_image_center = msg
        self.__object_image_center_update = True

    def __uav_odom_callback(self, msg):
        q = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self.__euler = tf.transformations.euler_from_quaternion(q)
        self.__object_pos[2] = msg.pose.pose.position.z - self.__object_z_offset
        
    # Orientaion Transformation for tracking
    def __get_rotation_matrix_from_world_coord_to_image_coord(self):
        ### TODO: this is hard coding , we have to add another matrix for this transformation
        ### e.g. : param for camera  setting direction(pitch, roll, yaw) related to uav body coord
        m_roll = np.array([[1.0, 0.0, 0.0],[0.0, math.cos(self.__euler[0]), -math.sin(self.__euler[0])],[0.0, math.sin(self.__euler[0]), math.cos(self.__euler[0])]])
        m_pitch = np.array([[math.cos(self.__euler[1]), 0.0, math.sin(self.__euler[1])],[0.0, 1.0, 0.0],[-math.sin(self.__euler[1]), 0.0, math.cos(self.__euler[1])]])
        m_yaw = np.array([[0.0, -1.0, 0.0],[1.0, 0.0, 0.0],[0.0, 0.0, 1.0]])
        return (m_pitch.dot(m_roll)).dot(m_yaw)

    # Main Process
    def run(self):
        rate = rospy.Rate(20)
        
        while not rospy.is_shutdown():
            
            """
             uav_coordinate means uav is tilt when flying, we have to mapping to a coord whose z axis is verticle with ground,
             and x/y axis is parallel with ground, in order to give correct control value for uav
            """
            ## transformation part
            if self.__object_image_center_update:
                self.__object_image_center_update = False
                if self.__object_image_center.point.x == -1 and self.__object_image_center.point.y == -1:
                    self.__object_pos[0] = 0
                    self.__object_pos[1] = 0
                else:
                    # calculate object position respect to world horizontal coordinate: 
                    # camera_coord_image_pos = [u,v,1]
                    camera_coord_image_pos = np.array([self.__object_image_center.point.x, self.__object_image_center.point.y, 1.0])
                    # world_coord_image_pos = R * invC * [u,v,1]
                    world_coord_image_pos = self.__get_rotation_matrix_from_world_coord_to_image_coord().dot((self.__inv_c.dot(camera_coord_image_pos)).transpose())
                    # pos_x / world_coord_image_pos_x = pos_y / world_coord_image_pos_y = pos_z / world_coord_image_pos_z
                    self.__object_pos[0] = self.__object_pos[2] * world_coord_image_pos[0] / world_coord_image_pos[2]
                    self.__object_pos[1] = self.__object_pos[2] * world_coord_image_pos[1] / world_coord_image_pos[2]

                # calculate the image position in terms of the horizontal camera plane:69
                                
                # level_camera_coord_image_center = [u_dash, v_dash, 1] = C * object_pos / object_pos_z
                m_yaw_dash = np.array([[0.0, 1.0, 0.0],[-1.0, 0.0, 0.0],[0.0, 0.0, 1.0]])
                level_camera_coord_image_center = self.__c.dot(m_yaw_dash.dot(world_coord_image_pos))
                self.__level_camera_coord_target_image_center.point.x = level_camera_coord_image_center[0] / level_camera_coord_image_center[2]
                self.__level_camera_coord_target_image_center.point.y = level_camera_coord_image_center[1] / level_camera_coord_image_center[2]
                
            ## tracking control part
            ## P control : vel = p_gain * pos_err
            ## CAUTION: BE CAREFULL THAT THE TARGET_POS DENOTES TO THE POSITION OF OBJECT IN TERMS OF LEVEL CAMERA FRAME, NOT THE UAV POSITION IN TERMS OF WORLD FRAME WITH OBJECT ORIGIN 
            raw_control_vel = self.__p_gain * (self.__object_pos - self.__target_pos)
            self.__control_vel = np.clip(raw_control_vel, -self.__max_control_vel, self.__max_control_vel)
            
            ## DJI
            if(self.__use_dji):
                self.__drone.velocity_control(0, self.__control_vel[0], self.__control_vel[1] , 0, 0)

                
            ## publish part
            control_veloicty_msg = TwistStamped()
            control_veloicty_msg.header.stamp = rospy.Time.now()
            control_veloicty_msg.twist.linear.x = self.__control_vel[0]
            control_veloicty_msg.twist.linear.y = self.__control_vel[1]
            self.__publisher_control_velocity.publish(control_veloicty_msg)
                        
            object_pos_msg = PointStamped()
            object_pos_msg.header.stamp = rospy.Time.now()
            object_pos_msg.point.x = self.__object_pos[0]
            object_pos_msg.point.y = self.__object_pos[1]
            object_pos_msg.point.z = self.__object_pos[2]            
            self.__publisher_object_pos.publish(object_pos_msg)

            self.__level_camera_coord_target_image_center.header.stamp = rospy.Time.now()
            self.__publisher_level_camera_coord_object_image_center.publish(self.__level_camera_coord_target_image_center)
            
            rate.sleep()

if __name__ == '__main__':
    try:
        ObjectTracker = tracker()
        ObjectTracker.init()
        ObjectTracker.run()
    except rospy.ROSInterruptException:
        pass
