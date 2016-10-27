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
from nav_msgs.msg import Odometry

class RPYClass:
    def __init__(self):
        self.roll_ = 0.0
        self.pitch_ = 0.0
        self.yaw_ = 0.0
        self.rotation_matrix = []
        
    def get_rpy_from_quaternion(self, q):
        self.roll_  = math.atan2(2.0 * (q.q3 * q.q2 + q.q0 * q.q1) , 1.0 - 2.0 * (q.q1 * q.q1 + q.q2 * q.q2))
        self.pitch_ = math.asin(2.0 * (q.q2 * q.q0 - q.q3 * q.q1))
        self.yaw_ = math.atan2(2.0 * (q.q3 * q.q0 + q.q1 * q.q2) , - 1.0 + 2.0 * (q.q0 * q.q0 + q.q1 * q.q1))

    def matrix_multiply(self, m1, m2):
        m = []
        for i in range(0,3):
            row = []
            for j in range(0,3):
                value = 0
                for k in range(0,3):
                    value += m1[i][k] * m2[k][j]
                row.append(value)
            m.append(row)
        return m

    def matrix_inverse(self, m):
        inverse_matrix = []
        for i in range(0,3):
            row = []
            for j in range(0,3):
                row.append(m[j][i])
            inverse_matrix.append(row)
        return inverse_matrix

    def get_rotation_matrix_from_rpy(self):
        ### Current camera is upside down
        ### TODO: this is hard coding , we have to add another matrix for this transformation
        ### e.g. : param for camera  setting direction(pitch, roll, yaw) related to uav body frame
        m_roll = [[1,0,0],[0,math.cos(self.roll_),-math.sin(self.roll_)],[0,math.sin(self.roll_),math.cos(self.roll_)]]
        m_pitch = [[math.cos(self.pitch_),0,math.sin(self.pitch_)],[0,1,0],[-math.sin(self.pitch_),0,math.cos(self.pitch_)]]
        m_yaw = [[0,-1,0],[1,0,0],[0,0,1]]
        self.rotation_matrix = self.matrix_multiply(self.matrix_multiply(m_pitch, m_roll), m_yaw)
                
    def get_coordinate_after_rotation(self, local_coordinate_position):
        new_coordinate_position = []
        r_inv = self.matrix_inverse(self.rotation_matrix)
        for i in range(0, 3):
            current_value = 0
            for j in range(0, 3):
                current_value += self.rotation_matrix[i][j] * local_coordinate_position[j]
            new_coordinate_position.append(current_value)
        return new_coordinate_position
        
class tracker:

    def init(self):
        #rospy.init_node('relative_velocity', anonymous=True)
        self.relative_position = Twist()
        self.relative_position.linear.x = 0
        self.relative_position.linear.y = 0
        self.ultrasonic_value = LaserScan()
        self.ultrasonic_value.intensities = [0]

        ## Krish's output
        #self.sub = rospy.Subscriber("/ros_cmt_tracker/output/twist", Twist, self.relative_position_subscriber_callback)
        
        ## Red region's output
        self.subscriber_relative_pixel_position = rospy.Subscriber("/relative_pixel_position", Twist, self.relative_position_subscriber_callback)
        ### TODO: please use subscribe to receive the camera info
        self.camera_f_value = 680.8
        self.camera_c_x = 619.2
        self.camera_c_y = 349.84

        self.subscriber_ultrasonic = rospy.Subscriber("/guidance/ultrasonic", LaserScan, self.ultrasonic_subscriber_callback)

        self.publisher_velocity = rospy.Publisher("control_velocity", Twist, queue_size = 10)

        ## test
        self.publisher_uav_coordinate_pos= rospy.Publisher("uav_coordinate_pos", Twist, queue_size = 10)
        self.publisher_uav_control_coordinate_pos= rospy.Publisher("uav_control_coordinate_pos", Twist, queue_size = 10)
        
        
        self.drone = DJIDrone()
        self.drone.request_sdk_permission_control()
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

        rotation_info = RPYClass()

        print "test"
        while not rospy.is_shutdown():
            
            rotation_info.get_rpy_from_quaternion(self.drone.attitude_quaternion)
            rotation_info.yaw_ = 0
            rotation_info.get_rotation_matrix_from_rpy()
            #print rotation_info.roll_
            #print rotation_info.pitch_
            #print "****"

            uav_coordinate_relative_z = 0
            ## When only use ultrasonic sensor to estimate height
            '''
            if self.ultrasonic_value.intensities[0] < 0.5:
                print "ultrasonic sensor is not reliable."
                self.publisher_velocity.publish(control_velocity)
                rate.sleep()
                continue
            else:
                uav_coordinate_relative_z = self.ultrasonic_value.ranges[0]
            '''
            ## When reading from dji_sdk/odometry
            uav_coordinate_relative_z = self.drone.odometry.pose.pose.position.z / (math.cos(rotation_info.roll_) * math.cos(rotation_info.pitch_))
            
            ## uav_coordinate means uav is tilt when flying, we have to mapping to a frame whose z axis is verticle with ground,
            ## and x/y axis is parallel with ground, in order to give correct control value for uav
            uav_coordinate_relative_pos = []
            if self.relative_position.linear.x == -1 and self.relative_position.linear.y == -1:
                uav_coordinate_relative_pos.append(0)
                uav_coordinate_relative_pos.append(0)
                uav_coordinate_relative_pos.append(uav_coordinate_relative_z)
                uav_control_coordinate_relative_pos = [0,0,uav_coordinate_relative_z]
            else:
	        uav_coordinate_relative_pos.append((self.relative_position.linear.x - self.camera_c_x) / self.camera_f_value * uav_coordinate_relative_z)
	        uav_coordinate_relative_pos.append((self.relative_position.linear.y - self.camera_c_y)/ self.camera_f_value * uav_coordinate_relative_z)
                uav_coordinate_relative_pos.append(uav_coordinate_relative_z)
            
                ## uav_control_coordinate means the frame we mentioned above
                uav_control_coordinate_relative_pos = rotation_info.get_coordinate_after_rotation(uav_coordinate_relative_pos)
                
            vel_x = 0
            vel_y = 0
            if uav_control_coordinate_relative_pos[0] > 5:
	        vel_x = 5
            elif uav_control_coordinate_relative_pos[0] < -5:
	        vel_x = -5
	    else:
		vel_x = uav_control_coordinate_relative_pos[0]
            if uav_control_coordinate_relative_pos[1] > 5:
	        vel_y = 5
            elif uav_control_coordinate_relative_pos[1] < -5:
	        vel_y = -5
	    else:
		vel_y = uav_control_coordinate_relative_pos[1]
            #self.drone.attitude_control(0x4A, vel_x, vel_y, 0, 0)
            p_gain = 6
            
            self.drone.velocity_control(0, vel_x / p_gain, vel_y / p_gain , 0, 0)
            control_velocity.linear.x = vel_x / p_gain
            control_velocity.linear.y = vel_y / p_gain
            self.publisher_velocity.publish(control_velocity)

            ## for test
            uav_coordinate_pos_msg = Twist()
            uav_control_coordinate_pos_msg = Twist()
            uav_coordinate_pos_msg.linear.x = uav_coordinate_relative_pos[0]
            uav_coordinate_pos_msg.linear.y = uav_coordinate_relative_pos[1]
            uav_coordinate_pos_msg.linear.z = uav_coordinate_relative_pos[2]
            
            uav_control_coordinate_pos_msg = Twist()
            uav_control_coordinate_pos_msg.linear.x = uav_control_coordinate_relative_pos[0]
            uav_control_coordinate_pos_msg.linear.y = uav_control_coordinate_relative_pos[1]
            uav_control_coordinate_pos_msg.linear.z = uav_control_coordinate_relative_pos[2]
            
            
            self.publisher_uav_coordinate_pos.publish(uav_coordinate_pos_msg)
            self.publisher_uav_control_coordinate_pos.publish(uav_control_coordinate_pos_msg)
            
            rate.sleep()


if __name__ == '__main__':
    try:
        myTracker = tracker()
        myTracker.init()
        myTracker.run()
    except rospy.ROSInterruptException:
        pass
