#!/usr/bin/env python

from dji_sdk.dji_drone import DJIDrone
import dji_sdk.msg 
import time
import sys
import math


def main():
    drone = DJIDrone()
    time.sleep(1)
    drone.request_sdk_permission_control()
    drone.takeoff()
    time.sleep(6)
    #drone.local_position_control(0, 0, 10, 0)
    drone.local_position_navigation_send_request(0, 0, 10)
    time.sleep(16)
    print "start tracking"
    
    target = [30, 20]
    while True:
        cur_pos = [0,0]
        cur_pos[0] = drone.odometry.pose.pose.position.x
        cur_pos[1] = drone.odometry.pose.pose.position.y
        print cur_pos
        diff_pos = [0,0]
        diff_pos[0] = target[0] - cur_pos[0]
        diff_pos[1] = target[1] - cur_pos[1]
        diff_dis = math.sqrt(diff_pos[0] ** 2 + diff_pos[1] ** 2)
        if diff_pos[0] < 0.2 and diff_pos[1] < 0.2:
            print "Mission finished."
            break
        yaw_diff = math.atan2(diff_pos[1], diff_pos[0])/math.pi * 180.0
        if diff_dis > 3:
            diff_dis = 3
        #0x43
        drone.attitude_control(0x43, diff_dis, 0, 0, yaw_diff)
        # if diff_pos[0] > 3:
        #     diff_pos[0] = 3
        # if diff_pos[1] > 3:
        #     diff_pos[1] = 3
        # drone.attitude_control(0x4A, diff_pos[0], diff_pos[1], 0, 0)
        time.sleep(0.02)
    
    
if __name__ == "__main__":
    main()
