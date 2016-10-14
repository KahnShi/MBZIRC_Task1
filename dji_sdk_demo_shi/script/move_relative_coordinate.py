#!/usr/bin/env python

from dji_sdk.dji_drone import DJIDrone
import dji_sdk.msg 
import time
import sys
import math
import rospy


if __name__ == '__main__':
    drone = DJIDrone()
    time.sleep(1)
    drone.request_sdk_permission_control()
    print "Getting control permission. Will take action after 2 seconds."
    time.sleep(2)
    print "Action starts"
    for i in range (0, 50):
        drone.attitude_control(0x4A, 0, 1, 0, 0)
        time.sleep(0.02)
    drone.attitude_control(0x4A, 0, 0, 0, 0)
    print "Action finished"
