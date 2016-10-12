#!/usr/bin/env python

from dji_sdk.dji_drone import DJIDrone
import dji_sdk.msg 
import time
import sys
import math


def main():
    drone = DJIDrone()
    time.sleep(2)
    drone.request_sdk_permission_control()
    print "[INFO] Taking off after 5 seconds"
    drone.takeoff()
    time.sleep(5)
    print "[INFO] Taking off now"
    time.sleep(8)
    print "[INFO] Moving upwards to 10 meters"
#    drone.attitude_control(0x90, 0, 0, 10, 0)  

    drone.local_position_navigation_send_request(0,0,10)

    while 1:
        if drone.local_position_navigation_action_client.get_result():
            break
        time.sleep(0.02)
    print "[INFO] Arriving 10 meters high"
    print "[INFO] Preparing to go home"
    drone.gohome()
    
if __name__ == "__main__":
    main()
