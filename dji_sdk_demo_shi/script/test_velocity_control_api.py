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

    for i in range(0, 50):
        drone.velocity_control(0, 0.5, 0, 0, 0)
        time.sleep(0.02)
    
if __name__ == "__main__":
    main()
