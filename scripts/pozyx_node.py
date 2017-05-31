#!/usr/bin/env python

import rospy
from pozyx_ros_driver import driver


if __name__ == "__main__":
    device = driver.get_device()
    if not device:
        exit(1)
    try:
        p = driver.PozyxROSDriver(device)
    except rospy.ROSInterruptException:
        pass
