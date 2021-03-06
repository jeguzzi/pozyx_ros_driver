#!/usr/bin/env python
"""
Meant to configure not only the device but the entire network
on the same UWB settings.

This should be run once every time you change settings etc but no
more than that, as it saves to flash. It's a ROS node that runs
until done then dies, so running this until it completes and then
starting the other nodes is valid.

PS: Don't forget the anchor configuration.
"""

import rospy
from pozyx_ros_driver import driver
from pozyx_ros_driver import configure


if __name__ == '__main__':
    rospy.init_node('uwb_configurator')
    baudrate = rospy.get_param('~baudrate', 115200)
    p = driver.pozyx_device(baudrate=baudrate)
    if not p:
        rospy.logerr('Could not initialize a device')
        exit(1)
    settings = rospy.get_param('~uwb')
    configure.set_uwb_settings(p, set_all=rospy.get_param('config_all', True), **settings)
