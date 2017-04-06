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

import pypozyx
import rospy
import sys
from pozyx import pozyx_device


def set_gain(gain):
    rospy.loginfo("Setting all encountered devices with gain: %f" % gain)
    pozyx = pozyx_device()
    if not pozyx:
        rospy.logerr('No device found')
        return
    pozyx.doDiscovery(pypozyx.POZYX_DISCOVERY_ALL_DEVICES)
    device_list_size = pypozyx.SingleRegister()
    pozyx.getDeviceListSize(device_list_size)
    if device_list_size[0] > 0:
        device_list = pypozyx.DeviceList(list_size=device_list_size[0])
        pozyx.getDeviceIds(device_list)
    pozyx.setTxPower(gain)
    for device in device_list:
        pozyx.setTxPower(gain, remote_id=device)
    rospy.loginfo("Gain set. Shutting down configurator node now ...")


if __name__ == '__main__':
    rospy.init_node('set_gain')
    gain = float(sys.argv[1])
    set_gain(gain)
