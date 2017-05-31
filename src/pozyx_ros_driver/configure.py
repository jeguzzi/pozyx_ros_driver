import pypozyx
import rospy


def get_devices(pozyx):
    pozyx.doDiscovery(pypozyx.POZYX_DISCOVERY_ALL_DEVICES)
    device_list_size = pypozyx.SingleRegister()
    pozyx.getDeviceListSize(device_list_size)
    if device_list_size[0] > 0:
        device_list = pypozyx.DeviceList(list_size=device_list_size[0])
        pozyx.getDeviceIds(device_list)
        return device_list
    else:
        return []


def set_gain(pozyx, gain, set_all=True):
    if set_all:
        rospy.loginfo("Setting all encountered devices with gain: %f" % gain)
        for device in get_devices(pozyx):
            rospy.loginfo("Setting device 0x%.04x with gain: %f" % (device, gain))
            pozyx.setTxPower(gain, remote_id=device)
    rospy.loginfo("Setting local device with gain: %f" % gain)
    pozyx.setTxPower(gain)
    rospy.loginfo("Gain set")


def set_uwb_settings(pozyx, set_all=True, **kwargs):
    new_uwb_settings = pypozyx.UWBSettings(**kwargs)
    gain_db = kwargs.get('gain', 15.0)
    devices_met = []
    uwb_registers = [0x1C, 0x1D, 0x1E, 0x1F]
    rospy.loginfo("Start to configure with UWB settings: %s" % str(kwargs))
    if set_all:
        rospy.loginfo('Will scan for all remote devices and configure them')
        for channel in pypozyx.POZYX_ALL_CHANNELS:
            for bitrate in pypozyx.POZYX_ALL_BITRATES:
                for prf in pypozyx.POZYX_ALL_PRFS:
                    for plen in pypozyx.POZYX_ALL_PLENS:
                        rospy.loginfo("Looking for devices on channel %d, bitrate 0x%x, "
                                      "prf 0x%x, plen 0x%x",
                                      channel, bitrate, prf, plen)
                        uwb_settings = pypozyx.UWBSettings(channel, bitrate, prf, plen, gain_db)
                        pozyx.clearDevices()
                        pozyx.setUWBSettings(uwb_settings)
                        for device in get_devices(pozyx):
                            if device not in devices_met:
                                pozyx.setUWBSettings(new_uwb_settings, device)
                                pozyx.saveRegisters(uwb_registers, device)
                                rospy.loginfo('Device with ID 0x%0.4x set', device)
                                devices_met.append(device)
        rospy.loginfo("Scanning end. Configured remote devices %s",
                      ["0x%x" % d for d in devices_met])
    pozyx.setUWBSettings(new_uwb_settings)
    pozyx.saveRegisters(uwb_registers)
    rospy.loginfo("Local device set. Configuration done.")
