#!/usr/bin/env python
from __future__ import division
from pozyx_proxy import PozyxProxy
from pozyx_proxy import (
    PozyxException,
    PozyxExceptionTimeout,
    # PozyxExceptionCalibration,
    # PozyxExceptionUwbConfig,
    # PozyxExceptionAnchorAdd,
    # PozyxExceptionGeneral,
    # PozyxExceptionI2CWrite,
    # PozyxExceptionI2CRead,
    # PozyxExceptionAnchorNotFound,
    # PozyxExceptionDiscovery,
    # PozyxExceptionFuncParam,
    # PozyxExceptionNotEnoughAnchors,
    # PozyxExceptionI2CCmdfull,
    PozyxExceptionCommQueueFull,
    # PozyxExceptionStartupBusfault,
    PozyxExceptionOperationQueueFull,
    # PozyxExceptionNone,
    # PozyxExceptionFlashInvalid
)
import pypozyx.definitions.bitmasks as bm
import pypozyx.definitions.registers as rg
import pypozyx as px
import rospy
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Vector3
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf2_ros
import numpy as np
from tf.transformations import quaternion_multiply, quaternion_inverse
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import Imu, MagneticField
import diagnostic_updater
from diagnostic_msgs.msg import DiagnosticStatus
from threading import Thread
from collections import deque

from pozyx_ros_driver.msg import Anchors

_algorithms = {'uwb_only': px.POZYX_POS_ALG_UWB_ONLY}
_dimensions = {'3d': px.POZYX_3D}

g = 9.81


def rotate_v(v, rotation):
    cr = quaternion_inverse(rotation)
    q = v + [0]
    z = quaternion_multiply(q, rotation)
    nq = quaternion_multiply(cr, z)
    return nq[:3]


def rotate_q(q, rotation):
    # cr = quaternion_inverse(rotation)
    # z = quaternion_multiply(q, rotation)
    # nq = quaternion_multiply(cr, z)
    # print(q)
    nq = quaternion_multiply(q, rotation)
    return nq


def cov(std_devs):
    d = len(std_devs)
    m = np.zeros((d, d), float)
    np.fill_diagonal(m, np.array(std_devs)**2)
    return m.flatten()


def register2version(register):
    d = register.data[0]
    major = d >> 4
    minor = d & 0xF
    return "%d.%d" % (major, minor)


def sensor_mode(register):
    modes = {
        0: 'MODE_OFF',
        1: 'ACCONLY',
        2: 'MAGONLY',
        3: 'GYROONLY',
        4: 'ACCMAGx',
        5: 'ACCGYRO',
        6: 'MAGGYRO',
        7: 'AMG',
        8: 'IMU',
        9: 'COMPASS',
        10: 'M4G',
        11: 'NDOF_FMC_OFF',
        12: 'NDOF'
    }
    m = register.data[0]
    return modes[m & 0x0F]


class Updater(Thread):

    DELAY = 0.005

    def __init__(self, driver, period):
        Thread.__init__(self)
        self.driver = driver
        self.deamon = True
        if driver.enable_raw_sensors:
            flag = bm.POZYX_INT_STATUS_IMU
        else:
            flag = 0
        if driver.enable_position:
            flag = flag | bm.POZYX_INT_STATUS_POS
        self.flag = flag
        self.period = period * 0.5
        self.start()

    def run(self):
        position = px.Coordinates()
        error = px.PositionError()
        sensor_data = px.SensorData()
        pozyx = self.driver.pozyx
        interrupt = px.SingleRegister()
        while not rospy.is_shutdown():
            try:
                if pozyx.checkForFlag(self.flag, self.DELAY, interrupt):
                    if self.driver.enable_position and interrupt.data[0] & bm.POZYX_INT_STATUS_POS:
                        pozyx.getCoordinates(position)
                        pozyx.getPositionError(error)
                        if self.driver.accept_position(position, error):
                            x = np.array([position.x, position.y, position.z]) / 1000.0
                            self.driver.publish_pose(x)
                    if (self.driver.enable_raw_sensors and
                       interrupt.data[0] & bm.POZYX_INT_STATUS_IMU):
                        pozyx.getAllSensorData(sensor_data)
                        self.driver.publish_sensor_data(sensor_data)
                rospy.sleep(self.period)
            except PozyxExceptionTimeout:
                pass
            except PozyxException as e:
                rospy.logerr(e.message)
                pozyx.ser.reset_output_buffer()
                pozyx.ser.reset_input_buffer()
                rospy.sleep(1)


class PozyxROSDriver(object):

    def __init__(self, device):
        rospy.init_node('pozyx_driver', anonymous=True)
        updater = diagnostic_updater.Updater()
        self.ps = deque(maxlen=20)
        self.es = deque(maxlen=20)
        self.remote_id = rospy.get_param('~remote_id', None)
        self.base_frame_id = rospy.get_param('~base_frame_id', 'base_link')
        debug = rospy.get_param('~debug', False)
        self.enable_orientation = rospy.get_param('~enable_orientation', True)
        self.enable_position = rospy.get_param('~enable_position', True)
        self.enable_raw_sensors = rospy.get_param('~enable_sensors', True)
        las = rospy.get_param("~linear_acceleration_stddev", 0.0)
        avs = rospy.get_param("~angular_velocity_stddev", 0.0)
        mfs = rospy.get_param("~magnetic_field_stddev", 0.0)
        os = rospy.get_param("~orientation_stddev", 0.0)
        ps = rospy.get_param('~position_stddev', 0.0)
        self.pose_cov = cov([ps] * 3 + [os] * 3)
        self.orientation_cov = cov([os] * 3)
        self.angular_velocity_cov = cov([avs] * 3)
        self.magnetic_field_cov = cov([mfs] * 3)
        self.linear_acceleration_cov = cov([las] * 3)
        _a = rospy.get_param('~algorithm', 'uwb_only')
        _d = rospy.get_param('~dimension', '3d')
        rate = rospy.get_param('~rate', 1.0)  # Hz
        self.algorithm = _algorithms.get(_a, px.POZYX_POS_ALG_UWB_ONLY)
        self.dimension = _dimensions.get(_d, px.POZYX_3D)
        baudrate = rospy.get_param('~baudrate', 115200)
        # height of device, required in 2.5D positioning
        self.height = rospy.get_param('~height', 0.0)  # mm
        p = None
        while not p and not rospy.is_shutdown():
            try:
                p = px.PozyxSerial(device, baudrate=baudrate,
                                   print_output=debug)
            except SystemExit:
                rospy.sleep(1)
        if not p:
            return

        self.pozyx = PozyxProxy(p, self.remote_id)
        self.pose_pub = rospy.Publisher('pose', PoseStamped, queue_size=1)
        self.pose_cov_pub = rospy.Publisher(
            'pose_cov', PoseWithCovarianceStamped, queue_size=1)
        self.imu_pub = rospy.Publisher('imu', Imu, queue_size=1)
        self.mag_pub = rospy.Publisher('mag', MagneticField, queue_size=1)
        self.frame_id = rospy.get_param('~anchors/frame_id')
        anchors = []
        if rospy.has_param('~anchors/positions'):
            anchors_data = rospy.get_param('~anchors/positions')
            anchors = [px.DeviceCoordinates(dev_id, 1, px.Coordinates(x, y, z))
                       for [dev_id, x, y, z] in anchors_data]
        self.check_visible_devices()
        rospy.sleep(0.1)
        self.set_anchors(anchors)
        rospy.sleep(0.1)
        self.check_config(anchors)
        rospy.sleep(0.1)
        self.tfBuffer = tf2_ros.Buffer()
        self.tf = tf2_ros.TransformListener(self.tfBuffer)
        if self.enable_orientation:
            try:
                t = self.tfBuffer.lookup_transform(
                    self.frame_id, 'utm', rospy.Time(), rospy.Duration(5.0))
                # r = t.transform.rotation
                # self.rotation = [r.x, r.y, r.z, r.w]
                self.rotation = quaternion_from_euler(0, 0, 0)  # = r
                # self.rotation = quaternion_multiply(r, self.rotation)
                rospy.loginfo('rotation map-> pozyx %s', self.rotation)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                    rospy.logerr('Can not tranform from utm to map frame')
                    self.enable_orientation = False

        sr = px.SingleRegister()
        self.pozyx.setOperationMode(0, self.remote_id)  # work as a tag

        self.pozyx.getWhoAmI(sr, self.remote_id)
        _id = '0x%.2x' % sr.data[0]
        rospy.set_param('~id', _id)
        updater.setHardwareID('Pozyx %s' % _id)
        self.pozyx.getFirmwareVersion(sr, self.remote_id)
        rospy.set_param('~firmware', register2version(sr))
        self.pozyx.getHardwareVersion(sr, self.remote_id)
        rospy.set_param('~hardware', register2version(sr))
        ni = px.NetworkID()
        self.pozyx.getNetworkId(ni)
        rospy.set_param('~uwb/network_id', str(ni))

        s = px.UWBSettings()
        self.pozyx.getUWBSettings(s, self.remote_id)
        rospy.set_param('~uwb/channel', s.channel)
        rospy.set_param('~uwb/bitrate', s.parse_bitrate())
        rospy.set_param('~uwb/prf', s.parse_prf())
        rospy.set_param('~uwb/plen', s.parse_plen())
        rospy.set_param('~uwb/gain', s.gain_db)
        self.pozyx.getOperationMode(sr, self.remote_id)
        rospy.set_param('~uwb/mode', 'anchor' if (sr.data[0] & 1) == 1 else 'tag')
        self.pozyx.getSensorMode(sr, self.remote_id)
        rospy.set_param('~sensor_mode', sensor_mode(sr))
        self_test = self.check_self_test()
        if not all(self_test.values()):
            rospy.logwarn('Failed Self Test %s', self_test)
        else:
            rospy.loginfo('Passed Self Test %s', self_test)
        freq_bounds = {'min': rate * 0.8, 'max': rate * 1.2}
        freq_param = diagnostic_updater.FrequencyStatusParam(freq_bounds, 0.1, 10)
        stamp_param = diagnostic_updater.TimeStampStatusParam()
        self.pose_pub_stat = diagnostic_updater.DiagnosedPublisher(
            self.pose_pub, updater, freq_param, stamp_param)
        updater.add("Sensor calibration", self.update_sensor_diagnostic)
        updater.add("Localization", self.update_localization_diagnostic)
        rospy.on_shutdown(self.cleanup)
        continuous = rospy.get_param('~continuous', False)
        rospy.Timer(rospy.Duration(1), lambda evt: updater.update())
        rospy.Subscriber('set_anchors', Anchors, self.has_updated_anchors)
        if continuous:
            if self.enable_position:
                ms = int(1000.0 / rate)
                # self.pozyx.setUpdateInterval(200)
                msr = px.SingleRegister(ms, size=2)
                self.pozyx.pozyx.setWrite(rg.POZYX_POS_INTERVAL, msr, self.remote_id)
                self.pozyx.setPositionAlgorithm(self.algorithm, self.dimension)
                rospy.sleep(0.1)
            else:
                msr = px.SingleRegister(0, size=2)
                self.pozyx.pozyx.setWrite(rg.POZYX_POS_INTERVAL, msr, self.remote_id)
            if self.enable_position or self.enable_raw_sensors:
                Updater(self, ms * 0.001)
            #     PositionUpdater(self, ms / 1000.0)
            # if self.enable_raw_sensors:
            #     IMUUpdater(self)
            #     # position = px.Coordinates()
            #     # while not rospy.is_shutdown():
            #     #     try:
            #     #         self.pozyx.checkForFlag(bm.POZYX_INT_STATUS_POS, 0.2)
            #     #         self.pozyx.getCoordinates(position)
            #     #         x = np.array([position.x, position.y, position.z])/1000.0
            #     #         self.publish_pose(x)
            #     #     except PozyxException as e:
            #     #         rospy.logerr(e.message)
            #     #         rospy.sleep(1)
            rospy.spin()
        else:
            rospy.Timer(rospy.Duration(1 / rate), self.update)
            rospy.spin()

    def accept_position(self, pos_mm, error_mm):
        self.es.append(abs(error_mm.y))
        if abs(error_mm.y) > 1000000 or error_mm.x != 0:
            rospy.logwarn('Faulty measurement %s %s', pos_mm, error_mm)
            self.ps.append(0)
            return False
        else:
            self.ps.append(1)
            return True

    def cleanup(self):
        rospy.loginfo('Clean and close serial port')
        ser = self.pozyx.ser
        ser.reset_output_buffer()
        ser.reset_input_buffer()
        ser.close()

    def check_sensors_calibration(self):
        sr = px.SingleRegister()
        self.pozyx.getCalibrationStatus(sr)
        v = sr.data[0]
        # print('calib {0:b} {0}'.format(v))
        mask = {3: 'SYS', 2: 'GYRO', 1: 'ACCELEROMETER', 0: 'MAGNETOMETER'}
        return {s: (((3 << (2 * i)) & v) >> (2 * i)) == 3 for i, s in mask.items()}

    def check_self_test(self):
        mask = {5: 'UWB',
                4: 'PRESSURE',
                3: 'IMU',
                2: 'GYRO',
                1: 'MAGNETOMETER',
                0: 'ACCELEROMETER'}
        sr = px.SingleRegister()
        self.pozyx.getSelftest(sr, self.remote_id)
        v = sr.data[0]
        return {s: ((1 << i) & v) != 0 for i, s in mask.items()}

    def update_localization_diagnostic(self, stat):
        if len(self.ps) == 0:
            stat.summary(DiagnosticStatus.WARN, 'Not localized')
        else:
            fit = sum(self.ps) / len(self.ps)
            mean_error = sum(self.es) / len(self.es)
            if fit > 0.8:
                status = DiagnosticStatus.OK
            elif fit > 0.2:
                status = DiagnosticStatus.WARN
            else:
                status = DiagnosticStatus.ERROR
            stat.summary(status, '')
            stat.add('valid measurements', '{0:.0f}%'.format(100 * fit))
            stat.add('mean error', '{0:.2f} m'.format(mean_error * 0.001))

    def update_sensor_diagnostic(self, stat):
        self.pozyx.lock.acquire(True)
        rs = self.check_sensors_calibration()
        self.pozyx.lock.release()
        if all(rs.values()):
            status = DiagnosticStatus.OK
        else:
            status = DiagnosticStatus.WARN
        stat.summary(status, "")
        for sensor, state in rs.items():
            stat.add(sensor, state)
        return stat

    def quaternion_in_map_frame(self, quaternion_in_utm_frame):
        return rotate_q(quaternion_in_utm_frame, self.rotation)

    def update_position(self):
        position = px.Coordinates()
        error = px.PositionError()
        self.pozyx.doPositioning(
            position, self.dimension, self.height, self.algorithm,
            remote_id=self.remote_id)
        self.pozyx.getPositionError(error)
        if self.accept_position(position, error):
            return (np.array([position.x, position.y, position.z]) / 1000.0, error)
        else:
            return None, None

    def update_orientation(self):
        quat = px.Quaternion()
        self.pozyx.getQuaternion(quat)
        return [quat.x, quat.y, quat.z, quat.w]

    def publish_pose(self, position=[0, 0, 0], orientation=[0, 0, 0, 1]):
        msg = PoseStamped()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = rospy.Time.now()
        msg.pose.position = Point(*position)
        msg.pose.orientation = Quaternion(*orientation)
        self.pose_pub.publish(msg)
        self.pose_pub_stat.tick(msg.header.stamp)

        pmsg = PoseWithCovarianceStamped()
        pmsg.header = msg.header
        pmsg.pose.pose = msg.pose
        pmsg.pose.covariance = self.pose_cov
        self.pose_cov_pub.publish(pmsg)

    def update(self, event):
        try:
            if self.enable_position:
                position, error = self.update_position()
            else:
                position = [0, 0, 0]
        except (PozyxExceptionCommQueueFull, PozyxExceptionOperationQueueFull,
                PozyxExceptionTimeout) as e:
            rospy.logwarn('%s: will sleep for 5 s', e.message)
            rospy.sleep(5)
            return
        except PozyxException as e:
            rospy.logerr('%s', e.message)
            return
        try:
            if self.enable_orientation:
                # !! pozyx orientation is not absolute but relative to the orientation at start
                q = self.update_orientation()
                self.orientation = self.quaternion_in_map_frame(q)
            else:
                self.orientation = [0, 0, 0, 1]
        except Exception as e:
            rospy.logerr(e.__class__)
            return
        if position is not None:
            self.publish_pose(position, self.orientation)

        if self.enable_raw_sensors:
            sensor_data = px.SensorData()
            try:
                self.pozyx.getAllSensorData(sensor_data, self.remote_id)
            except Exception as e:
                rospy.logerr(e)
                return
            self.publish_sensor_data(sensor_data)

    def publish_sensor_data(self, sensor_data):
        acc = np.array(sensor_data.acceleration.data) / 1000.0 * g
        ang_vel = np.array(sensor_data.angular_vel.data) / 180.0 * np.pi
        # ang_vel = rotate_v(ang_vel.tolist(), self.rotation)

        msg = Imu()
        msg.header.frame_id = self.base_frame_id
        msg.header.stamp = rospy.Time.now()
        msg.linear_acceleration = Vector3(*acc)
        # It happens that sensor_data.quaterion and pose.quaternion are NOT in the same frame
        q = sensor_data.quaternion.data
        q = [q[1], q[2], q[3], q[0]]
        msg.orientation = Quaternion(*q)
        # msg.orientation = Quaternion(*self.orientation)

        msg.angular_velocity = Vector3(*ang_vel)

        msg.linear_acceleration_covariance = self.linear_acceleration_cov
        msg.orientation_covariance = self.orientation_cov
        msg.angular_velocity_covariance = self.angular_velocity_cov
        self.imu_pub.publish(msg)

        mag = np.array(sensor_data.magnetic.data) * 1e-6
        m_msg = MagneticField()
        m_msg.header = msg.header
        m_msg.magnetic_field = Vector3(*mag)
        m_msg.magnetic_field_covariance = self.magnetic_field_cov
        self.mag_pub.publish(m_msg)

    def set_anchors(self, anchors):
        try:
            self.pozyx.clearDevices(self.remote_id)
            for anchor in anchors:
                self.pozyx.addDevice(anchor, self.remote_id)
            if len(anchors) > 4:
                self.pozyx.setSelectionOfAnchors(
                    px.POZYX_ANCHOR_SEL_AUTO, len(anchors))
        except Exception as e:
            rospy.logerr("%s", e)

    def check_config(self, anchors):
        list_size = px.SingleRegister()
        try:
            self.pozyx.getDeviceListSize(list_size, self.remote_id)
            if list_size[0] != len(anchors):
                rospy.logerr('anchors were not properly configured')
                return
            device_list = px.DeviceList(list_size=list_size[0])
            self.pozyx.getDeviceIds(device_list, self.remote_id)
            for anchor in device_list:
                anchor_coordinates = px.Coordinates()
                self.pozyx.getDeviceCoordinates(
                    anchor, anchor_coordinates, self.remote_id)
                rospy.loginfo("anchor 0x%0.4x set to %s",
                              anchor, anchor_coordinates)
        except Exception as e:
            rospy.logerr("%s", e)

    def check_visible_devices(self):
        try:
            self.pozyx.doDiscovery()
            list_size = px.SingleRegister()
            self.pozyx.getDeviceListSize(list_size)
            if list_size[0] == 0:
                return
            device_list = px.DeviceList(list_size=list_size[0])
            self.pozyx.getDeviceIds(device_list)
            for d in device_list.data:
                rospy.loginfo("Found device 0x%0.4x" % d)
        except Exception as e:
            rospy.logerr(e)

    def has_updated_anchors(self, msg):
        self.frame_id = msg.header.frame_id
        anchors = [
            px.DeviceCoordinates(anchor.id, 1, px.Coordinates(
                anchor.position.x, anchor.position.y, anchor.position.z))
            for anchor in msg.anchors]
        self.pozyx.lock.acquire(True)
        self.set_anchors(anchors)
        self.check_config(anchors)
        self.pozyx.lock.release()


def get_device():
    devices = [d for d in px.get_serial_ports() if 'Pozyx' in d.manufacturer]
    if devices:
        return devices[0].device
    return None


# TODO add proper shutdown

if __name__ == "__main__":
    device = get_device()
    if not device:
        exit(1)
    try:
        p = PozyxROSDriver(device)
    except rospy.ROSInterruptException:
        pass
