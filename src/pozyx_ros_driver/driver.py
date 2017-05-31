#!/usr/bin/env python
from __future__ import division
from pozyx_ros_driver.proxy import PozyxProxy
from pozyx_ros_driver.proxy import (
    PozyxException,
    # PozyxExceptionTimeout,
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
    # PozyxExceptionCommQueueFull,
    # PozyxExceptionStartupBusfault,
    # PozyxExceptionOperationQueueFull,
    # PozyxExceptionNone,
    # PozyxExceptionFlashInvalid
)
import pypozyx.definitions.bitmasks as bm
import pypozyx.definitions.registers as rg
import pypozyx as px
import rospy
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Vector3
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float32
# import tf2_ros
import numpy as np
import math
# from tf.transformations import quaternion_multiply, quaternion_inverse
# from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import Imu, MagneticField
import diagnostic_updater
from diagnostic_msgs.msg import DiagnosticStatus
from threading import Thread
from collections import deque

from pozyx_ros_driver.msg import Anchors
from pozyx_ros_driver import configure

_algorithms = {'uwb_only': px.POZYX_POS_ALG_UWB_ONLY}
_dimensions = {'3d': px.POZYX_3D}

g = 9.81


# def rotate_v(v, rotation):
#     cr = quaternion_inverse(rotation)
#     q = v + [0]
#     z = quaternion_multiply(q, rotation)
#     nq = quaternion_multiply(cr, z)
#     return nq[:3]


# def rotate_q(q, rotation):
#     nq = quaternion_multiply(q, rotation)
#     return nq


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


def log_pozyx_exception(e):
    rospy.logerr("%s: %s", e.__class__.__name__, e.message)


def h_error(error_mm):
    e = math.sqrt(error_mm.x ** 2 + error_mm.y ** 2)
    return e / 1000.0


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
        self.period = period
        self.start()

    def run(self):
        position = px.Coordinates()
        error = px.PositionError()
        sensor_data = px.SensorData()
        pozyx = self.driver.pozyx
        interrupt = px.SingleRegister()
        while not rospy.is_shutdown():
            try:
                if pozyx.checkForFlag(self.flag, self.period, interrupt):
                    if self.driver.enable_position and interrupt.data[0] & bm.POZYX_INT_STATUS_POS:
                        pozyx.getCoordinates(position)
                        pozyx.getPositionError(error)
                        if self.driver.accept_position(position, error):
                            x = np.array([position.x, position.y, position.z]) / 1000.0
                            self.driver.publish_pose(x, error=error)
                    if (self.driver.enable_raw_sensors and
                       interrupt.data[0] & bm.POZYX_INT_STATUS_IMU):
                        pozyx.getAllSensorData(sensor_data)
                        self.driver.publish_sensor_data(sensor_data)
            # except PozyxExceptionTimeout as e:
            #     rospy.logwarn('%s: %s', e.__class__.__name__, e.message)
            except PozyxException as e:
                log_pozyx_exception(e)
                # pozyx.ser.reset_output_buffer()
                # pozyx.ser.reset_input_buffer()
                # rospy.sleep(1)
                self.driver.reset()
            finally:
                rospy.sleep(self.period * 0.5)


class PozyxROSDriver(object):

    def reset_or_exit(self):
        if not self.reset():
            rospy.logerr('Failed to reset, will exit')
            rospy.signal_shutdown("User exited MORSE simulation")

    def reset(self, max_trials=0):
        self.reset_count += 1
        rospy.logwarn('Resetting pozyx')
        try:
            self.pozyx.resetSystem()
        except PozyxException as e:
            log_pozyx_exception(e)
            return False
            # rospy.logerr('Failed to reset, will exit')
            # rospy.signal_shutdown("User exited MORSE simulation")
        self.sleep(2)
        return self.robust_init_pozyx(max_trials=max_trials)

    def init_diagnostics(self):
        updater = diagnostic_updater.Updater()
        _id = rospy.get_param('~id', '?')
        updater.setHardwareID('Pozyx %s' % _id)
        freq_bounds = {'min': self.rate * 0.8, 'max': self.rate * 1.2}
        freq_param = diagnostic_updater.FrequencyStatusParam(freq_bounds, 0.1, 10)
        stamp_param = diagnostic_updater.TimeStampStatusParam()
        self.pose_pub_stat = diagnostic_updater.DiagnosedPublisher(
            self.pose_pub, updater, freq_param, stamp_param)
        if self.enable_raw_sensors:
            updater.add("Sensor calibration", self.update_sensor_diagnostic)
        updater.add("Localization", self.update_localization_diagnostic)
        updater.add("Resets", self.update_reset_diagnostic)
        rospy.Timer(rospy.Duration(1), lambda evt: updater.update())

    def expose_pozyx_config(self):
        sr = px.SingleRegister()
        # Changed: Not available in firware 1.1 (at least in the corresp. python lib)
        # self.pozyx.setOperationMode(0, self.remote_id)  # work as a tag
        self.pozyx.getWhoAmI(sr, self.remote_id)
        _id = '0x%.2x' % sr.data[0]
        rospy.set_param('~id', _id)
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

    def init_pozyx(self):

        if False and rospy.has_param('~gain_db'):
            gain = rospy.get_param('~gain_db')
            configure.set_gain(self.pozyx, gain, set_all=False)

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
        if self.continuous:
            if self.enable_position:
                ms = int(1000.0 / self.rate)
                # self.pozyx.setUpdateInterval(200)
                msr = px.SingleRegister(ms, size=2)
                self.pozyx.pozyx.setWrite(rg.POZYX_POS_INTERVAL, msr, self.remote_id)
                self.pozyx.setPositionAlgorithm(self.algorithm, self.dimension)
            else:
                msr = px.SingleRegister(0, size=2)
                self.pozyx.pozyx.setWrite(rg.POZYX_POS_INTERVAL, msr, self.remote_id)
        rospy.sleep(0.1)

    def robust_init_pozyx(self, max_trials=0):
        try:
            self.init_pozyx()
            self.expose_pozyx_config()
        except PozyxException as e:
            log_pozyx_exception(e)
            if max_trials:
                return self.reset(max_trials=max_trials - 1)
            else:
                return False
        return True

    def __init__(self, device):
        rospy.init_node('pozyx_driver', anonymous=True)
        self.max_faulty = 5
        self.reset_count = 0
        self.ps = deque(maxlen=20)
        self.es = deque(maxlen=20)
        self.remote_id = rospy.get_param('~remote_id', None)
        if self.remote_id:
            self.remote_id = int(self.remote_id, 0)
        self.base_frame_id = rospy.get_param('~base_frame_id', 'base_link')
        debug = rospy.get_param('~debug', False)
        self.enable_position = rospy.get_param('~enable_position', True)
        self.enable_raw_sensors = rospy.get_param('~enable_sensors', True)
        self.max_error = rospy.get_param("~max_error", 10.0)
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
        self.rate = rospy.get_param('~rate', 1.0)  # Hz
        self.algorithm = _algorithms.get(_a, px.POZYX_POS_ALG_UWB_ONLY)
        self.dimension = _dimensions.get(_d, px.POZYX_3D)
        baudrate = rospy.get_param('~baudrate', 115200)
        # height of device, required in 2.5D positioning
        self.height = rospy.get_param('~height', 0.0)  # mm
        p = pozyx_device(baudrate=baudrate, print_output=debug)
        if not p:
            rospy.logerr('Could not initialize a device')
            return

        self.pozyx = PozyxProxy(p, self.remote_id)
        if self.remote_id:
            rospy.loginfo('Device %s initialized with remote id 0x%04x', device, self.remote_id)
        else:
            rospy.loginfo('Device %s initialized', device)
        self.error_pub = rospy.Publisher('error', Float32, queue_size=1)
        self.pose_pub = rospy.Publisher('pose', PoseStamped, queue_size=1)
        self.pose_cov_pub = rospy.Publisher(
            'pose_cov', PoseWithCovarianceStamped, queue_size=1)
        self.imu_pub = rospy.Publisher('imu', Imu, queue_size=1)
        self.mag_pub = rospy.Publisher('mag', MagneticField, queue_size=1)
        self.frame_id = rospy.get_param('~anchors/frame_id')
        self.continuous = rospy.get_param('~continuous', False)
        # self.tfBuffer = tf2_ros.Buffer()
        # self.tf = tf2_ros.TransformListener(self.tfBuffer)

        rospy.on_shutdown(self.cleanup)
        continuous = rospy.get_param('~continuous', False)

        if not self.robust_init_pozyx(max_trials=1):
            rospy.logerr('Failed to initialize pozyx')
            return

        self.init_diagnostics()
        rospy.Subscriber('set_anchors', Anchors, self.has_updated_anchors)
        if continuous:
            if self.enable_position or self.enable_raw_sensors:
                ms = int(1000.0 / self.rate)
                Updater(self, ms * 0.001)
            rospy.spin()
        else:
            rospy.Timer(rospy.Duration(1 / self.rate), self.update)
            rospy.spin()

    def accept_position(self, pos_mm, error_mm):

        if(error_mm.x == error_mm.x == error_mm.z == 1 and
           error_mm.xy == error_mm.yz == error_mm.xz == 0):
            rospy.logwarn('Faulty measurement %s %s', pos_mm, error_mm)
            self.ps.append(0)
            self.max_faulty -= 1
            if self.max_faulty <= 0:
                self.reset_or_exit()
                self.max_faulty = 5
            return False
        self.max_faulty = 5
        e = h_error(error_mm)
        self.es.append(e)
        # rospy.loginfo('NEW Measurement %s %s', pos_mm, error_mm)
        if abs(error_mm.y) > 1000000 or error_mm.x != 0 or e > self.max_error:
            rospy.logwarn('Faulty measurement %s %s', pos_mm, error_mm)
            self.ps.append(0)
            return False
        else:
            self.ps.append(1)
            return True

    def cleanup(self):
        rospy.loginfo('Clean and close serial port')
        self.pozyx.resetSystem()
        # ser = self.pozyx.ser
        # ser.reset_output_buffer()
        # ser.reset_input_buffer()
        # ser.close()

    def check_sensors_calibration(self):
        try:
            sr = px.SingleRegister()
            self.pozyx.getCalibrationStatus(sr)
        except PozyxException as e:
            log_pozyx_exception(e)
            self.reset_or_exit()
            return
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

    def update_reset_diagnostic(self, stat):
        if self.reset_count > 0:
            stat.summary(DiagnosticStatus.WARN, 'Reset %d times' % self.reset_count)
        else:
            stat.summary(DiagnosticStatus.OK, 'Never reset')
        return stat

    def update_localization_diagnostic(self, stat):
        if len(self.ps) == 0:
            stat.summary(DiagnosticStatus.WARN, 'Not localized')
        else:
            if len(self.ps):
                fit = sum(self.ps) / len(self.ps)
            else:
                fit = 0
            mean_error = sum(self.es) / len(self.es)
            if fit > 0.8:
                status = DiagnosticStatus.OK
            elif fit > 0.2:
                status = DiagnosticStatus.WARN
            else:
                status = DiagnosticStatus.ERROR
            stat.summary(status, '')
            stat.add('valid measurements', '{0:.0f}%'.format(100 * fit))
            stat.add('mean error', '{0:.2f} m'.format(mean_error))
        return stat

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

    # def quaternion_in_map_frame(self, quaternion_in_utm_frame):
    #     return rotate_q(quaternion_in_utm_frame, self.rotation)

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

    def publish_pose(self, position=[0, 0, 0], orientation=[0, 0, 0, 1], error=None):
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

        if error is not None:
            e = h_error(error)
            self.error_pub.publish(e)

    def update(self, event):
        try:
            if self.enable_position:
                position, error = self.update_position()
            else:
                position = [0, 0, 0]
                error = None
        # except (PozyxExceptionTimeout) as e:
        #     # PozyxExceptionCommQueueFull, PozyxExceptionOperationQueueFull
        #     rospy.logwarn('%s: %s. Will sleep for 1 s', e.__class__.__name__, e.message)
        #     rospy.sleep(1)
        #     self.ps.append(0)
        #     return
        except PozyxException as e:
            log_pozyx_exception(e)
            self.ps.append(0)
            self.reset_or_exit()
            return
        if position is not None:
            self.publish_pose(position, error=error)

        if self.enable_raw_sensors:
            sensor_data = px.SensorData()
            try:
                self.pozyx.getAllSensorData(sensor_data, self.remote_id)
            except PozyxException as e:
                log_pozyx_exception(e)
                self.reset_or_exit()
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
        rospy.loginfo('Set anchors to')
        self.pozyx.clearDevices(self.remote_id)
        for anchor in anchors:
            self.pozyx.addDevice(anchor, self.remote_id)
        if len(anchors) > 4:
            self.pozyx.setSelectionOfAnchors(
                px.POZYX_ANCHOR_SEL_AUTO, len(anchors))

    def check_config(self, anchors):
        list_size = px.SingleRegister()
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

    def check_visible_devices(self):
        rospy.loginfo("Check for visible devices")
        devices = configure.get_devices(self.pozyx)
        for d in devices:
            rospy.loginfo("Found device 0x%0.4x" % d)
        if not devices:
            rospy.logwarn("No visible device")

    def has_updated_anchors(self, msg):
        self.frame_id = msg.header.frame_id
        anchors = [
            px.DeviceCoordinates(anchor.id, 1, px.Coordinates(
                anchor.position.x * 1000.0, anchor.position.y * 1000.0,
                anchor.position.z * 1000.0))
            for anchor in msg.anchors]
        self.pozyx.lock.acquire(True)
        try:
            self.set_anchors(anchors)
            self.check_config(anchors)
        except PozyxException as e:
            log_pozyx_exception(e)
            self.reset_or_exit()
        finally:
            self.pozyx.lock.release()


def is_pozyx(device):
    if device.manufacturer:
        if 'Pozyx' in device.manufacturer:
            return True
    if device.product:
        if 'Pozyx' in device.product:
            return True
    return False


def get_device():
    devices = [d for d in px.get_serial_ports() if is_pozyx(d)]
    if devices:
        return devices[0].device
    return None


def pozyx_device(**kwargs):
    p = None
    device = get_device()
    while not p and not rospy.is_shutdown():
        try:
            p = px.PozyxSerial(device, **kwargs)
        except SystemExit:
            rospy.sleep(1)
    return p

# TODO add proper shutdow
