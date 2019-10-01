"""
ROSWELD
Version 0.0.1, March 2019
http://rosin-project.eu/ftp/rosweld

Copyright (c) 2019 PPM Robotics AS

This library is part of ROSWELD project,
the Focused Technical Project ROSWELD - ROS based framework
for planning, monitoring and control of multi-pass robot welding
is co-financed by the EU project ROSIN (www.rosin-project.eu)
and ROS-Industrial initiative (www.rosindustrial.eu).

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

import ctypes as ct
import pylinllt as llt
import time

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import ChannelFloat32, PointCloud, PointCloud2, PointField
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, Header
from std_srvs.srv import Empty, EmptyResponse

from src.drivers.misc.status import STATE, status

# Last published PCL
last_scan = None

# Driver
driver = None

# PointCloud2 fields for the laser scanner
FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
]


def new_packet_callback_wrapper(data, size, user_data):
    """Wrapper for the cpp callback function
    calls the current drivers measure function
    to handle the measurement

    Arguments:
        data {long} -- memory address
        size {int} -- data size
        user_data {c_void_p} -- user data
    """

    driver.measure(data, size)


def now():
    """Returns the current timestamp in ms

    Returns:
        int -- current ms
    """

    return rospy.Time.now().to_nsec()


class ScanningMode(object):
    """Scanning mode enum

    Arguments:
        object {object} -- parent class
    """

    Disabled = 0
    Continues = 1


class uEpsilon_Driver(object):
    """MEL M2D Scanner driver

    Arguments:
        object {object} -- parent class
    """

    TCP = None

    def __init__(self, name="laser_scanner", resolution=1280, serial=None):
        """Init MEL M2D Driver

        Arguments:
            host {str} -- scanner ip
            port {int} -- scanner port

        Keyword Arguments:
            name {str} -- node name (default: {"laser_scanner"})
            robot_name {str} -- name of the current robot (default: {""}) this parameters will define the RobotState object to use
        """

        global driver

        self.matrix = None
        self.name = name
        self.scanning = ScanningMode.Disabled
        self.measuring = False
        self.resolution = resolution
        self.serial = serial
        self.scanning_started = None

        # init MEL laser scanner node
        rospy.init_node(name, anonymous=True)

        # create ROS topics, services and subscriptions
        rospy.Service('start', Empty, self.start)
        rospy.Service('stop', Empty, self.stop)
        self.reading_publisher_pcl2 = rospy.Publisher(
            'laser_scan', PointCloud2, queue_size=100, latch=True)
        self.reading_publisher_pcl = rospy.Publisher(
            'meassured_z', PointCloud, queue_size=100, latch=True)
        self.status_publisher = rospy.Publisher(
            'is_scanning', Bool, queue_size=1, latch=True)

        # Publish stopped
        self.status_publisher.publish(Bool(data=False))

        status(self.name, "Laser scanner - Ready (%s)" % (name))

        driver = self

        # Try to connect to the interface
        self.connect(True)
        self.disconnect()

    def start(self, *args, **kwargs):
        """Start scanning service handler
        """

        if self.scanning != ScanningMode.Disabled:
            # return and do nothing, if the scanning is already started
            return EmptyResponse()

        self.scanning = ScanningMode.Continues
        self.scanning_started = now()

        # Warm-up time
        rospy.sleep(0.2)

        self.connect()
        self.start_transmission()

        # Publish started
        self.status_publisher.publish(Bool(data=True))

        return EmptyResponse()

    def stop(self, *args, **kwargs):
        """Stop scanning service handler
        """

        if self.scanning != ScanningMode.Disabled:
            self.scanning = ScanningMode.Disabled

            # Publish stopped
            self.status_publisher.publish(Bool(data=False))
            self.stop_transmission()
            self.disconnect()

        return EmptyResponse()

    def connect(self, test=False):
        """Connect to the scanner and setup for measurement

        Raises:
            ValueError -- Getting interfaces failed
            ValueError -- Setting interface failed
            Exception -- Cant connect to interface
            ValueError -- Getting resolutions failed
            AttributeError -- Wrong resolutuion
            ValueError -- Error setting scanner type
            ValueError -- Error setting resolutin
            ValueError -- Error setting profile type
            ValueError -- Error setting transfer type
        """

        # Parametrize transmission --- Important: make sure this is compliant
        # to sensor
        self.scanner_type = ct.c_int(0)

        # Init profile buffer and timestamp info

        available_resolutions = (ct.c_uint * 4)()

        available_interfaces = [ct.create_string_buffer(8) for i in range(6)]
        available_interfaces_p = (
            ct.c_char_p * 6)(*map(ct.addressof, available_interfaces))

        self.hLLT = llt.create_llt_device()

        ret = llt.get_device_interfaces(
            available_interfaces_p,
            len(available_interfaces))
        if ret < 1:
            raise ValueError("Error getting interfaces : " + str(ret))

        if_index = 0
        if_found = False
        for i, available_interface in enumerate(available_interfaces_p):
            if self.serial in available_interface:
                if_found = True
                if_index = i

        if not if_found:
            status(
                self.name,
                "Serial is not defined or not available, connecting to the first one",
                STATE.INFO)
        else:
            status(self.name, "Connecting to %s" % (self.serial))

        ret = llt.set_device_interface(
            self.hLLT, available_interfaces[if_index])
        if ret < 1:
            raise ValueError("Error setting device interface: " + str(ret))

        # Connect
        ret = llt.connect(self.hLLT)
        if ret < 1:
            raise Exception("Error connect: " + str(ret))

        if test:
            return

        # Get available resolutions
        ret = llt.get_resolutions(
            self.hLLT,
            available_resolutions,
            len(available_resolutions))
        if ret < 1:
            raise ValueError("Error getting resolutions : " + str(ret))

        if self.resolution not in available_resolutions:
            raise AttributeError("Wrong resolution")

        # Scanner type
        ret = llt.get_llt_type(self.hLLT, ct.byref(self.scanner_type))
        if ret < 1:
            raise ValueError("Error scanner type: " + str(ret))

        # Scanner type
        ret = llt.set_resolution(self.hLLT, self.resolution)
        if ret < 1:
            raise ValueError("Error setting resolution: " + str(ret))

        # Set profile config
        ret = llt.set_profile_config(self.hLLT, llt.TProfileConfig.PROFILE)
        if ret < 1:
            raise ValueError("Error setting profile config: " + str(ret))

        p = ct.c_void_p()
        # Register new packet callback
        ret = llt.register_buffer_callback(
            self.hLLT, llt.buffer_cb_func(new_packet_callback_wrapper), p)
        if ret < 1:
            raise ValueError(
                "Error registering new packet callback: " +
                str(ret))

        # Warm-up time
        time.sleep(0.2)

    def start_transmission(self):
        # Start transfer
        ret = llt.transfer_profiles(
            self.hLLT, llt.TTransferProfileType.NORMAL_TRANSFER, 1)
        if ret < 1:
            raise ValueError("Error starting transfer profiles: " + str(ret))

    def stop_transmission(self):
        # Stop transmission
        ret = llt.transfer_profiles(
            self.hLLT, llt.TTransferProfileType.NORMAL_TRANSFER, 0)
        if ret < 1:
            raise ValueError("Error stopping transfer profiles: " + str(ret))

    def measure(self, data, size):
        """Trigger one measurement
        """

        profile_buffer = (ct.c_ubyte * (self.resolution * 64))()
        ct.memmove(profile_buffer, data, size)

        shutter_opened = ct.c_double(0.0)
        shutter_closed = ct.c_double(0.0)
        profile_count = ct.c_uint(0)

        # Declare measuring data array
        x = (ct.c_double * self.resolution)()
        z = (ct.c_double * self.resolution)()
        intensities = (ct.c_ushort * self.resolution)()

        # Null pointer if data not necessary
        null_ptr_short = ct.POINTER(ct.c_ushort)()
        null_ptr_int = ct.POINTER(ct.c_uint)()
        timestamp = (ct.c_ubyte * 16)()

        try:

            ret = llt.convert_profile_2_values(
                profile_buffer,
                len(profile_buffer),
                self.resolution,
                llt.TProfileConfig.PROFILE,
                self.scanner_type,
                0,
                null_ptr_short,
                intensities,
                null_ptr_short,
                x,
                z,
                null_ptr_int,
                null_ptr_int)

            if ret & llt.CONVERT_X is 0 or ret & llt.CONVERT_Z is 0 or \
                ret & llt.CONVERT_MAXIMUM is 0:
                raise ValueError("Error converting data: " + str(ret))

            for i in range(16):
                timestamp[i] = profile_buffer[self.resolution * 64 - 16 + i]

            llt.timestamp_2_time_and_count(
                timestamp,
                ct.byref(shutter_opened),
                ct.byref(shutter_closed),
                ct.byref(profile_count),
                null_ptr_short)

            pc = PointCloud()
            pc.header = Header()
            pc.header.stamp = rospy.Time(0)
            pc.header.frame_id = uEpsilon_Driver.TCP

            # Create intensity channel
            channel = ChannelFloat32()
            channel.name = "Intensity"

            for i in range(self.resolution):
                pc.points.append(Point(x=float(x[i]), z=-float(z[i])))
                channel.values.append(intensities[i])

            pc.channels.append(channel)
            gen = pc2.create_cloud(
                pc.header, FIELDS, [
                    (p.x, p.y, p.z) for p in pc.points])

            self.reading_publisher_pcl2.publish(gen)
            self.reading_publisher_pcl.publish(pc)

        except Exception as e:
            status(self.name, str(e), STATE.ERROR)

    def disconnect(self):
        """Disconnect from the scanner

        Raises:
            Exception -- Disconnection error
            Exception -- Delete error
        """

        # Disconnect
        ret = llt.disconnect(self.hLLT)
        if ret < 1:
            raise Exception("Error while disconnect: " + str(ret))

        ret = llt.del_device(self.hLLT)
        if ret < 1:
            raise Exception("Error while delete: " + str(ret))

    def close(self):
        """Close the connection and stop reading
        """

        self.stop()

        status(self.name, "Laser scanner - stopped")


if __name__ == "__main__":
    try:
        # get ros params
        _name = rospy.get_param("laser_name", "laser_driver")
        _resolution = int(rospy.get_param("resolution", 320))
        _serial = str(rospy.get_param("serial", "214040020"))
        uEpsilon_Driver.TCP = rospy.get_param("laser_tcp", "scanner")

        # create the driver
        driver = uEpsilon_Driver(_name, _resolution, serial=_serial)

        # start listening
        rospy.spin()

    except Exception, e:
        status(
            _name,
            'uEpsilon Failed to start driver: %s' %
            (str(e)),
            STATE.ERROR)

    finally:
        # close connections
        if 'driver' in vars() or 'driver' in globals():
            driver.close()
