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

import socket
from threading import Thread, currentThread

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import ChannelFloat32, PointCloud, PointCloud2, PointField
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, Header
from std_srvs.srv import Empty, EmptyResponse

from src.drivers.misc.status import STATE, status

# Last published PCL
last_scan = None

# PointCloud2 fields for the laser scanner
FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
]


def b14toInt(b):
    """Convert 14bit integers from bytes

    Arguments:
        b {bytearray} -- bytes

    Returns:
        int32 -- b14 decoded int[summary]
    """

    if len(b) < 2:
        return 0

    b1 = ord(b[0]) << 1
    b2 = ord(b[1]) << 8
    bc = b2 + b1
    bc = bc >> 1

    return bc


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

    Disabled = -1
    Continues = 1


class MEL_M2D_Driver(object):
    """MEL M2D Scanner driver

    Arguments:
        object {object} -- parent class
    """

    TCP = None

    def __init__(self, host, port, name="laser_scanner"):
        """Init MEL M2D Driver

        Arguments:
            host {str} -- scanner ip
            port {int} -- scanner port

        Keyword Arguments:
            name {str} -- node name (default: {"laser_scanner"})
        """

        self.matrix = None
        self.name = name
        self.host = host
        self.port = port
        self.scanning = ScanningMode.Disabled
        self.measuring = False
        self.sock = None
        self.scanning_started = None
        self.update_thread = None

        # init MEL laser scanner node
        rospy.init_node(name, anonymous=True)

        # init the laser scanner
        self.init_scanner()

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

    def start(self, data):
        """Start scanning service handler
        """

        if self.scanning != ScanningMode.Disabled:
            # return and do nothing, if the scanning is already started
            return EmptyResponse()

        self.scanning = ScanningMode.Continues
        self.scanning_started = now()

        if self.scanning == ScanningMode.Continues:
            # in continous mode, start the reading on a thread
            self.update_thread = Thread(target=self.update)
            self.update_thread.do_run = True
            self.update_thread.start()

        # Publish started
        self.status_publisher.publish(Bool(data=True))

        return EmptyResponse()

    def stop(self, *args, **kwargs):
        """Stop scanning service handler
        """

        if self.sock is not None:
            self.sock.close()
            self.sock = None

        self.scanning = ScanningMode.Disabled

        # Publish stopped
        self.status_publisher.publish(Bool(data=False))

        if not hasattr(self, "update_thread"):
            return EmptyResponse()

        self.update_thread.do_run = False
        self.update_thread.join()

        return EmptyResponse()

    def update(self):
        """Update scanner data with triggering the scanner
        """
        t = currentThread()
        status(self.name, "Continous scanning started")

        sock = None

        while not rospy.is_shutdown() and getattr(t, "do_run", True):

            if sock is None:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.connect((self.host, self.port))

            self.measure(sock)

        status(self.name, "Continous scanning finished")

        sock.close()

    def receive(self, sock, buff=4096):
        """Receive message from a socket

        Arguments:
            sock {socket} -- socket to use

        Keyword Arguments:
            buff {int} -- size of the buffer (default: {4096})

        Returns:
            bytearray -- data
        """

        try:
            return sock.recv(buff)
        except Exception as e:
            status(
                self.name,
                "Socket receive failed: %s" %
                (str(e)),
                STATE.ERROR)

    def send(self, sock, msg):
        """Send a message on a socket

        Arguments:
            sock {socket} -- socket to use
            msg {bytearray} -- message
        """
        try:
            sock.send(msg)
        except Exception as e:
            status(self.name, "Socket send failed: %s" % (str(e)), STATE.ERROR)

    def measure(self, sock):
        """Trigger one measurement

        Keyword Arguments:
            sock {socket} -- the communication socket (default: {None})
        """
        started = int(rospy.Time.now().to_nsec() / 1000000)

        # Create a temporary point cloud for the current measurement
        pc = PointCloud()
        pc.header = Header()
        pc.header.stamp = rospy.Time(0)
        pc.header.frame_id = MEL_M2D_Driver.TCP

        # Create intensity channel
        channel = ChannelFloat32()
        channel.name = "Intensity"

        try:
            # Trigger update request
            self.send(sock, bytearray([0x1D]))
            data = self.receive(sock, 4096)

            if len(data) == 0:
                return

            for i in range(66, 66 + 290 * 5, 5):

                try:
                    x = round(
                        self._scan_width_end * b14toInt(data[i:i + 2]) /
                        self._max_value_meas, 2) / 1000
                    z = round(self._begin_of_range + self._range *
                              b14toInt(data[i + 2:i + 4]) / self._max_value_scan, 2) / 1000
                    intensity = round(
                        self._begin_of_range *
                        int(format(ord(data[i + 4]), '08b')[::-1], 2) / 254, 2)

                    pc.points.append(Point(x=x, z=-z))
                    channel.values.append(intensity)
                except BaseException:
                    status(
                        self.name, "Error at index: %d, the max length is %d" %
                        (i, len(data)), STATE.ERROR)

            pc.channels.append(channel)

            gen = pc2.create_cloud(
                pc.header, FIELDS, [
                    (p.x, p.y, p.z) for p in pc.points])

            self.reading_publisher_pcl2.publish(gen)
            self.reading_publisher_pcl.publish(pc)

            finished = int(rospy.Time.now().to_nsec() / 1000000)
            status(
                self.name, "Measure received: %d" %
                (finished - started), STATE.DEBUG)

            return

        except Exception as e:
            status(self.name, "Measurement failed: %s" % (str(e)), STATE.ERROR)

    def close(self):
        """Close the connection and stop reading
        """

        self.stop()

        status(self.name, "Laser scanner - stopped")

    def init_scanner(self):
        """Init scanner data
        """

        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((self.host, self.port))

        self.send(sock, bytearray([0x14, 0x88, 0x1C, 0x81, 0x23, 0x00, 0x21]))
        data = self.receive(sock, 4096)

        # convert the given bytes to ints
        self._begin_of_range = b14toInt(data[98 + 8:98 + 10]) * 0.1
        self._range = b14toInt(data[98 + 10:98 + 12]) * 0.1
        self._scan_width_start = b14toInt(data[98 + 12:98 + 14]) * 0.1
        self._scan_width_end = b14toInt(data[98 + 14:98 + 16]) * 0.1
        self._max_value_meas = b14toInt(data[98 + 16:98 + 18])
        self._max_value_scan = b14toInt(data[98 + 18:98 + 20])

        status(self.name, "Init values: ")
        status(self.name, "_begin_of_range: %d" % (self._begin_of_range))
        status(self.name, "_range: %d" % (self._range))
        status(self.name, "_scan_width_start: %d" % (self._scan_width_start))
        status(self.name, "_scan_width_end: %d" % (self._scan_width_end))
        status(self.name, "_max_value_meas: %d" % (self._max_value_meas))
        status(self.name, "_max_value_scan: %d" % (self._max_value_scan))

        sock.close()


if __name__ == "__main__":
    try:
        nm = rospy.get_param("laser_name", "laser_scanner")
        # get ros params
        MEL_M2D_Driver.TCP = rospy.get_param("laser_tcp", "scanner")

        # create the driver
        driver = MEL_M2D_Driver(rospy.get_param("laser_ip", "192.168.1.210"),
                                rospy.get_param("laser_port", 3000),
                                nm)

        # start listening
        rospy.spin()

    except Exception as e:
        status(
            nm,
            'MEL M2D-iLAN Failed to start driver: %s' %
            (str(e)),
            STATE.ERROR)

    finally:
        # close connections
        if 'driver' in vars() or 'driver' in globals():
            driver.close()
