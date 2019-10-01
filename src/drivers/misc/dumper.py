#pylint: skip-file
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

import glob
import os

import rospy
from rosweld_drivers.msg import RobotState
from sensor_msgs.msg import PointCloud
from std_msgs.msg import Bool, Float32MultiArray

from src.drivers.misc.status import STATE, status

# get rosparams
dump_size = rospy.get_param("dump_size", 10)
laser_name = rospy.get_param("laser_name", "laser_scanner")
robot_name = rospy.get_param("trigger_robot_name", "nachi_robot")
logdir = rospy.get_param("log_dir", "/home/ubuntu/laser_log")

# is scanning mode active
is_scanning = False
# last robot state
robot_state = None
# timestamp of the last robot state
robot_timestamp = None
# stored readings
readings = []
# scan started
scanning_started = None
# node name
name = "scan_dumper"


class Reading(object):
    """Storage object for one reading

    Arguments:
        object {object} -- parent class
    """

    header = "Robot pose timestamp (ns)\tRobot pose\tLaser measurement timestamp (ns)\tPoint cloud\tIntensity"

    def __init__(
            self,
            robot_state=None,
            robot_timestamp=None,
            measure_timestamp=None,
            point_cloud=None):
        """Init for the Reading object

        Keyword Arguments:
            robot_state {RobotState} -- Current robot state (default: {None})
            robot_timestamp {int} -- timestamp of the last robot state in ms (default: {None})
            measure_timestamp {int} -- timestamp of the last measurement in ms (default: {None})
            point_cloud {PointCloud} -- PointCloud object for the last measurement (default: {None})
        """

        self.robot_state = robot_state
        self.robot_timestamp = robot_timestamp
        self.point_cloud = point_cloud
        self.measure_timestamp = measure_timestamp

    def __str__(self):
        """ToString method for the class

        Returns:
            str -- serialized object
        """

        robot_pose = "N / A" if self.robot_state is None else [
            round(self.robot_state.pose.position.x * 1000, 6),
            round(self.robot_state.pose.position.y * 1000, 6),
            round(self.robot_state.pose.position.z * 1000, 6),
            self.robot_state.pose.orientation.x,
            self.robot_state.pose.orientation.y,
            self.robot_state.pose.orientation.z,
            self.robot_state.pose.orientation.w,
        ]

        channel = [x for x in self.point_cloud.channels[0].values]
        pcl = [[round(i.x, 6), round(i.y, 6), round(i.z, 6)]
               for i in self.point_cloud.points]

        return "%s\t%s\t%s\t%s\t%s" % (
            self.robot_timestamp, robot_pose, self.measure_timestamp, pcl, channel)


def init():
    """Init dumper node
    """

    rospy.init_node(name, anonymous=True)
    rospy.Subscriber(
        "/%s/robot_state" %
        (robot_name),
        RobotState,
        robot_update)
    rospy.Subscriber("meassured_z", PointCloud, laser_update)
    rospy.Subscriber("is_scanning", Bool, is_scanning_update)


def now():
    """Returns the current timestamp in ms

    Returns:
        int -- current ms
    """

    return rospy.Time.now().to_nsec()


def laser_update(data):
    """Laser update handler

    Arguments:
        data {Float32MultiArray} -- Z array
    """

    global is_scanning
    global robot_state
    global robot_timestamp

    if not is_scanning:
        return

    temp = Reading(robot_state, robot_timestamp, now(), data)
    store_reading(temp)


def dump():
    """Dump measurements to file

    Returns:
        (str, int) -- filename, number of lines dumped
    """

    global readings
    global scanning_started
    global logdir
    global name

    if not os.path.exists(logdir):
        os.makedirs(logdir)

    _scan_name = "%s-%d" % (name, scanning_started)
    _filename = "%s/%s-%d.txt" % (logdir, _scan_name,
                                  len(glob.glob("%s/%s*.txt" % (logdir, _scan_name))))

    file = open(_filename, "w")
    file.write("%s\n" % (Reading.header))
    for reading in readings:
        file.write("%s\n" % (str(reading)))
    file.close()

    return (_filename, len(readings))


def store_reading(reading):
    """Store a reading for further process

    Arguments:
        reading {Reading} -- Laser scanner measurement
    """

    global readings

    if len(readings) > dump_size:
        dumped_nr = dump()
        readings = readings[dumped_nr[1] - 1:]

    readings.append(reading)


def robot_update(data):
    """Robot update handler

    Arguments:
        data {RobotState} -- Current robot state
    """

    global robot_state
    global robot_timestamp

    robot_state = data
    robot_timestamp = now()


def is_scanning_update(data):
    """Scanning mode change handler

    Arguments:
        data {Bool} -- state of the scannig
    """

    global is_scanning
    global scanning_started

    if is_scanning != data.data:
        if data.data:
            scanning_started = now()
        if not data.data:
            dump()

    is_scanning = data.data


if __name__ == "__main__":
    try:
        # init dumper
        init()
        status(name, "Listening to the scanner")
        # start listening
        rospy.spin()
    except Exception as e:
        status(name, "Dumper died: %s" % (str(e)), STATE.ERROR)
    finally:
        status(name, "Listening finished")
