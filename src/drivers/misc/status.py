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

import datetime

import rospy
from rosweld_drivers.msg import Status
from rosweld_drivers.srv import SendStatus


class STATE(object):
    ERROR = 2
    INFO = 1
    DEBUG = 0
    WARNING = 3

def status(node, msg, prio=STATE.INFO, *args, **kwargs):
    """Publish status to ROS and log

    Arguments:
        node {string} -- node name
        msg {string} -- message

    Keyword Arguments:
        prio {int} -- priority of the status message (default: {STATE.INFO})
    """

    msg = Status(
        message="[%s] %s - %s" % (node, datetime.datetime.now(), msg),
        node=node,
        priority=prio)

    try:
        if prio == 0:
            rospy.logdebug(msg.message)
        elif prio == 1:
            rospy.loginfo(msg.message)
        else:
            rospy.logerr(msg.message)

        srv = rospy.ServiceProxy("/notify", SendStatus)
        srv.call(msg)
        rospy.sleep(1)
    except Exception:
        return
