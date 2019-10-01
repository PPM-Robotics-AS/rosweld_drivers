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

import rospy
from rosweld_drivers.msg import Status
from rosweld_drivers.srv import SendStatus, SendStatusResponse
from std_msgs.msg import Empty
from std_srvs.srv import EmptyResponse

# status publisher topic
publisher = None


def publish(data):
    """Publish status to ROS and log

    Arguments:
        node {string} -- node name
        msg {string} -- message

    Keyword Arguments:
        prio {int} -- priority of the status message (default: {STATE.INFO})
    """

    publisher.publish(data.status)

    return SendStatusResponse()

if __name__ == "__main__":
    rospy.init_node("logger", anonymous=True)

    rospy.Service("notify", SendStatus, publish)
    publisher = rospy.Publisher(
        "sys_state",
        Status,
        tcp_nodelay=True,
        queue_size=5,
        latch=True)
    rospy.on_shutdown(publisher.unregister)
    rospy.spin()

