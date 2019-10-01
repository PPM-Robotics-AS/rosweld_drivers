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

This package is based on Ardya Dipta's ROS/camera_image package.
"""

import argparse
import os
import sys
import time
from threading import Timer

import cv2
import numpy as np
import requests
import roslib
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String

from src.drivers.misc.status import STATE, status


def timeoutHandler():
    status("IPCamera", "Timeout, exiting...", STATE.ERROR)
    os._exit(0)


class IPCamera(object):
    def __init__(self, url, topic, user, password):
        try:
            status("IPCamera", "Trying to connect to the camera: %s" % (url))
            # setup the stream and authenticate with the username and password
            self.stream = requests.get(
                url, auth=(user, password), stream=True).raw
            status("IPCamera", "IPCamera - connected.")
        except BaseException:
            status("IPCamera", "Error: %s" % (sys.exc_info()[1]), STATE.ERROR)
            sys.exit()
        self.bytes = ''
        # publish Image topic with the selected topic name
        self.image_pub = rospy.Publisher(topic, Image, queue_size=1)
        self.bridge = CvBridge()


if __name__ == '__main__':
    rospy.loginfo("Camera Driver initializing...")

    # parse the startup arguments
    parser = argparse.ArgumentParser(
        prog='ip_camera.py',
        description='reads a given url string and dumps it to a ros_image topic')
    parser.add_argument('-g', '--gui', action='store_true',
                        help='show a GUI of the camera stream')
    parser.add_argument(
        '-u',
        '--url',
        default='',
        help='camera stream url to parse')
    parser.add_argument(
        '-t',
        '--topic',
        default='camera_image',
        help='camera stream topic name')
    parser.add_argument('-s', '--user', default='', help='user name')
    parser.add_argument('-p', '--password', default='', help='password')
    parser.add_argument('-m', '--max_fps', default='10', help='maximum fps')
    args = parser.parse_args()

    timeoutTimer = Timer(5, timeoutHandler)
    timeoutTimer.start()

    # Init IPCamera node
    rospy.init_node('IPCamera', anonymous=True)
    ip_camera = IPCamera(args.url, args.topic, args.user, args.password)

    last = 0
    while not rospy.is_shutdown():
        ip_camera.bytes += ip_camera.stream.read(1024)
        a = ip_camera.bytes.find('\xff\xd8')
        b = ip_camera.bytes.find('\xff\xd9')
        if a != -1 and b != -1:
            jpg = ip_camera.bytes[a:b + 2]
            ip_camera.bytes = ip_camera.bytes[b + 2:]
            i = cv2.imdecode(
                np.fromstring(
                    jpg,
                    dtype=np.uint8),
                cv2.IMREAD_COLOR)
            image_message = i
            now = int(round(time.time() * 1000))

            # publish images with the maximum fps
            if now - 1000 / float(args.max_fps) > last:
                ip_camera.image_pub.publish(
                    ip_camera.bridge.cv2_to_imgmsg(
                        image_message, "bgr8"))
                timeoutTimer.cancel()
                timeoutTimer = Timer(5, timeoutHandler)
                timeoutTimer.start()
                last = now

            # show the camera image on the ui
            if args.gui:
                cv2.imshow('IP Camera Publisher Cam', i)

            # wait until ESC key is pressed in the GUI window to stop it
            if cv2.waitKey(1) == 27:
                exit(0)
