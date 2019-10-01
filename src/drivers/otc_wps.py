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

import configparser
import cStringIO
import ftplib
import os
import sys
from copy import deepcopy
from ftplib import FTP
from struct import pack, unpack
from threading import Thread, currentThread

import rospy
from rosweld_drivers.msg import WeldingJobs, WeldingState
from rosweld_drivers.srv import (Input, SetJobNumber, SetJobNumberResponse,
                                 SetWeldingParameters, SetWeldingParametersResponse)
from std_srvs.srv import Empty, EmptyResponse

from src.drivers.misc.status import STATE, status
from src.drivers.misc.udp import UdpConnector

thread_config_update = None
thread_state_update = None

# UDP Handler
udp = None
# Update response received
update_response = False
# The ip address of the WPS
host = None
# The TCP port of the WPS
port = None
# Available jobs on the robot
jobs = {}
# Current job number
current_job_idx = -1
# Path of the job files on the WPS
jobs_pathname = '/A_APPLICATION/AS'
# WPS Storage
ftp = None
# Defeault file names
file_name = "ASDA1ARCW"
# Auto update of the config
auto_update = True
# Is arc on
is_arc_on = False
# Current WPS values
current_values = None
# Current job values
current_job = None
# Define driver global params
t_config = None
t_sp_params = None
t_current_params = None
# Node name
name = None

# WPS commands enum
#
# Helps to identify wps command codes


class Commands(object):
    """WPS commands enum

    Arguments:
        object {object} -- parent class
    """

    update = 0
    arc_start = 1
    arc_stop = 2
    arc_update = 3
    set_job_number = 4


class InputParams(object):
    """Input params commands enum

    Arguments:
        object {object} -- parent class
    """

    voltage = 0
    amperage = 1
    feed_speed = 2
    arc_length = 3


def parse_job(cfg, nr):
    """Parse a given config to WeldingState

    Arguments:
        cfg {dict} -- configuration dictionary
        nr {int32} -- config number

    Returns:
        WeldingState -- welding state
    """

    wp = WeldingState()
    wp.amperage = float(cfg["AST_CONDITION"]["A_CURRENT"])
    wp.job_number = nr
    wp.filler_speed = float(cfg["AST_CONDITION"]["A_FILLER_SPEED"])
    wp.mode = int(cfg["AST_CONDITION"]["A_CURRENT_OUTPUT"])
    wp.speed = int(float(cfg["AST_CONDITION"]["A_SPEED"]))

    return wp


def update_jobs():
    """Download the config files from the WPS

       Parses the downladed parameters

    Returns:
        WeldingState[] -- welding jobs
    """

    global ftp
    global jobs
    global file_name

    files = []

    try:
        files = ftp.nlst()
    except ftplib.error_perm as resp:
        status(name, "Error updating job list: %s" % (resp), STATE.ERROR)
        return jobs

    for f in files:
        filename, file_extension = os.path.splitext(f)
        if filename == file_name:
            try:
                cfg = _download_config(f)
                nr = int(file_extension[1:])
                jobs[str(nr)] = parse_job(cfg, nr)
            except Exception, e:
                status(name, "Error downloading %s: %s" % (f, str(e)), STATE.ERROR)

    return jobs


def handleUpdateResponse(r):
    """Handles the response for an update request

       Parses the message received from the robot, then publish the result

    Arguments:
        r {bytearray} -- response object
    """

    global update_response
    global t_sp_params
    global t_current_params
    global current_job_idx
    global jobs
    global is_arc_on
    global current_values
    global current_job

    job_nr = unpack('>i', r[28:32])[0]

    # publish current setpoint values
    if str(job_nr) in jobs and (
            job_nr != current_job_idx or current_job != jobs[str(job_nr)]):
        current_job = jobs[str(job_nr)]
        t_sp_params.publish(current_job)

    _is_arc_on = unpack('>i', r[24:28])[0]

    # publish current values
    old_values = deepcopy(current_values)
    current_values = WeldingState(
        amperage=unpack('>f', r[8:12])[0],
        voltage=unpack('>f', r[12:16])[0],
        filler_speed=unpack('>i', r[16:20])[0],
        default_arc_length=unpack('>i', r[20:24])[0],
        is_arc_on=_is_arc_on,
        job_number=job_nr
    )

    if old_values != current_values:
        t_current_params.publish(current_values)

    is_arc_on = True if _is_arc_on == 1 else False

    update_response = True
    current_job_idx = job_nr


def send_update():
    """Sends an update request to the robot

       Asks for the latest values from the robot

    Returns:
        EmptyResponse -- Empty
    """

    global udp
    #print "Sending update"

    msg = bytearray()
    # add command - update
    msg.extend(pack('<i', Commands.update)[::-1])

    # append message to udp queue
    udp.appendToQueue(msg, "send_update", handleUpdateResponse)

    return EmptyResponse()


def wps_state_publisher():
    """Publishes the wps state

       Runs in a different thread, polling and publishing the wps state every second
    """

    global update_response
    global name
    update_response = False
    rate = rospy.Rate(20)  # 5hz
    t = currentThread()
    timeout = 60
    while not rospy.is_shutdown() and getattr(t, "do_run", True):
        try:
            if (not update_response):
                timeout -= 1
                if timeout == 0:
                    ws = WeldingState()
                    ws.error = "No response from the WPS"
                    t_current_params.publish(ws)
            else:
                timeout = 60

            update_response = False
            thread = Thread(target=update_asynch)
            thread.start()

        except Exception as e:
            status(name, "State publish error %s" % (e), STATE.ERROR)

        rate.sleep()

    status(name, "WPS state publisher stopped")


def update_asynch():
    """Updates the wps state on a new thread

       The blocking network response is handled this way
    """

    global udp

    # If the queue already contains an update request, return
    if (udp.isQueued("send_update")):
        return

    send_update()


def set_config(data, outputfile_nr=None):
    """Set the given config on the WPS

    Arguments:
        data {WeldingState} -- data with the current configuration

    Keyword Arguments:
        outputfile_nr {int32} -- which wps config nr to use when storing the config file on the WPS (default: {None})

    Returns:
        EmptyResponse -- empty
    """

    global file_name

    #print data
    try:
        f_in = "{0}.{1}".format(
            file_name, str(
                data.params.job_number).zfill(3))
        f_out = "{0}.{1}".format(
            file_name, str(
                data.params.job_number if outputfile_nr is None else outputfile_nr).zfill(3))

        cfg = _download_config(f_in)

        if data.params.amperage != 0:
            cfg.set("AST_CONDITION", "A_CURRENT", str(data.params.amperage))
        if data.params.mode != 0:
            cfg.set("AST_CONDITION", "A_CURRENT_OUTPUT", str(data.params.mode))
        if data.params.filler_speed != 0:
            cfg.set("AST_CONDITION", "A_FILLER_CONTROL", "1")
            cfg.set(
                "AST_CONDITION", "A_FILLER_SPEED", str(
                    data.params.filler_speed))
        if data.params.speed != 0:
            cfg.set("AST_CONDITION", "A_SPEED", str(data.params.speed))

        f = open(f_out, 'w')
        cfg.write(f, space_around_delimiters=False)
        f.close()

        # Upload to FTP
        f = open(f_out, 'r')
        ftp.storlines('STOR {0}'.format(f_out), f)
    except Exception as e:
        global name
        status(name, str(e), STATE.ERROR)

        sys.exit(0)

    return SetWeldingParametersResponse()


def _download_config(f):
    """Download a config file from the WPS

    Arguments:
        f {string} -- file name

    Returns:
        dict -- configuration
    """

    global jobs_pathname
    global ftp

    ftp_reader = cStringIO.StringIO()
    ftp.retrbinary('RETR {0}/{1}'.format(jobs_pathname, f), ftp_reader.write)
    lines = ftp_reader.getvalue().split("\n")
    return lines_to_config(lines)


def lines_to_config(lines):
    """Convert string lines to config

    Arguments:
        lines {str[]} -- string list with the lines

    Returns:
        dict -- configuration
    """

    config = configparser.ConfigParser()
    config.optionxform = str

    unilines = []

    # Parse shift-jis coding to unicode
    for line in lines:
        try:
            _t = u"{0}".format(line).upper()
            unilines.append(_t)
        except BaseException:
            continue

    config.read_string("\n".join(unilines))
    return config


def set_job_number(data):
    """Set the current job number on the WPS

    Arguments:
        data {SetJobNumber} -- set job number request object

    Returns:
        EmptyResponse -- empty
    """

    global udp

    if str(data.value) not in jobs:
        status(name, "Can not set job number %s: not found" % (data.value))
        return SetJobNumberResponse()

    status(name, "Set job number: %s" % (data.value))

    # create message and start the arc
    msg = bytearray()
    msg.extend(pack('<i', Commands.set_job_number)[::-1])
    msg.extend(pack('<i', data.value)[::-1])
    # append message to udp queue
    udp.appendToQueue(msg, "send_set_job_number")

    return SetJobNumberResponse()


def arc_start(data=None):
    """Start the arc on the WPS

    Keyword Arguments:
        data {EmptyRequest} -- empty (default: {None})

    Returns:
        EmptyResponse -- empty
    """

    global udp

    if str(data.value) not in jobs:
        status(
            name,
            "Can not start arc with job number %s: not found" %
            (data.value))
        return SetJobNumberResponse()

    status(name, "Arc start called: %s with job" % (data.value))

    # create message and start the arc
    msg = bytearray()
    msg.extend(pack('<i', Commands.arc_start)[::-1])
    msg.extend(pack('<i', data.value)[::-1])
    # append message to udp queue
    udp.appendToQueue(msg, "send_arc_start")

    return SetJobNumberResponse()


def arc_stop(data=None):
    """Stop the arc on the WPS

    Keyword Arguments:
        data {EmptyRequest} -- empty (default: {None})

    Returns:
        EmptyResponse -- empty
    """

    global udp

    # create message and start the arc
    msg = bytearray()
    msg.extend(pack('<i', Commands.arc_stop)[::-1])

    # append message to udp queue
    udp.appendToQueue(msg, "send_arc_stop")

    return EmptyResponse()


def set_params(data=None):
    """Set WPS setpoint parameters

    Keyword Arguments:
        data {SetWeldingParameters} -- config parameters (default: {None})

    Returns:
        SetWeldingParametersResponse -- empty
    """

    global udp
    global host
    global jobs_pathname
    global current_job_idx
    global is_arc_on

    # Update the job only if data is available
    if data is not None:
        #print data.params
        set_config(data, 999 if is_arc_on else None)

    # create message and change job nr
    msg = bytearray()
    msg.extend(pack('<i', Commands.arc_update)[::-1])

    # append message to udp queue
    udp.appendToQueue(msg, "send_store")

    return SetWeldingParametersResponse()


def welding_config_publisher():
    """Send welding config to ROS
    """

    global jobs
    global current_job_idx
    global name
    global auto_update
    rate = rospy.Rate(1)  # 1hz
    t = currentThread()
    while not rospy.is_shutdown() and getattr(t, "do_run", True):
        try:
            old_jobs = deepcopy(jobs)

            update_jobs()
            changed = False

            for key in jobs:
                if key not in old_jobs or jobs[key] != old_jobs[key]:
                    changed = True

            if len(old_jobs) != len(jobs) or len(set(old_jobs.keys())
                                                 & set(jobs.keys())) != len(jobs)or changed:

                config = WeldingJobs()
                config.configurations = []

                for job in jobs:
                    config.configurations.append(jobs[job])

                config.current_index = current_job_idx
                config.auto_update = auto_update

                t_config.publish(config)

        except Exception as e:
            exc_type, exc_obj, exc_tb = sys.exc_info()
            fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
            status(name, "Config publish error: %s (%s: %d)" % (e, fname, exc_tb.tb_lineno), STATE.ERROR)

            sys.exit(0)

        rate.sleep()

    status(name, "WPS config publisher stopped")


def ftp_connect():
    """Initiate FTP connection to the robot
    """

    global jobs_pathname
    global ftp
    global host

    ftp = FTP(host)
    ftp.login()

    ftp.cwd(jobs_pathname)


def init():
    """Initalizes the WPS controller

       Creates all variables required to control the wps.
    """

    global udp
    global t_config
    global t_sp_params
    global t_current_params
    global host
    global port
    global thread_state_update
    global thread_config_update
    global name

    try:

        host = rospy.get_param("ip")

        if host is None:
            raise Exception("Please set the ip of the WPS: wps_ip")

        port = rospy.get_param("port", 8002)
        name = rospy.get_param("wps_name", "welding_driver")

        # init driver node
        rospy.init_node(name, anonymous=True)

        if (host is None or port is None):
            status(
                name,
                "Please set WPS ip and port through rosparam: wps_ip, wps_port",
                STATE.ERROR)
            sys.exit()

        # UDP handler
        udp = UdpConnector(host, port, name=name)

        # Connect to FTP with anonymus user
        ftp_connect()

        # define services and topics
        rospy.Service('edit_config', SetWeldingParameters, set_config)
        rospy.Service('set_job_number', SetJobNumber, set_job_number)
        rospy.Service('set_params', SetWeldingParameters, set_params)
        rospy.Service('arc_start', SetJobNumber, arc_start)
        rospy.Service('arc_stop', Empty, arc_stop)

        t_config = rospy.Publisher(
            'jobs', WeldingJobs, queue_size=1, latch=True)
        t_sp_params = rospy.Publisher(
            'current_set_points',
            WeldingState,
            queue_size=1,
            latch=True)
        t_current_params = rospy.Publisher(
            'current_params', WeldingState, queue_size=1, latch=True)

        thread_state_update = Thread(target=wps_state_publisher)
        thread_state_update.do_run = True
        thread_state_update.start()

        thread_config_update = Thread(target=welding_config_publisher)
        thread_config_update.do_run = True
        thread_config_update.start()

        status(name, "OTC WPS - ready")

        # Wait for service calls or user interruption
        rospy.spin()
    except Exception as e:
        status(name, "OTC WPS Error: %s" % (str(e)), STATE.ERROR)

    finally:
        udp.stopConsumeThread()

        if thread_state_update is not None:
            thread_state_update.do_run = False
            thread_state_update.join()

        if thread_config_update is not None:
            thread_config_update.do_run = False
            thread_config_update.join()

        status("welding_driver", "OTC WPS - stopped")


if __name__ == "__main__":
    init()
