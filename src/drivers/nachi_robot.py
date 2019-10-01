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

import sys
from math import degrees, pi, radians
from struct import pack, unpack
from threading import Thread, currentThread

import numpy as np
import rospy
import tf
import rosweld_drivers.srv
from rosweld_drivers.msg import RobotState
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty, EmptyResponse

from src.drivers.misc.status import STATE, status
from src.drivers.misc.udp import UdpConnector

# The ip address of the robot
host = None
# The TCP port of the robot
port = {'update': None, 'command': None}
# Robot update thread
update_thread = None
# The Eueler angles convention used by the robot
axes = 'szxy'
# The last pose received from the robot
last_state = None
# Amount of the stored poses
stored_pose_count = 0
# UDP Handler
udp = {'update': None, 'command': None}
# Update response received
update_response = False
# Stored poses
allPositions = []
# batch size
batchSize = 5
# node name
name = None
# joint state publisher
p_joint_states = None
# robot state publisher
p_robot_state = None
#send update even it is the same as before
do_update = True


class Commands(object):
    """Robot commands enum

       Helps to identify robot command codes
    """

    playback = 1
    store = 2
    update = 3
    setSpeed = 4
    abort = 5
    setPose = 6


class RobotStates(object):
    """Two-way dictionary for robot states

    Two way translation of robot states

    Arguments:
        object {object} -- parent class
    """

    unknown = 0
    playbackStarted = 1
    playbackFinished = 2
    waitingForCommand = 3

    @staticmethod
    def toString(value):
        return [
            "Unknown",
            "PlaybackStarted",
            "PlaybackFinished",
            "WaitingForCommand"][value]


def getPositions():
    """Get positions

    Returns:
        Pose -- allPositions
    """

    global allPositions
    return allPositions


def getPathVectorAngle(pos):
    """Calculates the 7. axis (J1) angle, because NACHI can not handle
       xyzrpy position with 7 angle, just with 6

    Arguments:
        pos {Position} -- the x,y,z position

    Returns:
        float -- the angle
    """

    tan = 0.0

    if pos.x != 0:
        tan = pos.y / pos.x

    angle = np.arctan(tan)

    return degrees(angle)


def sendPoses(poseList):
    """Sends a list of poses to the robot

       Sends the poses in batches, to speed up the transfer

    Arguments:
        poseList {Move[]} -- move list
    """

    global udp
    global allPositions
    global batchSize
    sentPoses = 0
    poses = len(poseList)
    allPositions = []
    for i in range(0, poses):
        allPositions.append(poseList[i].pose)

    while poses != sentPoses:
        msg = bytearray()
        _tbs = 0

        if batchSize > poses - sentPoses:
            _tbs = poses - sentPoses
        else:
            _tbs = batchSize

        # add command - store
        msg.extend(pack('<i', Commands.store)[::-1])
        #not used
        msg.extend(pack('<i', 1)[::-1])
        # add this batch size
        msg.extend(pack('<i', _tbs)[::-1])
        # add start pose idx
        msg.extend(pack('<i', sentPoses + 1)[::-1])
        # add pose bytes
        for i in range(sentPoses, sentPoses + _tbs):
            p = poseList[i].pose
            pos = p.position

            msg.extend(pack('<f', round(pos.x * 1000, 3))[::-1])
            msg.extend(pack('<f', round(pos.y * 1000, 3))[::-1])
            msg.extend(pack('<f', round(pos.z * 1000, 3))[::-1])

            (rx, ry, rz) = tf.transformations.euler_from_quaternion(np.array(
                [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]), axes)

            msg.extend(pack('<f', round(degrees(rx), 3))[::-1])
            msg.extend(pack('<f', round(degrees(ry), 3))[::-1])
            msg.extend(pack('<f', round(degrees(rz), 3))[::-1])

            # add an angle pointing to the path
            msg.extend(pack('<f', round(getPathVectorAngle(pos), 3))[::-1])

        udp['command'].appendToQueue(msg, "send_poses")
        sentPoses += _tbs

    status(name, "%d poses sent to queue" % (len(allPositions)))


def sendPlay(start=1, end=-1, d=1):
    """Moves the robot between two given poses

    Arguments:
        start {int32} -- start index
        end {int32} -- end index
        d {int32} -- direction 1: forward, -1: backward

    Keyword Arguments:
        poses {Move[]} -- set of poses, if None, use allPositions (default: {None})
    """

    global udp

    msg = bytearray()

    # add command - play
    msg.extend(pack('<i', Commands.playback)[::-1])
    # start index
    msg.extend(pack('<i', start)[::-1])
    # end index
    msg.extend(pack('<i', end)[::-1])
    # direction
    msg.extend(pack('<i', d)[::-1])

    udp['command'].appendToQueue(msg, "send_play")
    status(name, "Play sent from %d to %d with direction: %d" %
           (start, len(allPositions) if end == -1 else end, d))


def sendUpdate():
    """Send update request to the robot

       Asks for the latest values from the robot
    """

    #print "Sending update"
    global udp

    # If the queue already contains an update request, return
    if udp["update"].isQueued("send_update"):
        return

    msg = bytearray()

    # add command - update
    msg.extend(pack('<i', Commands.update)[::-1])

    udp["update"].appendToQueue(msg, "send_update", handleUpdateResponse)
    #print "Update sent. Waiting for response..."


def handleUpdateResponse(r):
    """Handles the response for an update request

      Parses the message received from the robot, then publish the result

    Arguments:
        r {bytearray} -- response from the robot

    Returns:
        RobotState -- state object
    """

    global last_state
    global update_response
    global stored_pose_count
    global name
    global do_update
    global p_joint_states
    global p_robot_state

    try:

        i = 8

        state = rosweld_drivers.msg.RobotState()

        state.speed = unpack('>f', r[i:i + 4])[0]
        i = i + 4
        state.pose.position.x = unpack('>f', r[i:i + 4])[0] / 1000
        i = i + 4
        state.pose.position.y = unpack('>f', r[i:i + 4])[0] / 1000
        i = i + 4
        state.pose.position.z = unpack('>f', r[i:i + 4])[0] / 1000
        i = i + 4
        state.euler_angles.rx = unpack('>f', r[i:i + 4])[0]
        i = i + 4
        state.euler_angles.ry = unpack('>f', r[i:i + 4])[0]
        i = i + 4
        state.euler_angles.rz = unpack('>f', r[i:i + 4])[0]
        i = i + 4
        orientation = tf.transformations.quaternion_from_euler(
            radians(state.euler_angles.rz),
            radians(state.euler_angles.ry),
            radians(state.euler_angles.rx),
            axes)
        state.pose.orientation.x = orientation[0]
        state.pose.orientation.y = orientation[1]
        state.pose.orientation.z = orientation[2]
        state.pose.orientation.w = orientation[3]
        # -1 from the step index, because NACHI's list indexing is from 1, not zero
        state.step = unpack('>i', r[i:i + 4])[0] - 1
        i = i + 4
        state.storedPoses = unpack('>i', r[i:i + 4])[0]
        i = i + 4
        stateId = unpack('>i', r[i:i + 4])[0]
        state.robotProgramState = RobotStates.toString(stateId)
        i = i + 4
        state.mode = ["Teach", "Playback",
                      "High-speed Teach"][unpack('>i', r[i:i + 4])[0]]
        for _ in range(7):
            i = i + 4
            state.joints.append(unpack('>f', r[i:i + 4])[0])

        #print "Update received"
        #print seq_back, fingerY, currentMove, speed, RobotStates.toString(robotProgramState)

        update_response = True
        state.isMoving = last_state is not None and last_state.pose != state.pose

        if last_state != state or do_update:
            p_robot_state.publish(state)
            js = JointState()
            js.name = [
                "joint_1",
                "joint_2",
                "joint_7",
                "joint_3",
                "joint_4",
                "joint_5",
                "joint_6",
                "joint_tip"]
            js.position = [
                state.joints[0],
                -1 * (state.joints[1] - pi / 2),
                state.joints[6],
                -1 * state.joints[2],
                state.joints[3],
                -1 * state.joints[4],
                state.joints[5],
                0.0
            ]
            p_joint_states.publish(js)

            do_update = False

        last_state = state
        stored_pose_count = state.storedPoses

    except Exception as e:
        status(name, "Robot update failed: %s" % (str(e)), STATE.ERROR)

    return state


def sendAbort():
    """Aborts the current movement

       Stops the robot executing the current movement
    """

    global udp

    msg = bytearray()

    # add command - abort
    msg.extend(pack('<i', Commands.abort)[::-1])

    udp['command'].appendToQueue(msg, "send_abort")
    status(name, "Abort sent.")


def sendSetSpeed(value):
    """Sends a set speed command to the robot

       Sets the global movement speed of the robot

    Arguments:
        value {int32} -- speed
    """

    global udp

    msg = bytearray()

    # add command
    msg.extend(pack('<i', Commands.setSpeed)[::-1])
    # add speed
    msg.extend(pack('<i', value)[::-1])

    udp['command'].appendToQueue(msg, "send_set_speed")
    status(name, "SetSpeed sent: %d." % (value))


def move_between(req):
    """Handler for move between service request

       Moves along the robot between two poses

    Arguments:
        req {MoveBetweenRequest} -- request object, containing the start and stop index

    Returns:
        MoveBetweengResponse -- response object, empty
    """

    resp = rosweld_drivers.srv.MoveBetweenResponse()

    print req, len(allPositions)

    if req.start < 0 or req.start >= len(
            allPositions) or req.end < 0 or req.end >= len(allPositions):
        resp.result = "Invalid move"
        global name
        status(
            name,
            "Invalid move, the selected steps has to be between 0 and %d" %
            (len(allPositions)),
            STATE.ERROR)
        return resp

    resp.result = "OK"
    # Add +1 to the indexes, because NACHI's list is indexed from 1, not zero
    sendPlay(req.start + 1, req.end + 1, -1 if req.end < req.start else 1)

    return resp


def store_poses(req):
    """Handler for store_poses service request

       Stores the poses locally

    Arguments:
        req {MoveAlongRequest} -- request object, containing the moves

    Returns:
        MoveAlongResponse -- response object, empty
    """

    global last_state

    response = rosweld_drivers.srv.MoveAlongResponse()
    response.result = "OK"

    if len(req.moves) == 0:
        response.result = "No moves, exiting."
        return response

    if last_state is not None and last_state.isMoving:
        response.result = "The robot is moving, can't accept new poses."
        return response

    sendPoses(req.moves)

    return response


def move_along(req):
    """Moves along a set of points

       Expects a list of Moves, that specify the Pose and the speed of the current move

    Arguments:
        req {MoveAlongRequest} -- set of poses

    Returns:
        MoveAlongResponse -- Empty
    """

    global last_state

    response = rosweld_drivers.srv.MoveAlongResponse()
    response.result = "OK"

    if len(req.moves) == 0:
        response.result = "No moves, exiting."
        return response

    if last_state is not None and last_state.isMoving:
        response.result = "The robot is moving, can't accept new poses."
        return response

    sendPoses(req.moves)
    sendPlay()

    return response


def move_pose(req):
    """Moves to a set of poses without storing them

    Arguments:
        req {MoveAlongRequest} -- set of poses

    Returns:
        MoveAlongResponse -- Empty
    """

    if len(req.moves) == 0:
        return rosweld_drivers.srv.MoveAlongResponse()

    pose = req.moves[0].pose
    idx = -1
    if len(allPositions) > 0:
        for i, p in enumerate(allPositions):
            if p == pose:
                idx = i + 1

    global udp
    #print "Sending poses"

    msg = bytearray()
    # add command - store
    msg.extend(pack('<i', Commands.setPose)[::-1])
    #not used
    msg.extend(pack('<i', 1)[::-1])
    # add pose
    # add position xyz
    msg.extend(pack('<f', pose.position.x * 1000)[::-1])
    msg.extend(pack('<f', pose.position.y * 1000)[::-1])
    msg.extend(pack('<f', pose.position.z * 1000)[::-1])

    (rx, ry, rz) = tf.transformations.euler_from_quaternion(np.array(
        [pose.orientation.x,
         pose.orientation.y,
         pose.orientation.z,
         pose.orientation.w]), axes)

    # set orientation rpy
    msg.extend(pack('<f', degrees(rx))[::-1])
    msg.extend(pack('<f', degrees(ry))[::-1])
    msg.extend(pack('<f', degrees(rz))[::-1])

    # add an angle pointing to the path
    msg.extend(pack('<f', getPathVectorAngle(pose.position))[::-1])

    rospy.loginfo(
        "Goint to pose (%d): x: %d, y: %d, z: %d, r: %d, p: %d, y: %d, J1: %d" %
        (idx,
         pose.position.x *
         1000,
         pose.position.y *
         1000,
         pose.position.z *
         1000,
         degrees(rx),
         degrees(ry),
         degrees(rz),
         getPathVectorAngle(pose.position)))

    # set current move index
    msg.extend(pack('<i', idx)[::-1])

    udp['command'].appendToQueue(msg, "set_pose")

    status(name, "Pose sent to queue")

    return rosweld_drivers.srv.MoveAlongResponse()


def abort(req):
    """Aborts the current movement

       Stops the robot executing the current movement

    Arguments:
        req {EmptyRequest} -- Empty

    Returns:
        EmptyResponse -- Empty
    """

    sendAbort()
    return EmptyResponse()


def set_speed(req):
    """Handler for the set speed service request

       Sets the global moveit speed

    Arguments:
        req {SetSpeedRequest} -- request object, containing the value

    Returns:
        SetSpeedResponse -- response objet, empty
    """

    sendSetSpeed(req.value)
    return rosweld_drivers.srv.SetSpeedResponse()


def robot_state_publisher():
    """Publishes the robot pose

       Runs in a different thread, polling and publishing the robot pose every second
    """

    global update_response
    update_response = False
    global last_state
    global name
    last_state = None
    rate = rospy.Rate(5)  # 5Hz
    t = currentThread()
    timeout = 10
    while not rospy.is_shutdown() and getattr(t, "do_run", True):
        try:
            if not update_response:
                timeout -= 1
                if timeout == 0:
                    status(name, "No response from the robot", STATE.ERROR)
                    timeout = 10
            else:
                timeout = 10

            update_response = False
            sendUpdate()

        except Exception as e:
            status(name, e, STATE.ERROR)

        rate.sleep()

    status(name, "Robot state publisher stopped")

def update(req):
    """Request update

    Arguments:
        req {EmptyRequest} -- Empty

    Returns:
        EmptyResponse -- Empty
    """
    global do_update
    do_update = True
    return EmptyResponse()

def init():
    """ Initalizes the robot controller

        Creates all variables required to control the robot.
        Moves the robot to a neutral position.
    """

    global udp
    global host
    global port
    global update_thread
    global name
    try:

        host = rospy.get_param("robot_ip")

        if host is None:
            raise Exception("Please set the ip of the robot: robot_ip")

        port["update"] = rospy.get_param("robot_update_port", 8001)
        port["command"] = rospy.get_param("robot_command_port", 8000)
        name = rospy.get_param("robot_name", "nachi_robot")

        rospy.init_node(name, anonymous=True)

        udp['command'] = UdpConnector(host, port["command"], name=name)
        udp['update'] = UdpConnector(host, port["update"], name=name)

        # Registering services
        rospy.Service(
            'store_poses',
            rosweld_drivers.srv.MoveAlong,
            store_poses)
        rospy.Service('move_along', rosweld_drivers.srv.MoveAlong, move_along)
        rospy.Service('abort', Empty, abort)
        rospy.Service('update', Empty, update)
        rospy.Service('set_speed', rosweld_drivers.srv.SetSpeed, set_speed)
        rospy.Service(
            'move_between',
            rosweld_drivers.srv.MoveBetween,
            move_between)
        rospy.Service('move_pose', rosweld_drivers.srv.MoveAlong, move_pose)

        # Registering publishers
        global p_robot_state
        global p_joint_states

        p_robot_state = rospy.Publisher(
            'robot_state', RobotState, queue_size=10, latch=True)
        p_joint_states = rospy.Publisher(
            'robot_controller_joint_state',
            JointState,
            queue_size=10,
            latch=True)

        status(name, "NACHI Robot Driver - ready (%s)" % (name))

        # Starts the robot pose publisher on a new thread
        update_thread = Thread(target=robot_state_publisher)
        update_thread.do_run = True
        update_thread.start()

        # Wait for service calls or user interruption
        rospy.spin()
    finally:
        if update_thread is not None:
            update_thread.do_run = False
            update_thread.join()

        if 'command' in udp and udp['command'] is not None:
            udp['command'].stopConsumeThread()

        if 'update' in udp and udp['update'] is not None:
            udp['update'].stopConsumeThread()

        status(name, "NACHI Robot Driver - stopped (%s)" % (name))


if __name__ == "__main__":
    init()
