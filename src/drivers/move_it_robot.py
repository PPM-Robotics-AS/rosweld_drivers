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
from math import degrees
from threading import Thread, currentThread

import moveit_commander
import moveit_msgs.msg
import numpy as np
import rospy
import rosweld_drivers.srv
from rosweld_drivers.msg import RobotState
import tf
from std_srvs.srv import Empty, EmptyResponse
from src.drivers.misc.status import STATE, status

# publish state thread
thread = None
#global speed
speed = 10.0
# last robot state
lastState = None
# the robot is moving
isMoving = False
# Stored poses
allPositions = []
# node name
name = None
# last step
lastStep = -1
#move group commander
group = None
#pose publisher
p_pose = None
#min distance between points
min_d = 0
#send update even it is the same as before
do_update = True

def getPositions():
    """Get positions

    Returns:
        Pose -- allPositions
    """

    global allPositions
    return allPositions


def sendPlay(start, end, d, poses=None):
    """Moves the robot between two given poses

    Arguments:
        start {int32} -- start index
        end {int32} -- end index
        d {int32} -- direction 1: forward, -1: backward

    Keyword Arguments:
        poses {Move[]} -- set of poses, if None, use allPositions (default: {None})
    """

    # The last step along the multiPath
    global lastStep
    global allPositions
    global speed
    global name

    if poses is None:
        poses = allPositions

    lastStep = -1

    # Create plan
    plan = moveit_msgs.msg.RobotTrajectory()
    # Used to append plans with different speeds
    currentMove = 0
    # Going through all moves
    waypoints = []

    for currentMove in range(min(start, end), max(start, end) + 1)[::d]:
        p = poses[currentMove]
        waypoints.append(p)

    # Set planning start to the last waypoint - in case it exists
    group.set_start_state_to_current_state()

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0
    # disabling:
    (plan, fraction) = group.compute_cartesian_path(
        waypoints,   # waypoints to follow
        0.01,        # eef_step
        0.0)         # jump_threshold

    if fraction < sys.float_info.epsilon:
        status(
            name,
            "Can't follow path (fraction: %d)" %
            (fraction),
            STATE.ERROR)
        return

    # Divide the timing using the given speed for this plan
    for p in plan.joint_trajectory.points:
        p.time_from_start /= speed

    # Going through the multiPlan sync
    group.execute(plan, wait=False)

    status(name, "Play sent from %d to %d with direction: %d" %
           (start, len(allPositions) if end == -1 else end, d))


def move_along(req):
    """Moves along a set of points

       Expects a list of Moves, that specify the Pose and the speed of the current move

    Arguments:
        req {MoveAlongRequest} -- set of poses

    Returns:
        MoveAlongResponse -- Empty
    """

    # Save all positions of the request for later use
    store_poses(req)

    if len(req.moves) == 0:
        return rosweld_drivers.srv.MoveAlongResponse()

    sendPlay(0, len(req.moves) - 1, 1)

    return rosweld_drivers.srv.MoveAlongResponse()


def move_pose(req):
    """Moves to a set of poses without storing them

    Arguments:
        req {MoveAlongRequest} -- set of poses

    Returns:
        MoveAlongResponse -- Empty
    """

    if len(req.moves) == 0:
        return rosweld_drivers.srv.MoveAlongResponse()

    poses = []
    for i in range(0, len(req.moves)):
        poses.append(req.moves[i].pose)

    sendPlay(0, len(req.moves) - 1, 1, poses)

    return rosweld_drivers.srv.MoveAlongResponse()


def abort(req):
    """Aborts the current movement

       Stops the robot executing the current movement

    Arguments:
        req {EmptyRequest} -- Empty

    Returns:
        EmptyResponse -- Empty
    """
    global name
    group.stop()
    status(name, "Abort sent.")
    return EmptyResponse()


def robot_pose_publisher():
    """Publishes the robot pose

       Runs in a different thread, polling and publishing the robot pose every second
    """

    rate = rospy.Rate(5)  # 1hz
    t = currentThread()
    global lastState
    global isMoving
    global do_update

    while not rospy.is_shutdown() and getattr(t, "do_run", True):
        state = RobotState()

        state.pose = group.get_current_pose().pose
        state.step = get_robot_step(state.pose.position)
        state.joints = group.get_current_joint_values()

        rpy = tf.transformations.euler_from_quaternion(
            np.array([
                state.pose.orientation.x,
                state.pose.orientation.y,
                state.pose.orientation.z,
                state.pose.orientation.w]))

        isMoving = lastState is not None and state.pose != lastState.pose
        state.euler_angles.rx = degrees(rpy[0])
        state.euler_angles.ry = degrees(rpy[1])
        state.euler_angles.rz = degrees(rpy[2])
        state.isMoving = isMoving

        if lastState != state or do_update:
            p_pose.publish(state)
            do_update = False

        lastState = state

        rate.sleep()

def get_min_distance():
    global allPositions

    if len(allPositions) == 0:
        return 0

    min_d = sys.float_info.max

    for i in range(len(allPositions) - 1):
        pos1 = allPositions[i].position
        pos2 = allPositions[i + 1].position

        p1 = np.array([pos1.x, pos1.y, pos1.z])
        p2 = np.array([pos2.x, pos2.y, pos2.z])

        squared_dist = np.sum(p1**2 + p2**2, axis=0)
        dist = np.sqrt(squared_dist)

        min_d = min(min_d, dist)

    return min_d

def get_robot_step(currentPos):
    """Publishes the current approximate step on the path

       Searches for the closest point on the given path and publishes it's index
       The algorithm searches in a circle, trying to find a point after the current one

    Arguments:
        currentPos {Position} -- current position x,y,z

    Returns:
        int32 -- current pose index in the path
    """

    global lastStep
    global allPositions
    global min_d

    try:
        minD = 9999
        minI = -1

        inThreshold = False

        if len(allPositions) != 0:
            posCount = len(allPositions)
            for i in range(0, posCount):
                # Go in a circle, starting from the current position
                i = (lastStep + i) % posCount
                p = allPositions[i].position
                # The distance of the current robot pose and the checked pose
                d = abs(p.x - currentPos.x) + \
                    abs(p.y - currentPos.y) + abs(p.z - currentPos.z)

                # Marking a new best match
                if d < minD and d < min_d / 2:
                    minD = d
                    minI = i

    # Catches, if the allPositions is not yet defined
    except NameError as e:
        global name
        status(name, e, STATE.ERROR)

    lastStep = minI

    return minI


def set_speed(req):
    """Handler for the set speed service request

       Sets the global moveit speed

    Arguments:
        req {SetSpeedRequest} -- request object, containing the value

    Returns:
        SetSpeedResponse -- response objet, empty
    """

    global speed
    global name
    speed = req.value
    status(name, "SetSpeed sent: %d." % (req.value))
    return rosweld_drivers.srv.SetSpeedResponse()


def move_between(req):
    """Handler for move between service request

       Moves the robot between two poses

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

    resp = "OK"

    d = -1 if req.start > req.end else 1
    sendPlay(req.start, req.end, d)

    return resp


def store_poses(req):
    """Handler for store_poses service request

       Stores the poses locally

    Arguments:
        req {MoveAlongRequest} -- request object, containing the moves

    Returns:
        MoveAlongResponse -- response object, empty
    """

    global allPositions
    global min_d

    allPositions = []
    for i in range(0, len(req.moves)):
        allPositions.append(req.moves[i].pose)

    min_d = get_min_distance()

    return rosweld_drivers.srv.MoveAlongResponse()

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

    global thread
    global name

    name = rospy.get_param("robot_name", "move_it_robot")
    rospy.init_node(name, anonymous=True)

    try:
        rospy.wait_for_service('move_group/load_map', 1)
    except rospy.exceptions.ROSException:
        status(name, "Please start MoveIt first", STATE.ERROR)
        rospy.wait_for_service('move_group/load_map')

    moveit_commander.roscpp_initialize(sys.argv)
    moveit_commander.RobotCommander()
    moveit_commander.PlanningSceneInterface()
    global group
    group_name = rospy.get_param("move_group_name", "welding")
    group = moveit_commander.MoveGroupCommander(group_name)

    # # Registering services
    rospy.Service('move_along', rosweld_drivers.srv.MoveAlong, move_along)
    rospy.Service('abort', Empty, abort)
    rospy.Service('update', Empty, update)
    rospy.Service('store_poses', rosweld_drivers.srv.MoveAlong, store_poses)
    rospy.Service('move_pose', rosweld_drivers.srv.MoveAlong, move_pose)
    rospy.Service('set_speed', rosweld_drivers.srv.SetSpeed, set_speed)
    rospy.Service(
        'move_between',
        rosweld_drivers.srv.MoveBetween,
        move_between)

    # Registering publishers
    global p_pose
    p_pose = rospy.Publisher(
        'robot_state',
        RobotState,
        queue_size=100,
        latch=True)

    status(name, "MoveIt Robot Driver - ready (%s)" % (name))

    # Starts the robot pose publisher on a new thread
    thread = Thread(target=robot_pose_publisher)
    thread.do_run = True
    thread.start()

    try:
        # Wait for service calls or user interruption
        rospy.spin()

    finally:
        thread.do_run = False
        thread.join()

        status(name, "MoveIt Robot Driver - stopped")


if __name__ == "__main__":
    init()
