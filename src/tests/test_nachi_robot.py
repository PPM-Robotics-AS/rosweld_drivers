from mock import patch
import socket
import rospy
import sys
from struct import pack, unpack
import time
import math
from threading import Thread, currentThread
from sensor_msgs.msg import JointState
import src.drivers.nachi_robot
from std_srvs.srv import EmptyRequest
from rosweld_drivers.msg import Move, RobotState
from rosweld_drivers.srv import MoveAlongRequest, MoveBetweenRequest, SetSpeedRequest
from ..drivers.misc.udp import UdpConnector
from mocks import *
from sensor_msgs.msg import JointState
import src.drivers.misc.status

sendPlayCallParams = None

def mock_send_play(start = 1, end = -1, d = 1, poses = None):
    """Mock the send play command
    
    Keyword Arguments:
        start {int} -- start index of the step (default: {1})
        end {int} -- end index of the step (default: {-1})
        d {int} -- direction: 1 - forward, -1 - backward (default: {1})
        poses {Move[]} -- Moves to follow (default: {None})
    """

    global sendPlayCallParams
    sendPlayCallParams = { 'start': start, 'end': end, 'direction': d, 'poses': poses }

def mock_appendToQueue_setspeed(self, msg, label, handler = None):
    """Mock UDP's appendToQueue and check for setting the speed
    
    Arguments:
        msg {bytearray} -- the message object as a bytearray
        label {string} -- label of the message
    
    Keyword Arguments:
        handler {function} -- callback function (default: {None})
    """

    command = unpack('>i',msg[0:4])[0]

    assert label == "send_set_speed" 
    assert command == src.drivers.nachi_robot.Commands.setSpeed 

def mock_appendToQueue_abort(self, msg, label, handler = None):
    """Mock UDP's appendToQueue and check for calling the abort
    
    Arguments:
        msg {bytearray} -- the message object as a bytearray
        label {string} -- label of the message
    
    Keyword Arguments:
        handler {function} -- callback function (default: {None})
    """
    command = unpack('>i',msg[0:4])[0]

    assert label == "send_abort" 
    assert command == src.drivers.nachi_robot.Commands.abort 

def mock_appendToQueue_sendupdate(self, msg, label, handler = None):
    """Mock UDP's appendToQueue and check for requesting update
    
    Arguments:
        msg {bytearray} -- the message object as a bytearray
        label {string} -- label of the message
    
    Keyword Arguments:
        handler {function} -- callback function (default: {None})
    """
    command = unpack('>i',msg[0:4])[0]
    check_command(src.drivers.nachi_robot.Commands, command)

    assert label == "send_update"
    assert command == src.drivers.nachi_robot.Commands.update

class TestNachiRobot(object):
    """Unit tests for the NACHI Robot driver
    """

    def get_moves(self, count):
        """Generate moves list
        
        Arguments:
            count {int} -- Amount of the generated moves
        
        Returns:
            MoveAlongRequest -- Moves list
        """

        moves = []
        req = MoveAlongRequest()

        for i in range(0, count):
            moves.append(Move())

        req.moves = moves
        return req

    def test_store_poses(self):
        """Test the store poses functionality

        Requirements:
        - The driver has to store all of the given moves.
        - Only stores the last moves list, drops the old one.
        - Sends the moves in batch to the robot with UDP
        """

        # generate 500 moves
        req = self.get_moves(500)

        with patch.object(socket.socket, 'connect', return_value=True) as mock_connect, \
            patch.object(socket.socket, 'bind', return_value=True) as mock_bind:
            src.drivers.nachi_robot.udp["command"] = UdpConnector("localhost", 8000)
            src.drivers.nachi_robot.udp["update"] = UdpConnector("localhost", 8000)

        with patch.object(UdpConnector, 'appendToQueue') as u:

            # call store poses 
            src.drivers.nachi_robot.store_poses(req)

            # the driver has to return with the same amount of moves
            assert len(src.drivers.nachi_robot.getPositions()) == len(req.moves)

            # the amount of the udp calls is the smallest integer which is greater than 
            # the length of the move list / batch size
            assert u.call_count ==  math.ceil( len(req.moves) / src.drivers.nachi_robot.batchSize )

            # call store poses again
            src.drivers.nachi_robot.store_poses(req)

            # the old poses has to be removed
            assert len(src.drivers.nachi_robot.getPositions()) != 2 * len(req.moves)

        #stop udp threads
        src.drivers.nachi_robot.udp["command"].stopConsumeThread()
        src.drivers.nachi_robot.udp["update"].stopConsumeThread()

    def test_move_between(self):
        """Test move between function

        Requirements:
        - Work with stored poses
        - The robot side driver's indexing is +1
        - The direction is based on the start and end index, 1 - forward, -1 backward
        """

        global sendPlayCallParams

        # get moves
        req = self.get_moves(50)

        with patch.object(socket.socket, 'connect', return_value=True) as mock_connect, \
            patch.object(socket.socket, 'bind', return_value=True) as mock_bind:
            src.drivers.nachi_robot.udp["command"] = UdpConnector("localhost", 8000)
            src.drivers.nachi_robot.udp["update"] = UdpConnector("localhost", 8000)

        with patch('src.drivers.nachi_robot.sendPlay', side_effect = mock_send_play) as m, \
             patch.object(UdpConnector, 'appendToQueue') as u:

            src.drivers.nachi_robot.allPositions = []
            # calling the move_between without stored poses will return without performing the action
            ret = src.drivers.nachi_robot.move_between(MoveBetweenRequest( start = 2, end = 31 ))
            assert len(src.drivers.nachi_robot.allPositions) == 0 and ret.result != "OK"
            
            # store poses
            src.drivers.nachi_robot.store_poses(req)
            assert u.call_count == len(req.moves) / src.drivers.nachi_robot.batchSize

            # call move between the 2nd and 3rd step 
            src.drivers.nachi_robot.move_between(MoveBetweenRequest( start = 2, end = 3 ))
            
            assert sendPlayCallParams['start'] == 3
            assert sendPlayCallParams['end'] == 4
            assert sendPlayCallParams['direction'] == 1
            assert sendPlayCallParams['poses'] == None
            assert m.called

            # call move between the 3nd and 1rd step (backward)
            src.drivers.nachi_robot.move_between(MoveBetweenRequest( start = 3, end = 1 ))
            
            assert sendPlayCallParams['start'] == 4
            assert sendPlayCallParams['end'] == 2
            assert sendPlayCallParams['direction'] == -1
            assert sendPlayCallParams['poses'] == None
            assert m.call_count == 2

        # stop udp threads
        src.drivers.nachi_robot.udp["command"].stopConsumeThread()
        src.drivers.nachi_robot.udp["update"].stopConsumeThread()

    def test_move_along(self):
        """Test to move along a given path
        
        Requirements:
        - Uploads the moves to the robot
        - Go through the path from 0->len(allPositions)
        """

        global sendPlayCallParams
        
        # generate moves
        req = self.get_moves(5)

        with patch.object(socket.socket, 'connect', return_value=True) as mock_connect, \
            patch.object(socket.socket, 'bind', return_value=True) as mock_bind:
            src.drivers.nachi_robot.udp["command"] = UdpConnector("localhost", 8000)
            src.drivers.nachi_robot.udp["update"] = UdpConnector("localhost", 8000)
        
        with patch('src.drivers.nachi_robot.sendPlay', side_effect = mock_send_play) as m, \
             patch.object(UdpConnector, 'appendToQueue') as u:

            # upload the moves and start the movement
            src.drivers.nachi_robot.move_along(req)

            assert m.called
            assert m.call_count == 1
            
            # start from the 1st point
            assert sendPlayCallParams['start'] == 1

            # go 'til the end
            assert sendPlayCallParams['end'] == -1

            # go forward on the path
            assert sendPlayCallParams['direction'] == 1
            assert sendPlayCallParams['poses'] == None

        src.drivers.nachi_robot.udp["command"].stopConsumeThread()
        src.drivers.nachi_robot.udp["update"].stopConsumeThread()

    def test_move_pose(self):
        """Go to a specific move/pose

        Requirements:
        - go to the robot with 1 move
        """

        global sendPlayCallParams
        
        # generate a move
        req = self.get_moves(1)
        
        with patch.object(socket.socket, 'connect', return_value=True) as mock_connect, \
            patch.object(socket.socket, 'bind', return_value=True) as mock_bind:
            src.drivers.nachi_robot.udp["command"] = UdpConnector("localhost", 8000)
            src.drivers.nachi_robot.udp["update"] = UdpConnector("localhost", 8000)

        with patch.object(UdpConnector, 'appendToQueue') as u:
            
            # command the driver to move the robot
            src.drivers.nachi_robot.move_pose(req)

            # the command is forwarded through UDP
            assert u.call_count == 1

        src.drivers.nachi_robot.udp["command"].stopConsumeThread()
        src.drivers.nachi_robot.udp["update"].stopConsumeThread()

    def test_set_speed(self):
        """Set the speed on the robot

        Requirements:
        - the set speed command is forwarded to the robot
        """

        with patch.object(socket.socket, 'connect', return_value=True) as mock_connect, \
            patch.object(socket.socket, 'bind', return_value=True) as mock_bind, \
            patch.object(src.drivers.nachi_robot, 'sendSetSpeed') as setSpeed, \
            patch.object(UdpConnector, 'appendToQueue', mock_appendToQueue_setspeed):

            src.drivers.nachi_robot.udp["command"] = UdpConnector("localhost", 8000)
            src.drivers.nachi_robot.udp["update"] = UdpConnector("localhost", 8000)

            # create the command
            req = SetSpeedRequest()
            req.value = 5

            # forward to the robot
            src.drivers.nachi_robot.set_speed(req)

            src.drivers.nachi_robot.udp["command"].stopConsumeThread()
            src.drivers.nachi_robot.udp["update"].stopConsumeThread()

            # the command is appended to the queue
            assert setSpeed.called
            assert setSpeed.call_count == 1

    def test_abort(self):
        """Test abort command
        
        Requirements: the command is forwarded to the robot
        """

        with patch.object(socket.socket, 'connect', return_value=True) as mock_connect, \
            patch.object(socket.socket, 'bind', return_value=True) as mock_bind, \
            patch.object(src.drivers.nachi_robot, 'sendAbort') as abort, \
            patch.object(UdpConnector, 'appendToQueue', mock_appendToQueue_abort):

            src.drivers.nachi_robot.udp["command"] = UdpConnector("localhost", 8000)
            src.drivers.nachi_robot.udp["update"] = UdpConnector("localhost", 8000)

            # create command
            req = EmptyRequest()

            # send to the driver
            src.drivers.nachi_robot.abort(req)

            src.drivers.nachi_robot.udp["command"].stopConsumeThread()
            src.drivers.nachi_robot.udp["update"].stopConsumeThread()

            # the command is in the queue
            assert abort.called
            assert abort.call_count == 1

    def test_init(self):
        """Test the driver initalization

        Requirements: 
        - all of the services are advertised
        - all of the topics are published
        - the UDP connection is initalized
        """

        with patch.object(rospy, "wait_for_service", return_value=True), \
            patch.object(rospy, "get_param", mock_get_param), \
            patch.object(rospy, "init_node", return_value=None), \
            patch.object(rospy, 'spin', return_value=None), \
            patch.object(rospy.Service, '__init__', return_value=None) as mock_service_init, \
            patch.object(rospy.Publisher, '__init__', return_value=None) as mock_publisher_init, \
            patch.object(Thread, 'start', return_value=None) as mock_start_thread, \
            patch.object(Thread, 'join', return_value=None), \
            patch.object(socket.socket, 'connect', return_value=True) as mock_connect, \
            patch.object(socket.socket, 'bind', return_value=True) as mock_bind:

            src.drivers.nachi_robot.init()

            for sn in ["move_along", "abort", "store_poses", "move_pose", "set_speed", "move_between"]:
                # both required services are advertised
                assert len([call for call in mock_service_init.mock_calls if call[1][0] == sn]) == 1

            # topic is advertised
            assert mock_publisher_init.call_count == 2
            assert mock_publisher_init.mock_calls[0][1][0] == "robot_state"
            assert mock_publisher_init.mock_calls[0][1][1] == RobotState

            assert mock_publisher_init.mock_calls[1][1][0] == "robot_controller_joint_state"
            assert mock_publisher_init.mock_calls[1][1][1] == JointState

    def test_init_fail(self):
         with patch.object(rospy, "wait_for_service", return_value=True), \
            patch.object(rospy, "get_param", return_value=None), \
            patch.object(rospy, "init_node", return_value=None), \
            patch.object(rospy, 'spin', return_value=None), \
            patch.object(sys, 'exit', return_value=None) as mock_exit, \
            patch.object(rospy.Service, '__init__', return_value=None) as mock_service_init, \
            patch.object(rospy.Publisher, '__init__', return_value=None) as mock_publisher_init, \
            patch.object(Thread, 'start', return_value=None) as mock_start_thread, \
            patch.object(Thread, 'join', return_value=None), \
            patch.object(socket.socket, 'connect', return_value=True) as mock_connect, \
            patch.object(socket.socket, 'bind', return_value=True) as mock_bind:

            try:
                #this will fail, because no ip is set
                src.drivers.nachi_robot.init()
            except:
                assert True

            

    def test_send_play(self):
        """Test send play command

        Requirements:
        - Command the robot to move
        """

        with patch.object(socket.socket, 'connect', return_value=True) as mock_connect, \
            patch.object(socket.socket, 'bind', return_value=True) as mock_bind:
            src.drivers.nachi_robot.udp["command"] = UdpConnector("localhost", 8000)

        with patch.object(UdpConnector, 'appendToQueue') as mock_appendToQueue:
            src.drivers.nachi_robot.allPositions = self.get_moves(50).moves
            src.drivers.nachi_robot.move_between(MoveBetweenRequest( start = 2, end = 3 ))

            assert mock_appendToQueue.called
            
            # append the command to the UDP queue
            msg = mock_appendToQueue.mock_calls[0][1][0]

            # check the udp package for the right params
            assert src.drivers.nachi_robot.Commands.playback == unpack('>i',msg[0:4])[0]
            assert 3 == unpack('>i',msg[4:8])[0]
            assert 4 == unpack('>i',msg[8:12])[0]
            assert 1 == unpack('>i',msg[12:16])[0]

        src.drivers.nachi_robot.udp["command"].stopConsumeThread()

    def test_robot_update(self):
        """Test the robot state update handler

        Requirements:
        - The robot state is called
        - The topic is updating after each robot state update, if the state is different
        """

        rospy.Rate = MockRate
        src.drivers.nachi_robot.p_robot_state = rospy.Publisher("test1", RobotState)
        src.drivers.nachi_robot.p_joint_states = rospy.Publisher("test2", JointState)

        with patch.object(rospy.Publisher, 'publish', return_value=None) as mock_publish, \
            patch.object(UdpConnector, 'appendToQueue', mock_appendToQueue_sendupdate), \
            patch.object(socket.socket, 'connect', return_value=True) as mock_connect, \
            patch.object(socket.socket, 'bind', return_value=True) as mock_connect:

            src.drivers.nachi_robot.udp['update'] = UdpConnector("localhost", 8000)

            update_thread = Thread(target = src.drivers.nachi_robot.robot_state_publisher)
            update_thread.do_run = True
            update_thread.start()

            #run the thread for 4s, then stop, feed in every 0.1s
            i = 0.
            updates = 0
            while i < 4:
                # build up a test message
                msg = bytearray()

                msg.extend(pack('<i', 0)[::-1]) #seq
                msg.extend(pack('<i', 0)[::-1]) #cmd
                msg.extend(pack('<f', 0)[::-1]) #speed
                msg.extend(pack('<f', i)[::-1]) #x
                msg.extend(pack('<f', i)[::-1]) #y
                msg.extend(pack('<f', 0)[::-1]) #z
                msg.extend(pack('<f', 0)[::-1]) #rx
                msg.extend(pack('<f', 0)[::-1]) #ry
                msg.extend(pack('<f', 0)[::-1]) #rz
                msg.extend(pack('<i', 0)[::-1]) #step
                msg.extend(pack('<i', 0)[::-1]) #storedPoses 
                msg.extend(pack('<i', 0)[::-1]) #robotprogstate
                msg.extend(pack('<i', 0)[::-1]) #mode
                msg.extend(pack('<f', 1 * i)[::-1]) #j1
                msg.extend(pack('<f', 2)[::-1]) #j2
                msg.extend(pack('<f', 3)[::-1]) #j3
                msg.extend(pack('<f', 4)[::-1]) #j4
                msg.extend(pack('<f', 5)[::-1]) #j5
                msg.extend(pack('<f', 6)[::-1]) #j6
                msg.extend(pack('<f', 7)[::-1]) #j7

                # call handler
                src.drivers.nachi_robot.handleUpdateResponse(msg)

                # go to sleep
                time.sleep(0.0005)

                i = i + 0.05

                updates = updates + 1

            # stop the thread
            update_thread.do_run = False
            update_thread.join()

            # check if there is no new connection lost error
            assert mock_publish.call_count == updates * 2 # joint and robot state publish
            src.drivers.nachi_robot.udp["update"].stopConsumeThread()

    def test_broken_robot_update(self):
        """Test a broken robot update

        Requirements:
        - The driver keeps running
        """

        rospy.Rate = MockRate
        src.drivers.nachi_robot.p_robot_state = rospy.Publisher("test1", RobotState)
        src.drivers.nachi_robot.p_joint_states = rospy.Publisher("test2", JointState)

        with patch.object(rospy.Publisher, 'publish', return_value=None) as mock_publish, \
            patch.object(UdpConnector, 'appendToQueue', mock_appendToQueue_sendupdate), \
            patch.object(socket.socket, 'connect', return_value=True) as mock_connect, \
            patch.object(socket.socket, 'bind', return_value=True) as mock_connect, \
            patch.object(src.drivers.nachi_robot, "status", return_value=None) as mock_status:

            src.drivers.nachi_robot.udp['update'] = UdpConnector("localhost", 8000)

            update_thread = Thread(target = src.drivers.nachi_robot.robot_state_publisher)
            update_thread.do_run = True
            update_thread.start()

            #run the thread for 4s, then stop, feed in every 0.1s
            i = 0.
            updates = 0
            while i < 4:
                # build up a test message
                msg = bytearray()

                msg.extend(pack('<i', 0)[::-1]) #seq
                msg.extend(pack('<i', 0)[::-1]) #cmd
                msg.extend(pack('<f', 0)[::-1]) #speed
                msg.extend(pack('<f', i)[::-1]) #x

                # call handler
                src.drivers.nachi_robot.handleUpdateResponse(msg)

                # go to sleep
                time.sleep(0.0005)

                i = i + 0.05

                updates = updates + 1

            # stop the thread
            update_thread.do_run = False
            update_thread.join()

            # no update, because it failed, but reaches this point
            assert mock_publish.call_count == 0 
            for i in range(updates):
                call = mock_status.mock_calls[i]
                assert call[1][2] == src.drivers.misc.status.STATE.ERROR

            src.drivers.nachi_robot.udp["update"].stopConsumeThread()