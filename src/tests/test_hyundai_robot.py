from mock import patch
import socket
import rospy
import math
from struct import unpack
import time

from threading import Thread, currentThread
from mocks import *
import src.drivers.hyundai_robot
from std_srvs.srv import EmptyRequest
from rosweld_drivers.msg import Move, RobotState
from rosweld_drivers.srv import MoveAlongRequest, MoveBetweenRequest, SetSpeedRequest
from ..drivers.misc.udp import UdpConnector
from sensor_msgs.msg import JointState

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
    check_command(src.drivers.hyundai_robot.Commands, command)

    assert label == "send_set_speed" 
    assert command == src.drivers.hyundai_robot.Commands.setSpeed 

def mock_appendToQueue_sendupdate(self, msg, label, handler = None):
    """Mock UDP's appendToQueue and check for requesting update
    
    Arguments:
        msg {bytearray} -- the message object as a bytearray
        label {string} -- label of the message
    
    Keyword Arguments:
        handler {function} -- callback function (default: {None})
    """
    
    command = unpack('>i',msg[0:4])[0]
    check_command(src.drivers.hyundai_robot.Commands, command)

    assert label == "send_update"
    assert command == src.drivers.hyundai_robot.Commands.update

class TestHyundaiRobot(object):
    """Unit tests for the Hyundai Robot driver
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

        req = self.get_moves(500)

        with patch.object(socket.socket, 'connect', return_value=True) as mock_connect, \
            patch.object(socket.socket, 'bind', return_value=True) as mock_bind:
            src.drivers.hyundai_robot.udp = UdpConnector("localhost", 8000)

        with patch.object(UdpConnector, 'appendToQueue') as u:

            src.drivers.hyundai_robot.store_poses(req)
            assert len(src.drivers.hyundai_robot.getPositions()) == len(req.moves)
            assert u.call_count == len(req.moves) / src.drivers.hyundai_robot.batchSize

            src.drivers.hyundai_robot.store_poses(req)
            assert len(src.drivers.hyundai_robot.getPositions()) != 2 * len(req.moves)

        src.drivers.hyundai_robot.udp.stopConsumeThread()

    def test_send_play(self):
        """Test send play command

        Requirements:
        - Command the robot to move
        """

        with patch.object(socket.socket, 'connect', return_value=True) as mock_connect, \
            patch.object(socket.socket, 'bind', return_value=True) as mock_bind:
            src.drivers.hyundai_robot.udp = UdpConnector("localhost", 8000)

        with patch.object(UdpConnector, 'appendToQueue') as mock_appendToQueue:
            src.drivers.hyundai_robot.allPositions = self.get_moves(50).moves
            src.drivers.hyundai_robot.move_between(MoveBetweenRequest( start = 2, end = 3 ))

            assert mock_appendToQueue.called
            
            req = mock_appendToQueue.mock_calls[0][1][0]
            enc = src.drivers.hyundai_robot.encode

            assert req[0:5] == bytearray(enc(src.drivers.hyundai_robot.Commands.playback))
            assert req[5:10] == bytearray(enc(3)) #start+1
            assert req[10:15] == bytearray(enc(4)) #end+1
            assert req[15:20] == bytearray(enc(1))

        src.drivers.hyundai_robot.udp.stopConsumeThread()
            
    def test_move_between(self):
        """Test move between function

        Requirements:
        - Work with stored poses
        - The robot side driver's indexing is +1
        - The direction is based on the start and end index, 1 - forward, -1 backward
        """

        global sendPlayCallParams

        req = self.get_moves(50)

        with patch.object(socket.socket, 'connect', return_value=True) as mock_connect, \
            patch.object(socket.socket, 'bind', return_value=True) as mock_bind:
            src.drivers.hyundai_robot.udp = UdpConnector("localhost", 8000)

        with patch('src.drivers.hyundai_robot.sendPlay', side_effect = mock_send_play) as m, \
             patch.object(UdpConnector, 'appendToQueue') as u:
            
            src.drivers.hyundai_robot.allPositions = []
            src.drivers.hyundai_robot.move_between(MoveBetweenRequest( start = 2, end = 3 ))
            assert u.called == False

            src.drivers.hyundai_robot.store_poses(req)
            assert u.call_count == math.ceil( len(req.moves) / src.drivers.hyundai_robot.batchSize )

            src.drivers.hyundai_robot.move_between(MoveBetweenRequest( start = 2, end = 3 ))
            
            assert sendPlayCallParams['start'] == 3
            assert sendPlayCallParams['end'] == 4
            assert sendPlayCallParams['direction'] == 1
            assert sendPlayCallParams['poses'] == None
            assert m.called

            src.drivers.hyundai_robot.move_between(MoveBetweenRequest( start = 3, end = 1 ))
            
            assert sendPlayCallParams['start'] == 4
            assert sendPlayCallParams['end'] == 2
            assert sendPlayCallParams['direction'] == -1
            assert sendPlayCallParams['poses'] == None
            assert m.call_count == 2

        src.drivers.hyundai_robot.udp.stopConsumeThread()

    def test_move_along(self):
        """Test to move along a given path
        
        Requirements:
        - Uploads the moves to the robot
        - Go through the path from 0->len(allPositions)
        """

        global sendPlayCallParams
        
        req = self.get_moves(5)

        with patch.object(socket.socket, 'connect', return_value=True) as mock_connect, \
            patch.object(socket.socket, 'bind', return_value=True) as mock_bind:
            src.drivers.hyundai_robot.udp = UdpConnector("localhost", 8000)
            
        
        with patch('src.drivers.hyundai_robot.sendPlay', side_effect = mock_send_play) as m, \
             patch.object(UdpConnector, 'appendToQueue') as u:
            src.drivers.hyundai_robot.move_along(req)

            assert m.called
            assert m.call_count == 1
            assert sendPlayCallParams['start'] == 1
            assert sendPlayCallParams['end'] == -1
            assert sendPlayCallParams['direction'] == 1
            assert sendPlayCallParams['poses'] == None

        src.drivers.hyundai_robot.udp.stopConsumeThread()
        

    def test_move_pose(self):
        """Go to a specific move/pose

        Requirements:
        - go to the robot with 1 move
        """

        global sendPlayCallParams
        
        req = self.get_moves(1)
        
        with patch.object(socket.socket, 'connect', return_value=True) as mock_connect, \
            patch.object(socket.socket, 'bind', return_value=True) as mock_bind:
            src.drivers.hyundai_robot.udp = UdpConnector("localhost", 8000)
            

        with patch.object(UdpConnector, 'appendToQueue') as u:
            
            src.drivers.hyundai_robot.move_pose(req)

            assert u.call_count == 1

        src.drivers.hyundai_robot.udp.stopConsumeThread()

    # def test_abort(self):
    #      with patch.object(socket.socket, 'connect', return_value=True) as mock_connect, \
    #         patch.object(socket.socket, 'bind', return_value=True) as mock_bind:

    #         src.drivers.hyundai_robot.udp = UdpConnector("localhost", 8000)

    #         with patch.object(UdpConnector, 'appendToQueue', return_value=None) as mock_appendToQueue:
    #             src.drivers.hyundai_robot.abort(EmptyRequest())

    #             assert mock_appendToQueue.called
    #             req = mock_appendToQueue.mock_calls[0][1][0]
    #             enc = src.drivers.hyundai_robot.encode

    #             assert req[0:5] == bytearray(enc(src.drivers.hyundai_robot.Commands.abort))

    #         src.drivers.hyundai_robot.udp.stopConsumeThread()

    def test_set_speed(self):
        """Set the speed on the robot

        Requirements:
        - the set speed command is forwarded to the robot
        """

        with patch.object(socket.socket, 'connect', return_value=True) as mock_connect, \
            patch.object(socket.socket, 'bind', return_value=True) as mock_bind, \
            patch.object(src.drivers.hyundai_robot, 'sendSetSpeed') as setSpeed, \
            patch.object(UdpConnector, 'appendToQueue', mock_appendToQueue_setspeed):

            src.drivers.hyundai_robot.udp = UdpConnector("localhost", 8000)
            
            req = SetSpeedRequest()
            req.value = 5
            src.drivers.hyundai_robot.set_speed(req)

            src.drivers.hyundai_robot.udp.stopConsumeThread()
            
            assert setSpeed.called
            assert setSpeed.call_count == 1

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

            src.drivers.hyundai_robot.init()

            for sn in ["move_along", "abort", "store_poses", "move_pose", "set_speed", "move_between"]:
                # both required services are advertised
                assert len([call for call in mock_service_init.mock_calls if call[1][0] == sn]) == 1

            # topic is advertised
            assert mock_publisher_init.call_count == 2
            assert mock_publisher_init.mock_calls[0][1][0] == "robot_state"
            assert mock_publisher_init.mock_calls[0][1][1] == RobotState

            assert mock_publisher_init.mock_calls[1][1][0] == "robot_controller_joint_state"
            assert mock_publisher_init.mock_calls[1][1][1] == JointState

    def test_robot_update(self):
        """Test the robot state update handler

        Requirements:
        - The robot state is called
        - The topic is updating after each robot state update, if the state is different
        """

        rospy.Rate = MockRate
        src.drivers.hyundai_robot.p_robot_state = rospy.Publisher("test1", RobotState, queue_size=1)
        src.drivers.hyundai_robot.p_joint_states = rospy.Publisher("test2", JointState, queue_size=1)

        with patch.object(rospy.Publisher, 'publish', return_value=None) as mock_publish, \
            patch.object(UdpConnector, 'appendToQueue', mock_appendToQueue_sendupdate), \
            patch.object(socket.socket, 'connect', return_value=True) as mock_connect, \
            patch.object(socket.socket, 'bind', return_value=True) as mock_connect:

            src.drivers.hyundai_robot.udp = UdpConnector("127.0.0.1", 8000, False, 9000)

            update_thread = Thread(target = src.drivers.hyundai_robot.robot_state_publisher)
            update_thread.do_run = True
            update_thread.start()

            #run the thread for 4s, then stop, feed in every 0.1s
            i = 0.
            updates = 0
            while i < 4:
                # build up a test message
                msg = []

                msg.append(0) #seq
                msg.append(0)#cmd
                msg.append(0)#speed
                msg.append(i)#x
                msg.append(i)#y
                msg.append(0)#z
                msg.append(0)#rx
                msg.append(0)#ry
                msg.append(0)#rz
                msg.append(0)#step
                msg.append(0)#storedPoses 
                msg.append(0)#robotprogstate
                msg.append(0)#mode
                msg.append(1 * i)#j1
                msg.append(2)#j2
                msg.append(3)#j3
                msg.append(4)#j4
                msg.append(5)#j5
                msg.append(6)#j6
                msg.append(7)#j7

                # call handler
                src.drivers.hyundai_robot.handleUpdateResponse("%s\0"%(' '.join([str(i) for i in msg])))

                # go to sleep
                time.sleep(0.0005)

                i = i + 0.05

                updates = updates + 1

            # stop the thread
            update_thread.do_run = False
            update_thread.join()

            # check if there is no new connection lost error
            assert mock_publish.call_count == updates * 2 # joint and robot state publish
            src.drivers.hyundai_robot.udp.stopConsumeThread()

    def test_bad_robot_update(self):
        """Test a broken robot update

        Requirements:
        - The driver keeps running
        """

        rospy.Rate = MockRate
        src.drivers.hyundai_robot.p_robot_state = rospy.Publisher("test1", RobotState)
        src.drivers.hyundai_robot.p_joint_states = rospy.Publisher("test2", JointState)

        with patch.object(rospy.Publisher, 'publish', return_value=None) as mock_publish, \
            patch.object(UdpConnector, 'appendToQueue', mock_appendToQueue_sendupdate), \
            patch.object(socket.socket, 'connect', return_value=True) as mock_connect, \
            patch.object(socket.socket, 'bind', return_value=True) as mock_connect, \
            patch.object(src.drivers.hyundai_robot, "status", return_value=None) as mock_status:

            src.drivers.hyundai_robot.udp = UdpConnector("127.0.0.1", 8000, False, 9000)

            update_thread = Thread(target = src.drivers.hyundai_robot.robot_state_publisher)
            update_thread.do_run = True
            update_thread.start()

            #run the thread for 4s, then stop, feed in every 0.1s
            i = 0.
            updates = 0
            while i < 4:
                # build up a test message
                msg = []

                msg.append(0) #seq
                msg.append(7) #j7

                # call handler
                src.drivers.hyundai_robot.handleUpdateResponse("%s\0"%(' '.join([str(i) for i in msg])))

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
            src.drivers.hyundai_robot.udp.stopConsumeThread()