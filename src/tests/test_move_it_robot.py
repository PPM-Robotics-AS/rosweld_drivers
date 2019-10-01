from mock import patch
import rospy
import random
import moveit_commander
import src.drivers.move_it_robot
import time
from threading import Thread, currentThread

from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from std_srvs.srv import EmptyRequest
from geometry_msgs.msg import Pose, PoseStamped
from rosweld_drivers.msg import Move, RobotState
from rosweld_drivers.srv import MoveAlongRequest, MoveBetweenRequest,SetSpeedRequest
from mocks import *

sendPlayCallParams = None

def mock_send_play(start, end, d, poses = None):
    """Mock the send play command
    
    Keyword Arguments:
        start {int} -- start index of the step (default: {1})
        end {int} -- end index of the step (default: {-1})
        d {int} -- direction: 1 - forward, -1 - backward (default: {1})
        poses {Move[]} -- Moves to follow (default: {None})
    """

    global sendPlayCallParams
    sendPlayCallParams = { 'start': start, 'end': end, 'direction': d, 'poses': poses }

class TestMoveItRobot(object):
    """Unit tests for the MoveIt! Robot driver
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

        req = self.get_moves(5)

        src.drivers.move_it_robot.store_poses(req)
        assert len(src.drivers.move_it_robot.getPositions()) == 5

        src.drivers.move_it_robot.store_poses(req)
        assert len(src.drivers.move_it_robot.getPositions()) != 10

    def test_move_between(self):
        """Test move between function

        Requirements:
        - Work with stored poses
        - The robot side driver's indexing is +1
        - The direction is based on the start and end index, 1 - forward, -1 backward
        """

        global sendPlayCallParams

        req = self.get_moves(5)
        src.drivers.move_it_robot.store_poses(req)

        with patch('src.drivers.move_it_robot.sendPlay', side_effect = mock_send_play) as m:
            src.drivers.move_it_robot.move_between(MoveBetweenRequest( start = 2, end = 3 ))
            
            assert sendPlayCallParams['start'] == 2
            assert sendPlayCallParams['end'] == 3
            assert sendPlayCallParams['direction'] == 1
            assert sendPlayCallParams['poses'] == None
            assert m.called

            src.drivers.move_it_robot.move_between(MoveBetweenRequest( start = 3, end = 1 ))
            
            assert sendPlayCallParams['start'] == 3
            assert sendPlayCallParams['end'] == 1
            assert sendPlayCallParams['direction'] == -1
            assert sendPlayCallParams['poses'] == None
            assert m.called
            assert m.call_count == 2
    
    def test_move_along(self):
        """Test to move along a given path
        
        Requirements:
        - Uploads the moves to the robot
        - Go through the path from 0->len(allPositions)
        """

        global sendPlayCallParams
        req = self.get_moves(5)

        with patch('src.drivers.move_it_robot.sendPlay', side_effect = mock_send_play) as m:
            src.drivers.move_it_robot.move_along(req)

            assert m.called
            assert m.call_count == 1
            assert sendPlayCallParams['start'] == 0
            assert sendPlayCallParams['end'] == len(req.moves) - 1
            assert sendPlayCallParams['direction'] == 1
            assert sendPlayCallParams['poses'] == None

            src.drivers.move_it_robot.move_along(self.get_moves(0))
            # send play is not called 
            assert m.call_count == 1

    def test_move_pose(self):
        """Go to a specific move/pose

        Requirements:
        - go to the robot with 1 move
        """

        global sendPlayCallParams
        req = self.get_moves(1)

        with patch('src.drivers.move_it_robot.sendPlay', side_effect = mock_send_play) as m:
            src.drivers.move_it_robot.move_pose(req)

            assert m.called
            assert m.call_count == 1
            assert sendPlayCallParams['start'] == 0
            assert sendPlayCallParams['end'] == 0
            assert sendPlayCallParams['direction'] == 1
            assert len(sendPlayCallParams['poses']) == len(req.moves)

            src.drivers.move_it_robot.move_pose(self.get_moves(0))
            # send play is not called 
            assert m.call_count == 1

    def test_set_speed(self):
        """Set the speed on the robot

        Requirements:
        - the set speed command is forwarded to the robot
        """

        req = SetSpeedRequest(value = 10)

        src.drivers.move_it_robot.set_speed(req)

        assert src.drivers.move_it_robot.speed == req.value

    def test_abort(self):
        """Test abort command
        
        Requirements: the command is forwarded to the robot
        """

        with patch.object(moveit_commander.MoveGroupCommander, '__init__', return_value=None) as initGroup, \
            patch.object(moveit_commander.MoveGroupCommander, 'stop', return_value=None) as stop:
            
            src.drivers.move_it_robot.group = moveit_commander.MoveGroupCommander("test")
            src.drivers.move_it_robot.abort(EmptyRequest())
            assert stop.called

    def test_send_play(self):
        # set speed
        src.drivers.move_it_robot.speed = 10.

        # mock group
        src.drivers.move_it_robot.group = MockGroup()

        # build up a dummy position list
        src.drivers.move_it_robot.allPositions = []

        for i in range(100):
            p = Pose()
            p.position.x = i
            src.drivers.move_it_robot.allPositions.append(p)

        with patch.object(MockGroup, "execute", return_value=None) as mock_execute:

            src.drivers.move_it_robot.sendPlay(0, 10, 1)
            assert mock_execute.called

            for i in range(10):
                src.drivers.move_it_robot.sendPlay(i*10, (i+1)*10 - 1, 1)
            
            assert mock_execute.call_count == len(range(10)) + 1

            # set the fraction to zero = no plan found
            MockGroup.ret_fraction = 0.
            src.drivers.move_it_robot.sendPlay(0, 10, 1)
            # the execute not called again, no plan found
            assert mock_execute.call_count == len(range(10)) + 1

    def test_state_publisher(self):
        """Test the robot state publisher

        Requirements:
        - the step calculation returns -1 if the position can not found or returns with an error
        """

        rospy.Rate = MockRate
        src.drivers.move_it_robot.p_pose = rospy.Publisher("test", RobotState)

        with patch.object(rospy.Publisher, 'publish', return_value=None) as mock_publish:
            # mock the group
            src.drivers.move_it_robot.group = MockGroup()

            # build up a dummy position list
            src.drivers.move_it_robot.allPositions = []

            for i in range(100):
                p = Pose()
                p.position.x = i
                src.drivers.move_it_robot.allPositions.append(p)

            src.drivers.move_it_robot.min_d = src.drivers.move_it_robot.get_min_distance()

            # start publisher thread
            thread_state_update = Thread(target = src.drivers.move_it_robot.robot_pose_publisher)
            thread_state_update.do_run = True
            thread_state_update.start()

            # run for 2 sec
            time.sleep(2.)

            thread_state_update.do_run = False
            thread_state_update.join()

            assert mock_publish.called
            for call in mock_publish.mock_calls:
                # test step calculation based on the position
                if call[1][0].pose.position.x < 100:
                    assert call[1][0].pose.position.x == call[1][0].step
                else:
                    # cant found pose, unknown step test
                    assert call[1][0].step == -1

    def test_init(self):
        """Test the driver initalization

        Requirements: 
        - all of the services are advertised
        - all of the topics are published
        - the UDP connection is initalized
        """

        with patch.object(rospy, "wait_for_service", return_value=True), \
            patch.object(rospy, "get_param", return_value="test"), \
            patch.object(rospy, "init_node", return_value=None), \
            patch.object(rospy, 'spin', return_value=None), \
            patch.object(rospy.Service, '__init__', return_value=None) as mock_service_init, \
            patch.object(rospy.Publisher, '__init__', return_value=None) as mock_publisher_init, \
            patch.object(moveit_commander, 'roscpp_initialize', return_value=None), \
            patch.object(moveit_commander.RobotCommander, '__init__', return_value=None), \
            patch.object(moveit_commander.PlanningSceneInterface, '__init__', return_value=None), \
            patch.object(moveit_commander.MoveGroupCommander, '__init__', return_value=None), \
            patch.object(Thread, 'start', return_value=None) as mock_start_thread, \
            patch.object(Thread, 'join', return_value=None):

            src.drivers.move_it_robot.init()

            for sn in ["move_along", "abort", "store_poses", "move_pose", "set_speed", "move_between"]:
                # both required services are advertised
                assert len([call for call in mock_service_init.mock_calls if call[1][0] == sn]) == 1

            # topic is advertised
            assert mock_publisher_init.called
            assert mock_publisher_init.mock_calls[0][1][0] == "robot_state"
            assert mock_publisher_init.mock_calls[0][1][1] == RobotState


             



