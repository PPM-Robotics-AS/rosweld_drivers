from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Pose, PoseStamped
import random
import time

def mock_get_param(p, default = None):
    defaults = {
        "robot_update_port": 8001, 
        "robot_command_port": 8000,
        "robot_ip": "127.0.0.1"
    }

    if p in defaults:
        return defaults[p]

    if default != None:
        return default

    return p

class MockRate():
    """Mock class for rospy.rate
    """

    def __init__(self, hz):
        """Init function
        
        Arguments:
            hz {float} -- rate in hz
        """

        self.timeout = 1 / hz

    def sleep(self):
        """Put the current thread to sleep
        """

        time.sleep(self.timeout)

def check_command(commands, cmd):
    """Check the given command is in the Commands list
    
    Arguments:
        cmd {int} -- Value of the command
    """

    valid = False
    for property, value in vars(commands).iteritems():
        if value == cmd:
            valid = True
        
    assert valid

class MockGroup(): 
    """Mock MoveGroupCommander used functions
    """

    # set the fraction of the plan
    ret_fraction = 1.

    def __init__(self, *args, **kwargs):
        pass
    
    def get_current_pose(self):
        p = Pose()
        p.position.x = random.randint(1, 100)
        return PoseStamped(pose = p)

    def get_current_joint_values(self):
        return [1,2,3,4,5,6,7]

    def compute_cartesian_path(self, waypoints, eef_step, jump_threshold):
        trajectory = RobotTrajectory()
        for point in waypoints:
            trajectory.joint_trajectory.points.append(JointTrajectoryPoint())
        
        return (trajectory, MockGroup.ret_fraction)

    def execute(self, wait = True):
        pass

    def set_start_state_to_current_state(self):
        pass