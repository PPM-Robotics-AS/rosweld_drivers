import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import tf
import geometry_msgs.msg
from rosweld_drivers.srv import MoveAlong
from rosweld_drivers.msg import Move
from geometry_msgs.msg import Pose
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import rosweld_tools.srv
from rospy.exceptions import ROSException

def init():
    rospy.init_node('move_it_robot_test', anonymous=True)

    try: 
        rospy.wait_for_service('move_along', 1)
    except ROSException:
        print "Please run the move_it_robot component"
        rospy.wait_for_service('move_along')

    try:
        rospy.wait_for_service('move_group/load_map', 1)
    except ROSException:
        print "Please start MoveIt using the script: 'roslaunch panda_moveit_config demo.launch'"
        rospy.wait_for_service('move_group/load_map')

    move_along = rospy.ServiceProxy('move_along', MoveAlong)
    moves = []

    move = Move()
    move.pose.position.x = 0.5
    move.pose.position.y = 0.0
    move.pose.position.z = 1.5
    move.speed = 10.0

    q = tf.transformations.quaternion_from_euler(0, 0, 0)

    move.pose.orientation.x = q[0]
    move.pose.orientation.y = q[1]
    move.pose.orientation.z = q[2]
    move.pose.orientation.w = q[3]
    moves.append(copy.deepcopy(move))

    move_along(moves)


if __name__ == "__main__":
    init()



