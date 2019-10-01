import sys
import copy
import rospy
import moveit_msgs.msg
import tf
import geometry_msgs.msg
from geometry_msgs.msg import Pose
from math import pi
from std_msgs.msg import String
import rosweld_tools.srv
import rosweld_drivers.srv
from rospy.exceptions import ROSException

def init():
    rospy.init_node('hyundai_robot_test', anonymous=True)

    try: 
        rospy.wait_for_service('hyundai_robot/move_along', 1)
    except ROSException:
        print "Please run the hyundai_robot component"
        rospy.wait_for_service('hyundai_robot/move_along')

    move_along = rospy.ServiceProxy('hyundai_robot/move_along', rosweld_drivers.srv.MoveAlong)
    set_speed = rospy.ServiceProxy('hyundai_robot/set_speed', rosweld_drivers.srv.SetSpeed)
    moves = []

    for i in range(21):
        move = rosweld_drivers.msg.Move()
        move.pose.position.x = 0.5 
        move.pose.position.y = -0.3 + i * 0.01
        move.pose.position.z = 0.6 + i * 0.01

        q = tf.transformations.quaternion_from_euler(pi/2, i*pi/90, -pi/2)

        move.pose.orientation.x = q[0]
        move.pose.orientation.y = q[1]
        move.pose.orientation.z = q[2]
        move.pose.orientation.w = q[3]
        
        moves.append(copy.deepcopy(move))  

    # move = rosweld_drivers.msg.Move()
    # move.pose.position.x = 0.125 
    # move.pose.position.y = -0.25
    # move.pose.position.z = 0.14

    # q = tf.transformations.quaternion_from_euler(0, 0, -pi)

    # move.pose.orientation.x = q[0]
    # move.pose.orientation.y = q[1]
    # move.pose.orientation.z = q[2]
    # move.pose.orientation.w = q[3]
    #move.speed = 1
    # moves.append(copy.deepcopy(move))

    #TODO: uncomment
    #set_speed(40)
    move_along(moves)


if __name__ == "__main__":
    init()



