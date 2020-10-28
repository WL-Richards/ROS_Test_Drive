#!/usr/bin/python

from drive_distance.srv import RobotPosition,RobotPositionResponse
from nav_msgs.msg import Odometry
import rospy

CURRENT_POSITION = 0

def odom_callback(data):
    """When the odometry topic is updated set the new value to current position"""
    global CURRENT_POSITION
    CURRENT_POSITION = data.pose.pose.position

def on_service_contact(requested_data):
    """Whenever the service is requested return the robot's current position"""
    print(CURRENT_POSITION)
    return RobotPositionResponse(CURRENT_POSITION.x, CURRENT_POSITION.y, CURRENT_POSITION.z)

def start_service():
    """Start the odometry tracking service"""
    rospy.init_node("get_odom_service")

    # Service we call when we want to get odometry information
    odometry_service = rospy.Service(name="get_odom", service_class=RobotPosition, handler=on_service_contact)

    # Subscribe this node to the odom topic to get the odometry data
    odom_subscriber = rospy.Subscriber("odom", Odometry, callback=odom_callback)

    print("Awaiting instruction...")

    rospy.spin()

if __name__ == "__main__":
    start_service()