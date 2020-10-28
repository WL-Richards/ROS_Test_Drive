#!/usr/bin/python

import rospy
from drive_distance.srv import *
from geometry_msgs.msg import Twist, Vector3


def get_current_position():
    """Get the current position of the robot from the odometry service"""

    # Wait for the service to be ready
    rospy.wait_for_service("get_odom")

    try:

        # Create a proxy service that we can call
        odom_service = rospy.ServiceProxy("get_odom", RobotPosition)

        # Call the service to get the robots position
        odom_service_response = odom_service()

        # Return the seperate X, Y and Z coordinates
        return odom_service_response.x, odom_service_response.y, odom_service_response.z

    # Throw an exception if the service breaks
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def drive_1m():
    """Drive the simulated robot one meter forward"""

    # Get the current position of the robot and 1m in front
    x = get_current_position()[0]
    wanted_x = x + 1

    # Set the current node to publish to the cmd_vel topic
    speed_publisher = rospy.Publisher("cmd_vel", Twist, queue_size=10)

    # Create the node
    rospy.init_node("distance_controller", anonymous=True)

    # Set the rate to update the topic 
    update_rate = rospy.Rate(10)

    # While ROS is running and the current X is less than the one we want to reach keep driving forward
    while not rospy.is_shutdown():
        pub_data = Twist()
        if x < wanted_x:
            pub_data.angular = Vector3(0,0,0)
            pub_data.linear = Vector3(0.2,0,0)
            speed_publisher.publish(pub_data)
            x = get_current_position()[0]
            update_rate.sleep()

        # If not stop the robot announce the maneuver is done then break out of the loop
        else:
            pub_data.linear = Vector3(0,0,0)
            speed_publisher.publish(pub_data)
            print("Maneuver Complete")
            break

if __name__ == "__main__":
    try:
        drive_1m()
    except rospy.ROSInterruptException:
        pass