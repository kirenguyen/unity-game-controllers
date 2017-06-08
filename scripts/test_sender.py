#!/usr/bin/env python
"""
This script can be used to send test messages to the Unity game via the command line
"""

import rospy
from unity_game_msgs.msg import TapGameCommand
from std_msgs.msg import Header  # standard ROS msg header
from GameController import ROS_TO_TAP_GAME_TOPIC



def tap_game_sender():
    """
    This sender uses ROS to send messages via rosbridge_server
    """

    # build a message based on the command:
    # open ros up here, then run through the below and send all

    # start ROS node
    pub = rospy.Publisher(ROS_TO_TAP_GAME_TOPIC, TapGameCommand, queue_size=10)
    rospy.init_node('ros_to_tap_sender', anonymous=True)
    rate = rospy.Rate(10)  # spin at 10 Hz
    rate.sleep()  # sleep to wait for subscribers

    # start building message
    msg = TapGameCommand()
    # add header
    msg.header = Header()
    msg.header.stamp = rospy.Time.now()

    # fill in command and properties:
    msg.command = 0


    # send Opal message to tablet game
    pub.publish(msg)
    rospy.loginfo(msg)
    rate.sleep()


if __name__ == '__main__':
    try:
        tap_game_sender()
    except rospy.ROSInterruptException:
        print('ROSnode shutdown')
