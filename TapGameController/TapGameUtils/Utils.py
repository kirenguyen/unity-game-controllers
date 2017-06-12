"""
This is a basic class for the Game Controller
"""
# -*- coding: utf-8 -*-

import TapGameController.TapGameFSM

def tap_game_sender():
    """
    This sender uses ROS to send messages via rosbridge_server
    """
    import rospy
    from unity_game_msgs.msg import TapGameCommand
    from std_msgs.msg import Header  # standard ROS msg header

    # build a message based on the command:
    # open ros up here, then run through the below and send all

    # start ROS node
    pub = rospy.Publisher(TapGameController.TapGameFSM.ROS_TO_TAP_GAME_TOPIC, TapGameCommand, queue_size=10)
    rospy.init_node('ros_to_tap_sender', anonymous=True)
    rate = rospy.Rate(10)  # spin at 10 Hz
    rate.sleep()  # sleep to wait for subscribers

    # start building message
    msg = TapGameCommand()
    # add header
    msg.header = Header()
    msg.header.stamp = rospy.Time.now()

    # fill in command and properties:
    msg.command = 1


    # send Opal message to tablet game
    pub.publish(msg)
    rospy.loginfo(msg)
    rate.sleep()
