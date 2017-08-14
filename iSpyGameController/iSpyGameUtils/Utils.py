"""
This is a basic class for the Game Controller
"""
# -*- coding: utf-8 -*-
# pylint: disable=import-error

import json
import PronunciationResultsHandler

def ispy_game_sender():
    """
    This sender uses ROS to send messages via rosbridge_server
    """
    pass
    # import rospy
    # from unity_game_msgs.msg import iSpyCommand
    # from std_msgs.msg import Header  # standard ROS msg header
    #
    # # build a message based on the command:
    # # open ros up here, then run through the below and send all
    #
    # # start ROS node
    # pub = rospy.Publisher(iSpyGameController.iSpyGameFSM.ROS_TO_ISPY_GAME_TOPIC,
    #                       iSpyCommand, queue_size=10)
    # rospy.init_node('ros_to_game_sender', anonymous=True)
    # rate = rospy.Rate(10)  # spin at 10 Hz
    # rate.sleep()  # sleep to wait for subscribers
    #
    # # start building message
    # msg = iSpyCommand()
    # # add header
    # msg.header = Header()
    # msg.header.stamp = rospy.Time.now()
    #
    # # fill in command and properties:
    # msg.command = 1
    #
    #
    # # send Opal message to tablet game
    # pub.publish(msg)
    # rospy.loginfo(msg)
    # rate.sleep()

def organize_speechace_result(results):
    """ Converts the speechace results from a messy dictionary to list of phonemes
    and their bool values
    :param results: a dictionary of all the 
    :return: list of tuples with the phoneme and a bool of if it passes the threshold
    """
    
    results_handler = PronunciationResultsHandler.PronunciationResultsHandler()

    for word in range(len(results)):
        # print (results[word])
        letters, passed = results_handler.process_Speechace_results(results[word])

    #Uncomment out to test without phoneme to letter set up
    # speech_result = {
    # 'b-0': True, 'a-1': True, 'n-2': True, 'a-3': True, 'n-4': True, 'a-5': True, 's-6': True
    # }

    return letters, passed








