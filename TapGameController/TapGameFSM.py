"""
This is a basic class for the Game Controller
"""
# -*- coding: utf-8 -*-
# pylint: disable=import-error
from transitions import Machine
#from .TapGameUtils import Curriculum
from .TapGameUtils import GlobalSettings
from .StudentModel import StudentModel

import json

if GlobalSettings.USE_ROS:
    import rospy
    from std_msgs.msg import Header  # standard ROS msg header
    from unity_game_msgs.msg import TapGameCommand
    from unity_game_msgs.msg import TapGameLog
else:
    TapGameLog = GlobalSettings.TapGameLog #Mock object, used for testing in non-ROS environments
    TapGameCommand = GlobalSettings.TapGameCommand

ROS_TO_TAP_GAME_TOPIC = '/tap_game_from_ros'
TAP_GAME_TO_ROS_TOPIC = '/tap_game_to_ros'

FSM_LOG_MESSAGES = [TapGameLog.CHECK_IN, TapGameLog.GAME_START_PRESSED, TapGameLog.INIT_ROUND_DONE,
                    TapGameLog.START_ROUND_DONE, TapGameLog.ROBOT_RING_IN,
                    TapGameLog.PLAYER_RING_IN, TapGameLog.RESET_NEXT_ROUND_DONE]

class TapGameFSM: # pylint: disable=no-member
    """
    Each class should have a docstring describing what it does
    """

    def __init__(self):

        self.round_index = 1
        self.max_rounds = 2

        self.student_model = StudentModel()

        self.game_commander = None
        self.log_listener = None

        self.states = ['GAME_START', 'ROUND_START', 'ROUND_ACTIVE', 'ROUND_END', 'GAME_FINISHED']
        self.transitions = [
            {'trigger': 'initFirstRound', 'source': 'GAME_START', 'dest': 'ROUND_START'},
            {'trigger': 'startRound', 'source': 'ROUND_START', 'dest': 'ROUND_ACTIVE'},
            {'trigger': 'robotRingIn', 'source': 'ROUND_ACTIVE', 'dest': 'ROUND_END'},
            {'trigger': 'playerRingIn', 'source': 'ROUND_ACTIVE', 'dest': 'ROUND_END'},
            {'trigger': 'allRoundsFinished', 'source': 'ROUND_END', 'dest': 'GAME_FINISHED'},
            {'trigger': 'initNextRound', 'source': 'ROUND_END', 'dest': 'ROUND_START'}
        ]

        self.state_machine = Machine(self, states=self.states, transitions=self.transitions,
                                     initial='GAME_START')

    def on_log_received(self, data):
        """
        Rospy Callback for when we get log messages
        """
        rospy.loginfo(rospy.get_caller_id() + "I heard " + data.message)
        print("I heard " + data.message)

        if data.message in FSM_LOG_MESSAGES:
            print('its real!')

            if data.message == TapGameLog.CHECK_IN:
                print('Game Checked in!')

            if data.message == TapGameLog.GAME_START_PRESSED:                
                # get latest word                
                self.send_cmd(TapGameCommand.INIT_ROUND, self.student_model.get_next_best_word() )
                self.initFirstRound()

            if data.message == TapGameLog.INIT_ROUND_DONE:
                print('done initializing')
                self.startRound()                
                self.send_cmd(TapGameCommand.START_ROUND)

            if data.message == TapGameLog.START_ROUND_DONE:
                print('I heard Start Round DONE. Waiting for player input')

            if data.message == TapGameLog.PLAYER_RING_IN:
                print('Player Rang in!')
                self.playerRingIn()
                self.send_cmd(TapGameCommand.RESET_NEXT_ROUND)

            if data.message == TapGameLog.ROBOT_RING_IN:
                print('Robot Rang in!')
                self.robotRingIn()
                self.send_cmd(TapGameCommand.RESET_NEXT_ROUND)

            if data.message == TapGameLog.RESET_NEXT_ROUND_DONE:
                print('Done Resetting Round!')
                self.initNextRound()
                self.send_cmd(TapGameCommand.INIT_ROUND, self.student_model.get_next_best_word() )
        else:
            print('its not real!')


    def start_log_listener(self):
        """
        Start up the Game Log Subscriber node
        """
        print('Sub Node started')
        rospy.init_node('FSM_Listener_Controller', anonymous=True)
        self.log_listener = rospy.Subscriber(TAP_GAME_TO_ROS_TOPIC, TapGameLog,
                                             self.on_log_received)

    def start_cmd_publisher(self):
        """
        Starts up the command publisher node
        """
        print('Pub Node started')
        self.game_commander = rospy.Publisher(ROS_TO_TAP_GAME_TOPIC, TapGameCommand, queue_size=10)
        rate = rospy.Rate(10)  # spin at 10 Hz
        rate.sleep()  # sleep to wait for subscribers
        #rospy.spin()


    def send_cmd(self, command, *args):
        """
        send a TapGameCommand to game
        Args are optional parameters
        """
        msg = TapGameCommand()
        # add header
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()

        # fill in command and any params:
        msg.command = command
        if len(args) > 0:        
            msg.params = json.dumps(args[0]) #assume the

        # send message to tablet game
        if self.game_commander is None:
            self.start_cmd_publisher()
        self.game_commander.publish(msg)
        rospy.loginfo(msg)


    def evaluate_round(self):
        """
        used to evaluate the end of the round
        """
        if self.round_index == self.max_rounds:
            self.allRoundsFinished()
        else:
            self.round_index += 1
            self.initNextRound()
