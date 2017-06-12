"""
This is a basic class for the Game Controller
"""
# -*- coding: utf-8 -*-
from transitions import Machine
from .TapGameUtils import Curriculum
import rospy
from std_msgs.msg import Header  # standard ROS msg header
from std_msgs.msg import String 
from unity_game_msgs.msg import TapGameCommand
from unity_game_msgs.msg import TapGameLog

ROS_TO_TAP_GAME_TOPIC = '/tap_game_from_ros'
TAP_GAME_TO_ROS_TOPIC = '/tap_game_to_ros'

FSM_LOG_MESSAGES = [TapGameLog.CHECK_IN, TapGameLog.INIT_ROUND_DONE, TapGameLog.START_ROUND_DONE, TapGameLog.ROBOT_RING_IN, TapGameLog.PLAYER_RING_IN]

class TapGameFSM:
    """
    Each class should have a docstring describing what it does
    """



    def __init__(self):

        self.round_index = 1
        self.max_rounds = 2

        self.game_commander = None
        self.log_listener = None

        self.states = ['GAME_START', 'ROUND_START', 'ROUND_ACTIVE', 'ROUND_END', 'GAME_FINISHED']
        self.transitions = [
            {'trigger': 'initRound', 'source': 'GAME_START', 'dest': 'ROUND_START'},
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
                    print('initializing round')
                    self.initRound()
                    self.sendCmd(TapGameCommand.INIT_ROUND)
                if data.message == TapGameLog.INIT_ROUND_DONE:
                    print('done initializing')
                    self.startRound()
                    self.sendCmd(TapGameCommand.START_ROUND)
                if data.message == TapGameLog.START_ROUND_DONE:                    
                    print('I heard Start Round DONE. Waiting for player input')
                if data.message == TapGameLog.ROBOT_RING_IN:
                    print('Robot Rang in!')
                    self.robotRingIn()
            else:
                print('its not real!')


    def startLogListener(self):
            """
            Node starting function
            """
            print('Sub Node started')
            rospy.init_node('FSM_Listener_Controller', anonymous=True)
            self.log_listener = rospy.Subscriber(TAP_GAME_TO_ROS_TOPIC, TapGameLog, self.on_log_received)

    def startCmdPublisher(self):            
            print('Pub Node started')
            self.game_commander = rospy.Publisher(ROS_TO_TAP_GAME_TOPIC, TapGameCommand, queue_size=10)
            rate = rospy.Rate(10)  # spin at 10 Hz
            rate.sleep()  # sleep to wait for subscribers
            #rospy.spin()


    def sendCmd(self, command):
        # start building message
        msg = TapGameCommand()
        # add header
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()

        # fill in command and properties:
        msg.command = command

        # send message to tablet game
        if self.game_commander is None:
            self.startCmdPublisher()
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