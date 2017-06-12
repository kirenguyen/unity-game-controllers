"""
This is a basic class for the Game Controller
"""
# -*- coding: utf-8 -*-
from transitions import Machine
from .TapGameUtils import Curriculum

class TapGameFSM:
    """
    Each class should have a docstring describing what it does
    """

    ROS_TO_TAP_GAME_TOPIC = '/tap_game_from_ros'
    TAP_GAME_TO_ROS_TOPIC = '/tap_game_to_ros'

    def __init__(self):

        self.round_index = 1
        self.max_rounds = 2

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

        def on_log_received(data):
            """
            Rospy Callback for when we get log messages
            """
            #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
            print("I heard %s", data.data)



        def startLogListener():
            """
            Node starting function
            """
            print('Node started')
            # rospy.init_node('LogListener', anonymous=True)
            # rospy.Subscriber(TAP_GAME_TO_ROS_TOPIC, String, on_log_received)
            # rospy.spin()

    def evaluate_round(self):
        """
        used to evaluate the end of the round
        """
        if self.round_index == self.max_rounds:
            self.allRoundsFinished()
        else:
            self.round_index += 1
            self.initNextRound()