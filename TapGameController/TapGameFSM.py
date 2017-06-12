"""
This is a basic class for the Game Controller
"""
# -*- coding: utf-8 -*-
from transitions import Machine
from . import helpers
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

        def on_enter_GAME_START(self):
            """
            Called when we enter state GAME_START
            """
            print('I am starting the Game! Rd ' + self.round_index + 'coming up')

        def on_enter_ROUND_START(self):
            """
            Called when we enter state ROUND_START
            """
            print('I am starting Round ' + self.round_index + '!')

        def on_enter_ROUND_ACTIVE(self):
            """
            Called when we enter state ROUND_ACTIVE
            """
            print('DING DING, Round ' + self.round_index + ' is now LIVE!')

        def on_enter_ROUND_END(self):
            """
            Called when we enter state ROUND_END
            """
            print('Round ' + self.round_index + ' is over!')



        def on_log_received(data):
            """
            Rospy Callback for when we get log messages
            """
            #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
            print("I heard %s", data.data)


        def startNode():
            """
            Node starting function
            """
            print('Node started')
            # rospy.init_node('listener', anonymous=True)
            # rospy.Subscriber("chatter", String, callback)
            # spin() simply keeps python from exiting until this node is stopped
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



    @staticmethod
    def get_hmm():
        """Get a thought."""
        print(Curriculum.DOG)
        return 'hmmm...'


    def hmm(self):
        """Contemplation..."""
        if helpers.get_answer():
            print(self.get_hmm())
