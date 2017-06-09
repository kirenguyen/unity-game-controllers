"""
This is a basic class for the Game Controller
"""
# -*- coding: utf-8 -*-
from . import helpers
from .TapGameUtils import Curriculum

class GameController:
    """
    Each class should have a docstring describing what it does
    """

    ROS_TO_TAP_GAME_TOPIC = '/tap_game_from_ros'
    TAP_GAME_TO_ROS_TOPIC = '/tap_game_to_ros'

    def __init__(self, member_var1, member_var2):
        self.member_var1 = member_var1
        self.member_var2 = member_var2

    @staticmethod
    def get_hmm():
        """Get a thought."""
        print(Curriculum.DOG)
        return 'hmmm...'


    def hmm(self):
        """Contemplation..."""
        if helpers.get_answer():
            print(self.get_hmm())
