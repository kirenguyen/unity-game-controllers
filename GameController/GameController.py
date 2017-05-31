"""
This is a basic class for the Game Controller
"""
# -*- coding: utf-8 -*-
from . import helpers


class GameController:
    """
    Each class should have a docstring describing what it does
    """

    def __init__(self, member_var1, member_var2):
        self.member_var1 = member_var1
        self.member_var2 = member_var2

    @staticmethod
    def get_hmm():
        """Get a thought."""
        return 'hmmm...'


    def hmm(self):
        """Contemplation..."""
        if helpers.get_answer():
            print(self.get_hmm())
