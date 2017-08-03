"""
This Module handles all aspects of the robot/agent's decision-making,
modeling, and gameplay.
"""
from random import randint


class ActionSpace(): # pylint: disable=too-few-public-methods
    """
    This class defines constants signifying the potential actions an agent can take
    """
    RING_ANSWER_CORRECT = "RING_ANSWER_CORRECT"
    RING_ANSWER_WRONG = "RING_ANSWER_WRONG"
    #DONT_RING = "DONT_RING"

    #REACT_FRUSTRATED = "REACT_FRUSTRATED"

    def __init__(self):
        pass




class AgentModel():
    """
    This class implements a simple rule-based agent that chooses what to do each round
    """

    def __init__(self):
        # fancy python one-liner to read all string attributes off of a class
        self.action_space = [p for p in dir(ActionSpace)
                             if isinstance(getattr(ActionSpace, p), str)
                             and not p.startswith('__')]

        self.action_history = []

    def get_next_action(self):
        """
        Returns one of the actions from the ActionSpace
        """
        # randint is inclusive
        next_action = (self.action_space[randint(0, len(self.action_space) - 1)])
        #next_action = "RING_ANSWER_WRONG"
        self.action_history.append(next_action)
        print('NEXT_ACTION is ' + next_action)
        return next_action

    def get_action_history(self):
        """
        Returns one of the actions from the ActionSpace
        """
        return self.action_history
