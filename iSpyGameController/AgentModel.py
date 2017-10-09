"""
This Module handles aspects of the agent architecture's decision-making and gameplay.
"""
# pylint: disable=import-error
import random
from GameUtils.GlobalSettings import DO_EPSILON_INCREASING_POLICY # pylint: disable=import-error



class ActionSpace(): # pylint: disable=too-few-public-methods
    """
    This class defines constants signifying the potential actions an agent can take
    """
    RING_ANSWER_CORRECT = "RING_ANSWER_CORRECT"
    LATE_RING = "LATE_RING"
    #RING_ANSWER_WRONG = "RING_ANSWER_WRONG"
    DONT_RING = "DONT_RING"

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
        self.ring_rate = .5
        self.ring_increase_factor = .03 # amount each round that the starting_ring_rate increases

    def random_selection(self):
    	return random.choice([ActionSpace.RING_ANSWER_CORRECT, ActionSpace.LATE_RING, ActionSpace.DONT_RING])

    def get_next_action(self):
        """
        Returns one of the actions from the ActionSpace
        """

        next_action = self.random_selection()
        return next_action

    def get_action_history(self):
        """
        Returns one of the actions from the ActionSpace
        """
        return self.action_history
