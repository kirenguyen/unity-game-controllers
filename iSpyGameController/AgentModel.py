"""
This Module handles aspects of the agent architecture's decision-making and gameplay.
"""
# pylint: disable=import-error
import random


class ActionSpace(): # pylint: disable=too-few-public-methods
    """
    This class defines constants signifying the potential actions an agent can take
    """

    # CLicking under exploration / mission mode helps decides what panel to show up
    CLICK_EXPLORE = "CLICK_EXPLORE"
    CLICK_MISSION = "CLICK_MISSION"

    # Robot kicking in to help 
    RING_ANSWER = "RING_ANSWER"
    WAIT_FOR_RESPONSE = "WAIT_FOR_RESPONSE"

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
        self.help_rate = 0.2

    def robot_expert_mode(self):
        self.help_rate = 0.6

        if random.random() < self.help_rate:
            return get_next_action()
        else:
            return ActionSpace.RING_ANSWER

    def random_selection(self):
    	return random.choice([ActionSpace.RING_ANSWER, ActionSpace.WAIT_FOR_RESPONSE])

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
