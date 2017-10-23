"""
This Module handles aspects of the agent architecture's decision-making and gameplay.
"""
# pylint: disable=import-error
import random
from .RobotBehaviorList import RobotRoles
from .RobotBehaviorList import RobotRolesBehaviorsMap






class AgentModel():
    """
    This class implements a simple rule-based agent that chooses what to do each round
    """

    def __init__(self):

        self.role_space=[ RobotRoles.CUR_EXPERT]
        self.role_history=[]
        RobotRolesBehaviorsMap()

    def get_behaviors(self,role):
        '''
        Get corresponding virtual and physical actions for a given input robot's role
        '''
        return (self.perform_physical_action(), self.perform_virtual_action())

    def get_next_robot_role(self):
        """
        Returns one of the actions from the ActionSpace
        """

        next_action = random.choice(self.role_space)
        return next_action




