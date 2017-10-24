"""
This Module handles aspects of the agent architecture's decision-making and gameplay.
"""
# pylint: disable=import-error
import random
from .RobotBehaviorList import RobotRoles
from .RobotBehaviorList import RobotRolesBehaviorsMap

# reinforcement learning modules
from .RLAgent import *
from .iSpyRLEnv import *




class AgentModel():
    """
    This class implements a simple reinforcement learning agent that chooses what to do each round
    """

    def __init__(self):

        self.role_space=[ RobotRoles.CUR_EXPERT]
        self.role_history=[]
        RobotRolesBehaviorsMap()

        # set up reinforcement learning here
        self.rl_env = iSpyEnvironment()

        all_states = self.rl_env.get_all_states()
        num_states = sum([len(i) for i in all_states[:,]])

        self.rl_agent = QLearningAgent(num_states=num_states,num_actions=len(POSSIBLE_ACTIONS))

    def get_next_robot_role(self):
        """
        Returns one of the actions from the ActionSpace
        """
        # get the next action using a rl model
        current_state = self.rl_env.observe_cur_state()
        next_action = self.rl_agent.get_action(current_state)
        
        return next_action
       
    def onRewardReceived(self):
        '''
        after robot performs the action, child's new current state and rewards are received
        '''
        # perform action by updating rl's current state, getting reward and updating its q function
        prev_state, reward, cur_state= self.rl_env.perform_action(action)
        # the agent learns and updates its q value
        agent.learn(prev_state,action,reward,cur_state)




