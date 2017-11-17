"""
This Module handles aspects of the agent architecture's decision-making and gameplay.
"""
# pylint: disable=import-error
import random
from ..RobotBehaviorList.RobotBehaviorList import RobotRoles
from ..RobotBehaviorList.RobotBehaviorList import RobotRolesBehaviorsMap

# reinforcement learning modules
from .RLAgent import *
from .RLiSpyEnvironment import *




class AgentModel():
    """
    This class implements a simple reinforcement learning agent that chooses what to do each round
    """

    def __init__(self):

        self.role_space= [RobotRoles.EXPERT] #list(RobotRoles)
        
        self.role_history=[]
        RobotRolesBehaviorsMap()


        # set up reinforcement learning here
        self.rl_env = iSpyEnvironment()


        all_states = self.rl_env.get_all_states()
        num_states = sum([len(i) for i in all_states[:,]])

        self.rl_agent = QLearningAgent(num_states=num_states,num_actions=len(list(RobotRoles)))

        self.current_action = ""

    def get_next_robot_role(self):
        """
        Returns one of the actions from the ActionSpace
        """
        # get the next action using a rl model
        current_state = self.rl_env.observe_cur_state()
        next_action = self.rl_agent.get_action(current_state)
        self.current_action = next_action # update RL's current action here

        return RobotRoles(next_action)
       
    def onRewardsReceived(self,rewards):
        '''
        after robot performs the action, child's rewards are received from ChildStates
        let the agent learn based on RL model's new current state and received rewards.
        '''
        # perform action by updating rl's current state, getting reward and updating its q function
        prev_state, cur_state= self.rl_env.perform_action(self.current_action)
        # the agent learns and updates its q value
        if self.current_action != "":
            self.rl_agent.learn(prev_state,self.current_action,rewards,cur_state) 

