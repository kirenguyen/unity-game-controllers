import numpy as np
from enum import Enum
import random


REWARDS=[]


class POSSIBLE_ACTIONS(Enum):
	CUR_EXPERT = 0
	CUR_NOVICE = 1
	NCUR_EXPERT = 2
	NCUR_NOVICE = 3



class iSpyEnvironment():
	'''
	class for modeling the child's states in the iSpy game for robot's behavior policy
	'''
	def __init__(self):
		self.possible_actions = POSSIBLE_ACTIONS
		self.rewards = REWARDS

		# current state the agent is in
		# initialize it to be -1 (the state before entering the game)
		self.cur_state = -1

		# 
		self.all_states=self._generate_rl_states()


	def observe_cur_state(self):
		'''
		observe the current state of the env
		'''
		return self.cur_state

	def perform_action(self,action):
		'''
		perform the action generated from agent.py's learning algorithm
		'''
		print('perform action')
		print("current state: "+str(self.cur_state))
		prev_state = self.cur_state
		
		# update RL's state after the action is performed 
		self.update_state_after_action()

		# calculate the reward for the performed action
		total_reward = self.get_reward(self.cur_state,action) 

		return prev_state, total_reward, self.cur_state



	def get_all_states(self):
		print("get all states")
		print(self.all_states)
		return self.all_states

	def update_state_after_action(self):
		'''
		measure and get the state after a robot's action
		'''
		self.cur_state = "x"
		learning_level = self._get_cur_learning_level()
		exploration_level = self._get_cur_exploration_level()
		# update current state after the action is performed 
		self.cur_state = self.all_states[learning_level,exploration_level]

	
	def get_reward(self,state,action):
		'''
		get rewards for the robot's action by observing the child's performance/behavior right after the robot's action
		'''
		learning_reward = self._get_child_learning_reward()
		affect_reward =  self._get_child_affect_reward()
		total_reward = learning_reward + affect_reward

		return total_reward

	def _get_cur_learning_level(self):
		'''
		get current learning level of the child
		'''
		return random.randint(0, 2)

	def _get_cur_exploration_level(self):
		'''
		get current learning level of the child
		'''
		# dummy number
		return random.randint(0,2)

	def _get_child_learning_reward(self):
		'''
		get rewards from child's learning change after robot's action
		'''

		# dummy number 
		return 30

	def _get_child_affect_reward(self):

		# dummy number
		return 30

	def _generate_rl_states(self):
		'''
		generate all possible states for the rl model
		return a matrix that represents the all possible states
		'''
		state_dict = { 'learning_levels':3, 'exploration_levels': 3 }
		total_num_states =1

		matrix_shape=()
		for key,value in state_dict.items():
			total_num_states *=value
			matrix_shape  = matrix_shape+ (value,)

		print("total # states: "+ str(total_num_states))
		states_matrix = np.arange(total_num_states).reshape(matrix_shape)
		print("states matrix")
		print(states_matrix)
		return states_matrix



#test=iSpyEnvironment()


