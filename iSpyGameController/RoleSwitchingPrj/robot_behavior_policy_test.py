import numpy as np 

from .RLAgent import *
from .iSpyRLEnv import *

env = iSpyEnvironment()

all_states = env.get_all_states()
num_states = sum([len(i) for i in all_states[:,]])

agent = QLearningAgent(num_states=num_states,num_actions=len(POSSIBLE_ACTIONS))

print("len all states: "+str(env.get_all_states()))

for i in range(0,5,1):
	print("=================iteration: "+str(i)+"===============")
	action = agent.get_action(env.observe_cur_state())
	print("action from agent: "+str(action) + " | ")
	print(POSSIBLE_ACTIONS(action))
	print("................")
	# send the action as a ROS command to the Robot

	# perform action by updating rl's current state, getting reward and updating its q function
	prev_state, reward, cur_state= env.perform_action(action)
	print('.....previous state, reward, current state....')
	print(prev_state,reward,cur_state)

	# the agent learns and updates its q value
	agent.learn(prev_state,action,reward,cur_state)


