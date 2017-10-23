import numpy as np 

from agent import *
from ispy_child_env import *

env = iSpyEnvironment()
agent = QLearningAgent(num_states=len(env.get_all_states()),num_actions=len(POSSIBLE_ACTIONS))

print("len all states: "+str(env.get_all_states()))

for i in range(0,10,1):
	action = agent.get_action(env.observe_cur_state())

	# send the action as a ROS command to the Robot

	# perform action by updating rl's current state, getting reward and updating its q function
	prev_state, reward, cur_state= env.perform_action(action)
	print('previous state, reward, current state')
	print(prev_state,reward,cur_state)

	# the agent learns and updates its q value
	agent.learn(prev_state,action,reward,cur_state)


