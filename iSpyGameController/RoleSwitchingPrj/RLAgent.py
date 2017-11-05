import numpy as np 
import random

from ..RobotBehaviorList.RobotBehaviorList import RobotRoles


class Agent(object):
	'''
	base class
	'''
	def __init__(self, num_states, num_actions, discount_factor=0.9):
		self.num_states = num_states
		self.num_actions = num_actions
		self.discount_factor = discount_factor
		self.last_state = None
		self.last_action = None


class QLearningAgent(Agent):
    def __init__(self,  learning_rate =0.1, epsilon =0.5, value=0,**kwargs):
        super(QLearningAgent, self).__init__(**kwargs)
        self.learning_rate = learning_rate
        self.epsilon = epsilon
        self.value = value

        self.value_table = np.full((self.num_states, self.num_actions), self.value,dtype=float) # q table
    
    

    # RL: update q function with sample <s, a, r, s'>
    def learn(self, state, action, reward, next_state):
        print("..............agent.learn().....")
        print("value table")
        print(self.value_table)
        print("state: ")
        print(state)
        print("action..")
        print(action)
        current_q = self.value_table[state][action]
        print("state-action: "+str(state)+"|"+str(action))
        print("current q: "+str(current_q))
        # using Bellman Optimality Equation to update q function
        new_q = reward + self.discount_factor * max(self.value_table[next_state])
        print("discount factor part: "+str(self.discount_factor * max(self.value_table[next_state])))
        print("new q: "+str(new_q))
        self.value_table[state][action] += self.learning_rate * (new_q - current_q)

    

    # get action for the state according to the q function table
    # agent pick action of epsilon-greedy policy
    def get_action(self, state):
        print(".........agent.get_action()....")
        if np.random.rand() < self.epsilon:
            # take random action
            action_enum = np.random.choice(list(RobotRoles))
            action = action_enum.value

        else:
            # take action according to the q function table
            state_action = self.value_table[state]
            print("state action...")
            print(state_action)
            #action = self.arg_max(state_action)
            max_indices = np.where(state_action == state_action.max())[0]

            if len(max_indices) > 1:
                action = max_indices[random.randint(0, len(max_indices)-1) ]
            else:
                action = max_indices[0]

            print("chosen actions...!!!!.")
            print(max_indices)
        return action


    @staticmethod
    def arg_max(state_action):
        max_index_list = []
        max_value = state_action[0]
        for index, value in enumerate(state_action):
            if value > max_value:
                max_index_list.clear()
                max_value = value
                max_index_list.append(index)
            elif value == max_value:
                max_index_list.append(index)
        return random.choice(max_index_list)