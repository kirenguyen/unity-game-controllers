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


class Estimator():
    """
    Value Function approximator. 
    """
    def __init__(self,env):
        self.models = []
        for _ in range(env.nA):
            model = SGDRegressor(learning_rate="constant",penalty='l1')
            env.reset()
            model.partial_fit([self.featurize_state(env.next_episode())], [1])
            self.models.append(model)
            
            self.get_weights()
            
    def featurize_state(self, state):
        return env.get_state_features(state)
    
    def predict(self, s, a=None):
        """
        Makes value function predictions.
        
        Args:
            s: state to make a prediction for
            a: (Optional) action to make a prediction for
            
        Returns
            If an action a is given this returns a single number as the prediction.
            If no action is given this returns a vector or predictions for all actions
            in the environment where pred[i] is the prediction for action i.
            
        """
        features = self.featurize_state(s)
        if not a:
            return np.array([m.predict([features])[0] for m in self.models])
        else:
            return self.models[a].predict([features])[0]
    
    def update(self, s, a, y):
        """
        Updates the estimator parameters for a given state and action towards
        the target y.
        """
        features = self.featurize_state(s)
        print("features: {}. y: {}".format(features,y))
        
        self.models[a].partial_fit([features], [y])
    def get_weights(self):
        for i in self.models:
            print("model weights: {}".format(i.coef_ ))


class AgentModel():
    """
    This class implements a simple reinforcement learning agent that chooses what to do each round
    """

    def __init__(self):

        self.role_space= list(RobotRoles)
        
        self.role_history=[]
        #RobotRolesBehaviorsMap()


        # set up reinforcement learning here
        self.rl_env = iSpyEnvironment()


        all_states = self.rl_env.get_all_states()
        num_states = sum([len(i) for i in all_states[:,]])

        self.rl_agent = QLearningAgent(num_states=num_states,num_actions=len(list(RobotRoles)))

        self.current_action = ""

        self.estimator = Estimator()

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


    def make_epsilon_greedy_policy(estimator, epsilon, nA):
        """
        Creates an epsilon-greedy policy based on a given Q-function approximator and epsilon.

        Returns:
        A function that takes the observation as an argument and returns
        the probabilities for each action in the form of a numpy array of length nA.
    
        """
        def policy_fn(observation):
            A = np.ones(nA, dtype=float) * epsilon / nA
            q_values = estimator.predict(observation)
            best_action = np.argmax(q_values)
            A[best_action] += (1.0 - epsilon)
            return A
        return policy_fn

    def q_learning_step(self):
        if next_action is None:
            action_probs = policy(state)
            self.rl_action = np.random.choice(np.arange(len(action_probs)), p=action_probs)
        else:
            self.rl_action = next_action

         # Take a step
        next_state, reward, action = env.auto_step()

         # TD Update
        q_values_next = estimator.predict(next_state)
        print("q values next: {}".format(q_values_next))
        estimator.get_weights()
            
        # Use this code for Q-Learning
        # Q-Value TD Target
        td_target = reward + discount_factor * np.max(q_values_next)
        print("cur state: {}, next state: {}, action: {}, reward: {}, max: {}, td target: {}".format(state, next_state, action, reward,np.max(q_values_next),td_target))
            
        if np.isnan(reward):
            continue
            
        # Update the function approximator using our target
        print("state: {}".format(state))
        estimator.update(state, action, td_target)
            
        print("\rStep {} @ Episode {}/{} ({})".format(t, i_episode + 1, num_episodes, last_reward), end="")
        print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
                
        state = next_state

    def setup_q_learning(env, estimator, num_episodes, discount_factor=0.3, epsilon=0.1, epsilon_decay=1.0):
    
            
        # The policy we're following
        self.rl_policy = make_epsilon_greedy_policy(
            estimator, epsilon * epsilon_decay**i_episode, env.nA)
        
        self.rl_action = None
        self.rl_next_action = None
        





