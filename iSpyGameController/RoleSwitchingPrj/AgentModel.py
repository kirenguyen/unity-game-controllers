"""
This Module handles aspects of the agent architecture's decision-making and gameplay.
"""
# pylint: disable=import-error
import random
from ..RobotBehaviorList.RobotBehaviorList import RobotRoles
from ..RobotBehaviorList.RobotBehaviorList import RobotRolesBehaviorsMap
import numpy as np
import os
import pandas as pd
import math
import itertools
import sys
import sklearn.pipeline
import sklearn.preprocessing
from sklearn.externals import joblib
import pickle
import os.path


if "../" not in sys.path:
  sys.path.append("../") 

from sklearn.linear_model import SGDRegressor
from sklearn.kernel_approximation import RBFSampler
from sklearn.externals import joblib

RL_FOLDER = "iSpyGameController/saved_rl/"

class EnvBuilder():
    def __init__(self,child_id):
        self.A_map = {'EXPERT':0, 'NOVICE':1} #.name .value
        self.S_features = ['s1_total_turns','s2_prev_expert_roles','s3_child_correct_answers']
        self.reward_label = 'rl_reward_scores'

        self.nA = len(self.A_map.keys())
        self.nS = len(self.S_features)
        

        self.child_id = child_id
        
        self.S_dict = pickle.load( open( RL_FOLDER + "s_dict.p", "rb" ) )
        self.scaler = joblib.load(RL_FOLDER + 'scaler.pkl') 

        
    def get_rl_state(self,features):
        i = '-'.join([str(i) for i in list(features)])
        print("rl state: {}".format(i))
        if not i in self.S_dict.keys():
            print("cannot find {} in s dict".format(i))
            self.S_dict.update({i:len(self.S_dict)})
        return self.S_dict[i]
        
    def get_state_features(self,state):
        def quadratic(arr):
            arr = arr.tolist()
            qual =list(map(lambda x: math.pow(x,2),arr))
            ilist =arr + qual + [arr[0]*arr[1]*arr[2], qual[0]*qual[1]*qual[2]]
            for each in itertools.permutations(arr,2):
                ilist += [math.pow(each[0],2)*each[1], math.pow(each[1],2)*each[0]]
            for each in itertools.combinations(arr,2):
                ilist += [each[0]*each[1], math.pow(each[0],2)*math.pow(each[1],2)]
            return ilist
        def val(ar):
            return [ar[0], ar[1], ar[2], ar[0]*ar[1],ar[0]*ar[2],ar[1]*ar[2], ar[0]*ar[1]*ar[2],
                     math.pow(ar[0],2)*ar[1],math.pow(ar[1],2)*ar[0],math.pow(ar[0],2)*ar[2],math.pow(ar[1],2)*ar[2],
                     math.pow(ar[2],2)*ar[1],math.pow(ar[2],2)*ar[0]]
        def generate_features(ar):
            return val(ar)
        
        for i,v in self.S_dict.items():
            if state == v:
                return generate_features(self.scaler.transform([[float(m) for m in i.split('-')]])[0])
                break
        print("cannot find state features!!! {}".format(state))
        return "nan"
    
   

class Estimator():
    """
    Value Function approximator. 
    """
    def __init__(self,env,reset= False):
        def retrive_regressor_models():
            if os.path.isfile(self.model_names[0]) and reset == False:
                model_0 = joblib.load(self.model_names[0])
                model_1 = joblib.load(self.model_names[1]) 
                return [model_0, model_1]
            else:
                model_0 = joblib.load(RL_FOLDER + 'pretrained_regressor_model_0.pkl') 
                model_1 = joblib.load(RL_FOLDER + 'pretrained_regressor_model_1.pkl') 
                return [model_0, model_1]
        self.model_names = [RL_FOLDER+env.child_id+'_regressor_model_0.pkl', RL_FOLDER+env.child_id+'_regressor_model_1.pkl']
        self.models = retrive_regressor_models()  
        self.env = env       
            
    def save_updated_regressor_models(self):
        '''called at the end of each task mission'''
        for i in  range(len(self.model_names)):
            joblib.dump(self.models[i], self.model_names[i]) 
    
    def featurize_state(self, state):
        return self.env.get_state_features(state)
    
    def predict(self, s, a=None):
        """
        Makes value function predictions
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
        self.models[a].partial_fit([features], [y])

    def get_weights(self):
        for i in self.models:
            print("model weights: {}".format(i.coef_ ))


class QModel():
    def initialize_q_learning(self, env, estimator, initial_features, reset=False, discount_factor=0.5, epsilon=0.25, epsilon_decay=1.0):
        ''''''
        self.env = env
        self.estimator = estimator
        self.discount_factor = discount_factor
        self.epsilon = epsilon
        self.epsilon_decay = epsilon_decay
        try:
            self.i_episode =  pickle.load( open( RL_FOLDER+self.env.child_id+"_i_episode.pkl", "rb" ) ) if reset == False else 0
            self.action_history = pickle.load(open(RL_FOLDER+self.env.child_id+"_action_history.pkl","rb")) if reset == False else dict()
        except:
            self.i_episode = 0
            self.action_history = dict()  ## load
        self.i_time = 0
        self.start_state = self.env.get_rl_state(initial_features)


    def save_rl(self):
        '''save after each timestamp'''
        pickle.dump(self.action_history, open(RL_FOLDER+self.env.child_id+"_action_history.pkl","wb"))
        pickle.dump(self.i_episode, open(RL_FOLDER+self.env.child_id+"_i_episode.pkl","wb"))
        self.estimator.save_updated_regressor_models()
        
    def q_new_episode(self,initial_features):
        '''called when the child finishes the first turn for a given task. then call q_get_action'''
        self.i_episode += 1

        # Reset the environment and pick the first action
        self.start_state = self.env.get_rl_state(initial_features)
        self.action_history.update({self.i_episode:{}})

        
    def q_get_action(self): 
        def make_epsilon_greedy_policy( epsilon):
            def policy_fn(observation):
                print("observation: {}".format(observation))
                A = np.ones(self.env.nA, dtype=float) * epsilon / self.env.nA
                q_values = self.estimator.predict(observation)
                best_action = np.argmax(q_values)
                A[best_action] += (1.0 - epsilon)
                print("++++++++++++~~cur epsilon: {} : best action: {}".format(epsilon,best_action))
                return A

            return policy_fn
        
        
        # The policy we're following
        policy = make_epsilon_greedy_policy(
            self.epsilon * self.epsilon_decay**self.i_episode)
        next_action = None
        
        # Choose an action to take
        # If we're using SARSA we already decided in the previous step
        if next_action is None:
            action_probs = policy(self.start_state)
            self.action = np.random.choice(np.arange(len(action_probs)), p=action_probs)
            print("~action probs: {}:{}. action: {}".format(action_probs[0],action_probs[1],self.action))

        else:
            self.action = next_action
        
        return self.action
    
    def q_learning_evaluate(self,next_state,reward,vocab_word):
            
        print("\n=======current episode: {}=========".format(self.i_episode))
        print("+++++++++++++++++++++++reward++++++++++++: {}".format(reward))
        
        if isinstance(next_state, list):
            next_state = self.env.get_rl_state(next_state)

        self.i_time += 1



        # TD Update
        q_values_next = self.estimator.predict(next_state)
        cur_q_values = self.estimator.predict(self.start_state)
        print("q values next: {} | current q values: {}".format(q_values_next,cur_q_values))


        # Use this code for Q-Learning
        # Q-Value TD Target
        td_target = reward + self.discount_factor * np.max(q_values_next)
        print("timeframe: {} cur state: {}, next state: {}, action: {}, reward: {}, max: {}, td target: {}".format(self.i_time, self.start_state, next_state, self.action, reward,np.max(q_values_next),td_target))
        if not self.i_episode in self.action_history.keys():
            self.action_history.update({self.i_episode:{}})

      
        self.action_history[self.i_episode].update({self.i_time: [vocab_word,self.start_state, next_state, self.action, reward,np.max(cur_q_values), np.max(q_values_next),td_target]})
        
            
        # Use this code for SARSA TD Target for on policy-training:
            # next_action_probs = policy(next_state)
            # next_action = np.random.choice(np.arange(len(next_action_probs)), p=next_action_probs)             
            # td_target = reward + discount_factor * q_values_next[next_action]

        # Update the function approximator using our target
        self.estimator.update(self.start_state, self.action, td_target)

        print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")

        self.start_state = next_state
            



