import pickle
import os
import numpy as np
import sys
import sklearn.pipeline
import sklearn.preprocessing
from sklearn.externals import joblib
from matplotlib import pyplot as pp

def read_action_history(p_id):
	def read_state_dict():
		s_dict= pickle.load(open("s_dict.p","rb"))
	def get_state_features(s):
		for k,v in s_dict.items():
			if v == s:
				return k.split('-')
				break
		return []
	def plot_rewards():
		rewards = []
		epis = []
		errors=[]
		i = 0
		for epi, instance in action_history.items():
			for k,v in instance.items():
				rewards.append(v[map_dict['reward']])
				errors.append(v[map_dict['td_target']] - v[map_dict['cur_value_max']])
				epis.append(i)
				i+=1
		print("rewards: {}".format(rewards))
		print("error: {}".format(errors))
	

	def get_state_features():

		# for epi, instance in action_history.items():
		# 	for k,v in instance.items():
		# 		s_dict[v[map_dict['start_state']]]
		pass

	#self.i_episode].update({self.i_time: [self.start_state, next_state, self.action, reward,np.max(q_values_next),td_target]})
	map_dict={'start_state':1,'next_state':2,'action':3,'reward':4,'cur_value_max':5,'next_value_max':6, 'td_target':7}
	action_history = pickle.load(open(p_id + "_action_history.pkl","rb"))
	read_state_dict()
	print("action hiostry")
	print(action_history)
	plot_rewards()

def read_i_episdoe(p_id):
	i_episdoe = pickle.load(open(p_id + "_i_episode.pkl","rb"))
	print("i episode")
	print(i_episdoe)

def read_regressor(p_id):
	reg1 = joblib.load(p_id + '_regressor_model_0.pkl')
	reg2 = joblib.load(p_id + '_regressor_model_1.pkl')
	reg1_coef = reg1.coef_ 
	reg2_coef = reg2.coef_
	reg1_inter = reg1.intercept_
	reg2_inter = reg2.intercept_
	print("reg1 coef:{}".format(reg1_coef))
	print("reg2 coef:{}".format(reg2_coef))

def read_pretrained_regressor():
	reg0 = joblib.load('pretrained_regressor_model_0.pkl')
	reg0_coef = reg0.coef_ 
	print("pretrained reg0 coef:{}".format(reg0_coef))
	reg1 = joblib.load('pretrained_regressor_model_1.pkl')
	reg1_coef = reg1.coef_ 
	print("pretrained reg1 coef:{}".format(reg1_coef))


def read_child_states_data():
	child_states = pickle.load(open(p_id + "_child_states183.pkl","rb"))
	print("child states data")
	print(child_states)

def read_pretrained_data():
	reg1 = joblib.load('pretrained_regressor_model_0.pkl')
	reg2 = joblib.load('pretrained_regressor_model_1.pkl')
	reg1_coef = reg1.coef_ 
	reg2_coef = reg2.coef_
	reg1_inter = reg1.intercept_
	reg2_inter = reg2.intercept_
	print("pretrained reg1 coef:{}".format(reg1_coef))
	print("pretrained reg2 coef:{}".format(reg2_coef))

	scaler = joblib.load('scaler.pkl')
	print("scaler mean: {}".format(scaler.mean_))




p_id = "p00"
read_action_history(p_id)
read_i_episdoe(p_id)
read_regressor(p_id)
read_pretrained_regressor()
read_child_states_data()

read_pretrained_data()


