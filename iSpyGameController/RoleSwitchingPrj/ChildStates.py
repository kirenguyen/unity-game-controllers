from GameUtils.GlobalSettings import iSpyRobotInteractionStates as ris
from ..RobotBehaviorList.RobotBehaviorList import RobotBehaviors
from .AgentModel import EnvBuilder
from .AgentModel import Estimator
from .AgentModel import QModel
from ..RobotBehaviorList.RobotBehaviorList import RobotRoles
import pickle
from random import *

RL_FOLDER = "iSpyGameController/saved_rl/"
class ChildStates:
	'''
	child's states for RL model.
	two parts: learning states and affective states
	'''	

	def __init__(self,child_id,subj_cond,task_ctl):
		## RL model ##
		self.rl_env = EnvBuilder(child_id)
		self.rl_estimator = Estimator(self.rl_env)
		self.rl_qmodel = QModel()
		self.rl_qmodel.initialize_q_learning(self.rl_env, self.rl_estimator,[0.0,0.0,0.0],reset=True)

		self.subj_cond = subj_cond

		self.num_available_target_objs = 0

		# for a given turn: number of pronunciation trials / number of available clickable objects in the game
		self.correct_rate_arr = []
		self.total_num_trials = 0 # total number of trials

		# keep track of whose turn it is
		self.current_turn = ""

		# how many questions robot has asked the child
		self.num_robot_questions_asked = 0 # updated by ChildRobotInteraction
		self.num_robot_questions_answered = 0  # updated by ChildRobotInteraction
		self.child_answer_content = ""

		# no tablet touch alert 
		self.numTouchAbsenceAlertPerTask = 0
		self.numChildAttemptsCurrTask = 0
		self.objectWordPronounced = False


		self.pos_answers = 0 
		self.neg_answers = 0
		self.other_answers = 0 
		self.no_answers_attempt1 = 0
		self.no_answers_attempt2 = 0

		### for RL model: state features
		self.s1_total_turns = 0
		self.s2_prev_expert_roles = 0 
		self.numChildCorrectAttemptsCurrTask = 0

		### for RL model: rewards features
		self.child_help = False
		self.child_turn_success = False
		self.child_success_rate = 0
		self.consecutive_incorrect_attempts = 0
		self.rl_reward_scores = 0

		self.rl_action = None

		self.child_states_history = []

		self.task_controller = task_ctl

	def get_next_robot_role(self):
		self.rl_action = self.rl_qmodel.q_get_action()
		return self.rl_action

	def evaluate_rl_action(self):
		def save_child_states():
			self.child_states_history.append([self.task_controller.vocab_word, self.s1_total_turns,self.s2_prev_expert_roles,self.numChildCorrectAttemptsCurrTask,
				self.child_help,self.child_turn_success, self.child_success_rate, self.consecutive_incorrect_attempts, self.rl_reward_scores])
		def reset_rewards():
			self.child_help = False
			self.child_turn_success = False
			self.rl_reward_scores = 0
		def get_rl_state_features():
			return [self.s1_total_turns,self.s2_prev_expert_roles, self.numChildCorrectAttemptsCurrTask]

		if self.subj_cond != 'adaptive':
			return

		self.s1_total_turns += 1
		if self.s1_total_turns == 1:
			self.rl_qmodel.q_new_episode(get_rl_state_features())
		elif self.s1_total_turns >1: # make sure it is not the first child's turn
			self.rl_qmodel.q_learning_evaluate(get_rl_state_features(),self.calculate_rl_rewards(),self.task_controller.vocab_word)
		save_child_states()
		reset_rewards()

		
	def save(self):
		if self.subj_cond != 'adaptive':
			return
		self.rl_qmodel.save_rl()
		pickle.dump(self.child_states_history, open(RL_FOLDER+self.rl_env.child_id+"_child_states"+str(randint(1,500))+".pkl","wb"))



	def calculate_rl_rewards(self):
		def novice_rewards():
			print("~~~~~~~~~~~~~~~~~~~ novice rewards~~~~~~")
			reward_scores = 0
			if self.child_help: reward_scores += 1
			if self.child_turn_success: reward_scores +=1*(4-self.numChildCorrectAttemptsCurrTask)
			if self.child_turn_success == False: reward_scores = 1*(4-self.numChildAttemptsCurrTask )
			return reward_scores
		def expert_rewards():
			print("~~~~~~~~~~~~~~~~~~~ expert rewards~~~~~~")
			reward_scores = 0
			if self.child_turn_success == True: reward_scores += 1 * self.old_inconsecutive
			if self.child_turn_success== True and self.child_success_rate > 0.5: reward_scores += 1  * (self.child_success_rate-0.5)*10
			return reward_scores
		def no_rewards():
			print("~~~~~~~~~~~~~~~~~~~ no rewards~~~~~~~~")
			return 0
		rewards_type = {RobotRoles.EXPERT.value:expert_rewards,RobotRoles.NOVICE.value:novice_rewards, 'novice':no_rewards,'expert':no_rewards}
		self.rl_reward_scores = rewards_type[self.rl_action]() 
		print("~~~~~~~~~~~~~~~~~~~~ rl rewards: {}\n".format(self.rl_reward_scores))
		return self.rl_reward_scores

	
	def on_new_task_received(self):
		'''
		reset some variables when a new task is received
		'''
		self.numTouchAbsenceAlertPerTask = 0
		self.numChildAttemptsCurrTask = 0
		self.numChildCorrectAttemptsCurrTask = 0
		self.total_num_trials = 0
		# QA related
		self.num_robot_questions_asked = 0
		self.pos_answers = 0 
		self.neg_answers = 0
		self.other_answers = 0 
		self.no_answers_attempt1 = 0
		self.no_answers_attempt2 = 0

		### for RL model: state features
		self.s1_total_turns = 0
		self.s2_prev_expert_roles = 0 

		### for RL model: rewards features
		self.child_help = False
		self.child_turn_success = False
		self.child_success_rate = 0.0
		self.consecutive_incorrect_attempts = 0
		self.rl_reward_scores = 0
		self.old_inconsecutive = 0 

		### reset rl model
		self.save()

	
	def done(self):
		'''the learning session is finished'''
		self.save()

	def set_num_available_objs(self,_num_available_target_objs):
		self.num_available_target_objs = _num_available_target_objs	
	

	def update_turn_result(self,whoseTurn,correct):
		'''
		update the current object retrieval result 
		'''
		self.old_inconsecutive = self.consecutive_incorrect_attempts
		self.total_num_trials += 1
		if whoseTurn == "childTURN" or "childHelp" in whoseTurn:
			self.numChildAttemptsCurrTask += 1
			if correct == True: # the child fails to find a correct target object 
				self.numChildCorrectAttemptsCurrTask +=1
				self.child_turn_success = True
				self.consecutive_incorrect_attempts = 0
			else:
				self.consecutive_incorrect_attempts += 1

		self.child_success_rate = self.numChildCorrectAttemptsCurrTask / self.numChildAttemptsCurrTask

	def update_qa_info(self,q_type,q_cmd):
		'''	
		update information about question & answer activity
		'''
		# update child's state: increment the number of questions asked
		self.num_robot_questions_asked += 1
		self.current_q_type = q_type
		# if "yes/no" == q_type:
		# 	self.numRobotYNQuestion +=1
		# elif "open-ended" == q_type:
		# 	self.numRobotOpenQuestion += 1

	def update_qa_result(self,category,attempt):
		if category == "positive":
			self.pos_answers +=1
		elif category == "negative":
			self.neg_answers += 1
		elif category == "others":
			self.other_answers +=1
		elif category == "absence" and attempt == 1:
			self.no_answers_attempt1 +=1
		elif category == "absence" and attempt == 2:
			self.no_answers_attempt2 +=1


	def start_tracking_rewards(self,turn):
		'''
		start tracking rewards for reinforcement learning model
		'''
		self.current_turn = turn
		# pitch? 
		# vision?
		# responsiveness?

	def stop_tracking_rewards(self,turn):
		'''
		stop tracking rewards and get the reward scores from this session
		'''
		if self.current_turn == turn:
			# stop tracking and calculate RL reward scores
			reward_score = 30
			return reward_score
		else:
			# something wrong...
			return 0


	