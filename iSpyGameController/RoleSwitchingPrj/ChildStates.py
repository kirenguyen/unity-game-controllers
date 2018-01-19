from GameUtils.GlobalSettings import iSpyRobotInteractionStates as ris
from ..RobotBehaviorList.RobotBehaviorList import RobotBehaviors


class ChildStates:
	'''
	child's states for RL model.
	two parts: learning states and affective states
	'''	

	def __init__(self):
		self.num_available_target_objs = 0

		# metric for learning: 
		# for a given turn: number of pronunciation trials / number of available clickable objects in the game
		self.correct_rate_arr = []

		# number of child's attempts of retrieving an object
		self.current_num_trials = 0

		# number of child's correct attempts (num of objs collected by the child)
		self.current_num_correct_trials = 0

		# keep track of whose turn it is
		self.current_turn = ""

		# how many questions robot has asked the child
		self.num_robot_questions_asked = 0 # updated by ChildRobotInteraction

		self.num_robot_questions_answered = 0  # updated by ChildRobotInteraction

		self.child_answer_content = ""

		self.child_turn_length = 0

		self.numRobotYNQuestion = 0

		# how many yes/no questions answered
		self.numRobotYNQuestionAnswered = 0

			# how many open ended questions
		self.numRobotOpenQuestion = 0

			# how many open ended questions answered
		self.numRobotOpenQuestionAnswered = 0

			# no tablet touch alert 
		self.numTouchAbsenceAlertPerTask = 0

		self.childCurrAttemptCorrectness = False

		self.childPrevAttemptCorrectness = False

		self.numChildAttemptsCurrTask = 0

		self.numChildCorrectAttemptsCurrTask = 0

		self.numRobotOfferHelp = 0

		self.numChildAcceptHelp = 0

		self.numRobotAskHelp = 0

		self.numChildOfferHelp = 0

		self.objectWordPronounced = False


	def on_new_task_received(self):
		'''
		reset some variables when a new task is received
		'''
		self.numTouchAbsenceAlertPerTask = 0
		self.numChildAttemptsCurrTask = 0
		self.numChildCorrectAttemptsCurrTask = 0



		
	def set_num_available_objs(self,_num_available_target_objs):
		self.num_available_target_objs = _num_available_target_objs	
	

	def update_child_turn_result(self,correct):
		'''
		update the current object retrieval result 
		'''
		self.childPrevAttemptCorrectness = self.childCurrAttemptCorrectness
		self.childCurrAttemptCorrectness = correct

		self.current_num_trials += 1
		if correct == True:
			# the child fails to find a correct target object 
			self.current_num_correct_trials +=1
			if self.num_available_target_objs != 0:
				self.correct_rate_arr.append(float(self.current_num_trials / self.num_available_target_objs))

	def update_qa_info(self,q_type,q_cmd):
		'''	
		update information about question & answer activity
		'''
		# update child's state: increment the number of questions asked
		self.num_robot_questions_asked += 1
		self.current_q_type = q_type
		if "yes/no" == q_type:
			self.numRobotYNQuestion +=1
		elif "open-ended" == q_type:
			self.numRobotOpenQuestion += 1

		if q_cmd == RobotBehaviors.Q_ROBOT_OFFER_HELP:
			self.numRobotOfferHelp += 1
		elif q_cmd == RobotBehaviors.Q_ROBOT_ASK_HELP:
			self.numRobotAskHelp += 1

	def update_qa_child_response(self,answered):
		'''
		update: whether the child answers a given question or not
		'''
		if answered == True:
			self.num_robot_questions_answered += 1
			if self.current_q_type == "yes/no":
				self.numRobotYNQuestionAnswered += 1
			elif self.current_q_type == "open-ended":
				self.numRobotOpenQuestionAnswered += 1


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


	