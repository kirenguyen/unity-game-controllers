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


		self.total_num_trials = 0 # total number of trials

		# keep track of whose turn it is
		self.current_turn = ""

		# how many questions robot has asked the child
		self.num_robot_questions_asked = 0 # updated by ChildRobotInteraction

		self.num_robot_questions_answered = 0  # updated by ChildRobotInteraction

		self.child_answer_content = ""


		# no tablet touch alert 
		self.numTouchAbsenceAlertPerTask = 0


		self.numChildCorrectAttemptsCurrTask = 0

		self.numChildAttemptsCurrTask = 0

		self.objectWordPronounced = False


		self.pos_answers = 0 
		
		self.neg_answers = 0
		
		self.other_answers = 0 
		
		self.no_answers_attempt1 = 0

		self.no_answers_attempt2 = 0



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


	def set_num_available_objs(self,_num_available_target_objs):
		self.num_available_target_objs = _num_available_target_objs	
	

	def update_turn_result(self,whoseTurn,correct):
		'''
		update the current object retrieval result 
		'''
		self.total_num_trials += 1
		if whoseTurn == "childTURN" or "childHelp" in whoseTurn:
			self.numChildAttemptsCurrTask += 1
			if correct == True: # the child fails to find a correct target object 
				self.numChildCorrectAttemptsCurrTask +=1

			

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

	# def update_qa_child_response(self,answered):
	# 	'''
	# 	update: whether the child answers a given question or not
	# 	'''
	# 	if answered == True:
	# 		self.num_robot_questions_answered += 1
	# 		if self.current_q_type == "yes/no":
	# 			self.numRobotYNQuestionAnswered += 1
	# 		elif self.current_q_type == "open-ended":
	# 			self.numRobotOpenQuestionAnswered += 1


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


	