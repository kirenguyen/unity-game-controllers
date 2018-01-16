from GameUtils.GlobalSettings import iSpyRobotInteractionStates as ris


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


		
	def set_num_available_objs(self,_num_available_target_objs):
		self.num_available_target_objs = _num_available_target_objs	
	
	def update_child_turn_result(self,correct):
		'''
		update the current object retrieval result 
		'''
		self.current_num_trials += 1
		if correct == True:
			# the child fails to find a correct target object 
			self.current_num_correct_trials +=1
			if self.num_available_target_objs != 0:
				self.correct_rate_arr.append(float(self.current_num_trials / self.num_available_target_objs))


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


	