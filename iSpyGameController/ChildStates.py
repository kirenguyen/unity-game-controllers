
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

		# 
		self.current_num_trials = 0
		
	def set_num_available_objs(self,_num_available_target_objs):
		self.num_available_targeet_objs = _num_available_target_objs	
	
	def update_child_turn_result(self,correct):
		'''
		update the current object retrieval result 
		'''
		if correct == False:
			# the child fails to find a correct target object 
			self.current_num_trials += 1
		else:
			pass
			#self.correct_rate_arr.append(float(self.current_num_trials / self.num_available_target_objs))





	