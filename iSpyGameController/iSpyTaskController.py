import csv 
import os
import random
import datetime

from GameUtils import GlobalSettings

# number of words that need to be retrieved in order to complete the task
NUM_WORDS_THRESHOLD =4

class iSpyTaskController():
	"""docstring for iSpyTaskController"""

	def __init__(self,game_round,session_number):
		# List that will hold the names of the target objects
		self.target_list = []

		self.nontarget_list = []

		self.available_quests = []

		self.task_in_progress = False

		# number of words that have been retrieved
		self.num_finished_words = 0

		self.current_task_index = 0

		# vocab word in the prompts
		self.vocab_word = ""

		self.task_start_time = None

		self.task_end_time = None

		self.task_duration = ""


		self.load_task_list(game_round,session_number)
		self.load_object_list(game_round,session_number)

	def get_vocab_word(self):
		return self.vocab_word

	def get_task_time(self):
		'''
		get task start time, end time and duration
		'''
		return {'start':str(self.task_start_time) , 'end': str(self.task_end_time), 'duration':self.task_duration}

	def load_task_list(self,game_round,session_number):
		""" Loads the task_list csv file into a 2d array """
		dir_path = os.path.dirname(os.path.realpath(__file__))
		print (dir_path)

		self.task_dict = dict()

		# What one row will look like
		# {task_number: (task, category, attribute)}
		# Ex:
		# {1: ("What objects are related to weather?", object_type, weather)}
		file_name =  '/../GameUtils/task_list_practice.csv' if game_round == "practice" else '/../GameUtils/task_list_experiment.csv'
		if session_number == "s02":
			file_name = '/../GameUtils/task_list_indoor.csv'

		
		with open(dir_path + file_name,'r') as csvfile:
			spamreader = csv.reader(csvfile, delimiter=',')
			for row in spamreader:
				if row[0] != "":
					self.task_dict[int(row[0])] = (row[1], row[2], row[3],row[4]) # 2: category 3: vocab word 4: audio file name
					

		# Fill the available quests list with the ID of all the quests
		for i in self.task_dict:
			self.available_quests.append(i)
		for i in self.task_dict:
			self.available_quests.append(i)



	def load_object_list(self, game_round,session_number):
		""" Loads the object_list csv file into a 2d array """
		dir_path = os.path.dirname(os.path.realpath(__file__))
		print (dir_path)

		self.object_dict= dict()

		# What one row will look like
		# {object: (word, color, size, object_type, location, attribute)}
		# Ex:
		# {'airplane': ('airplane', 'red', 'huge', 'sky', 'wings')}
		"""
		# WORKS WITH OBJECT_LIST.CSV
		with open(dir_path + '/../GameUtils/object_list.csv','r') as csvfile:
			spamreader = csv.reader(csvfile)
			for row in spamreader:
				if row[0] != "" and row[0] != "key_object":
					self.object_dict[row[0]] = (row[1], row[2], row[3], row[4], row[5], row[6])
		"""

		# INDOOR : object_list_indoor.csv
		# OUTDOOR : object_list3.csv
		if session_number == "s02":
			filename = 'object_list_indoor.csv'
		else:
			filename = "object_list3.csv"

		# What one row will look like
		# {object: (word, object_type, object_vocab, attribute, color)}
		# Ex:
		# {'bicycle': ('bicycle', 'vehicle', 'vehicle', '2wheels:road', 'emerald')}
		# WORKS WITH OBJECT_LIST2.CSV
		with open(dir_path + '/../GameUtils/'+ filename,'r') as csvfile:
			spamreader = csv.reader(csvfile)
			for row in spamreader:
				if row[0] != "" and row[0] != "key_object":
					self.object_dict[row[0]] = (row[1], row[2], row[3], row[4], row[5], row[6],row[7],row[8],row[9],row[10],row[11],row[12],row[13],row[14])


	def get_next_task(self):
		""" Gets a random task number from the list of available quests and returns the task and the targets"""

		# If there are no more available quests, you won the game and return None
		if len(self.available_quests) == 0:
			return

		# Gets a random index from the list of available quests, and used the value as the key in the task_dict
		#index = random.randint(0, len(self.available_quests)-1)
		index = 0

		self.current_task_index += 1 # update current task index


		
		task, task_category, task_attribute, prompt_audio_name = self.task_dict[self.available_quests[index]]

		self.vocab_word = task_attribute

		
		# Delete the quest after it is chosen
		del self.available_quests[index]

		# Get the index in the dicitonary from a given category. See object_dict for indices
		category_index = self.category_to_index(task_category)
		
		
		# If the item has the attribute that matches the quest, add it to the list
		for key in self.object_dict:
			
			#print(self.object_dict[key][category_index])
			if task_attribute in self.object_dict[key][category_index]:
				self.target_list.append(key)
			else:
				self.nontarget_list.append(key)

		self.task_in_progress = True



		task_message = self.get_task_message(str(len(self.target_list)),task_category,task_attribute)

		self.task_start_time = datetime.datetime.now()
		self.task_end_time = None
		self.task_duration = ""
		
		return {"task": task_message, "task_vocab": task_attribute, "list" : self.target_list,"prompt_audio_name":prompt_audio_name}

	def get_task_message(self, how_many, task_category, task_attribute):
		obj = ""
		if task_category == "color":
			obj = "things with color " + task_attribute
		elif task_category == "animal":
			obj = task_attribute + " animals"

		task_message = "Can you find " + how_many + " " + obj + "?"
		return task_message

	def category_to_index(self, category):
		""" Find what the index is for the given category """
		if category == "word":
			return 0
		elif category == "object_type":
			return 1
		elif category == "object_vocab":
			return 2
		elif category == "attribute":
			return 3
		elif category == "color":
			return 4
		elif category == "animal":
			return 5
		elif category == "vehicle":
			return 6
		elif category == "emotion":
			return 7
		elif category == "action":
			return 8
		elif category == "aquatic":
			return 9
		elif category == "size":
			return 10
		elif category == "activity":
			return 11
		elif category == "cloth":
			return 12
		elif category == "land":
			return 13

	def isTarget(self, object_name):
		""" Returns whether or not the object is a target """
		if object_name in self.target_list:
			return True
		return False

	def update_target_list(self, object_name):
		""" Updates the list of targets when one is found """
		
		self.target_list.remove(object_name)

		self.num_finished_words = self.num_finished_words + 1

		if self.num_finished_words == NUM_WORDS_THRESHOLD:
			print("**update target list: task in progress false")
			self.task_in_progress = False
			self.task_end_time = datetime.datetime.now()
			self.task_duration = str(self.task_end_time - self.task_start_time)
			self._reset_for_new_task()

			#self.current_task_index += 1

	def get_current_answer_size(self):
		pass

	def _reset_for_new_task(self):
		'''
		reset target list, nontarget list 
		clear data of the last finished task for the new task
		'''
		self.target_list = []
		self.nontarget_list = []
		#self.num_finished_words = 0



	'''
	the following functions are for child robot interaction
	they are called by ChildRobotInteractionFSM.py
	'''

	def get_random_target_obj(self):
		'''
		randomly select an object on the target list to return
		'''
		ran_index = random.randint(0, len(self.target_list)-1)
		return self.target_list[ran_index]

	def get_a_nontarget_object(self):
		'''
		return a nontarget object
		'''
		ran_index = random.randint(0, len(self.nontarget_list)-1)
		return self.nontarget_list[ran_index]
			

	
	def get_obj_for_robot(self,correct):
		'''
		return an object for robot to spy and pronounce
		'''
		
		if correct == True:
			# return a correct target object for now
			ran_index = random.randint(0, len(self.target_list)-1)
			return self.target_list[ran_index]
		else:
			return self.get_a_nontarget_object()

	def get_num_available_target_objs(self):
		'''
		return the number of available target objects 
		'''
		return len(self.target_list)


		


