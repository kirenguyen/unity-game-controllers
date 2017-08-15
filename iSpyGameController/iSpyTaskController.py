import csv 
import os
import random

class iSpyTaskController():
	"""docstring for iSpyTaskController"""

	def __init__(self):
		# List that will hold the names of the target objects
		self.target_list = []

		self.available_quests = []

		self.task_in_progress = False

		self.load_task_list()

		# Checks which csv file to use for the tasks
		# if GlobalSettings.SCENE == "BEDROOM":
		# 	self.load_bedroom_object_list()
		# elif GlobalSettings.SCENE == "OUTDOORS":
		# 	self.load_outdoors_object_list()
		self.load_object_list()

	def load_task_list(self):
		""" Loads the task_list csv file into a 2d array """
		dir_path = os.path.dirname(os.path.realpath(__file__))

		self.task_dict = dict()

		# if GlobalSettings.SCENE == "BEDROOM":
		task_file = '/iSpyGameUtils/game_task_list.csv'
		# elif GlobalSettings.SCENE == "OUTDOORS":
		# 	task_file = '/iSpyGameUtils/task_list_outdoors.csv'

		# What one row will look like
		# {task_number: (task, category, attribute)}
		# Ex:
		# {1: ("What objects are related to weather?", object_type, weather)}

		with open(dir_path + task_file,'r') as csvfile:
			spamreader = csv.reader(csvfile, delimiter=',')
			for row in spamreader:
				if row[0] != "":
					self.task_dict[int(row[0])] = (row[1], row[2], row[3])

		# Fill the available quests list with the ID of all the quests
 		for i in self.task_dict:
			self.available_quests.append(i)

	def load_object_list(self):
		""" Loads the object_list csv file into a 2d array """
		dir_path = os.path.dirname(os.path.realpath(__file__))

		self.object_dict= dict()
		# What one row will look like
		# {object: (word, color, size, object_type)}
		# Ex:
		# {'clock': ('clock', 'white', 'average', 'home')}

		with open(dir_path + '/iSpyGameUtils/object_list.csv','r') as csvfile:
			spamreader = csv.reader(csvfile, delimiter=',')
			for row in spamreader:
				if row[0] != "" and row[0] != "key_object":
					self.object_dict[row[0]] = (row[2], row[3], row[4], row[5])

	def load_outdoors_object_list(self):
		""" Loads the object_list csv file into a 2d array """
		dir_path = os.path.dirname(os.path.realpath(__file__))

		self.object_dict= dict()
		# What one row will look like
		# {object: (word, color, size, object_type, location, attribute)}
		# Ex:
		# {'airplane': ('airplane', 'red', 'huge', 'sky', 'wings')}

		with open(dir_path + '/iSpyGameUtils/object_list_outdoors.csv','r') as csvfile:
			spamreader = csv.reader(csvfile, delimiter=',')
			for row in spamreader:
				if row[0] != "" and row[0] != "key_object":
					self.object_dict[row[0]] = (row[1], row[2], row[3], row[4], row[5], row[6])


	def get_random_task(self):
		""" Gets a random task number from the list of available quests and returns the task and the targets"""

		# If there are no more available quests, you won the game and return None
		if len(self.available_quests) == 0:
			return

		# Gets a random index from the list of available quests, and used the value as the key in the task_dict
		index = random.randint(0, len(self.available_quests)-1)
		task, task_category, task_attribute = self.task_dict[self.available_quests[index]]

		# Delete the quest after it is chosen
		del self.available_quests[index]

		# Get the index in the dicitonary from a given category. See object_dict for indices
		category_index = self.category_to_index(task_category)
		print (category_index)

		# If the item has the attribute that matches the quest, add it to the list
		for key in self.object_dict:
			if task_attribute in self.object_dict[key][category_index]:
				self.target_list.append(key)

		self.task_in_progress = True

		return {"task": task, "list" : self.target_list}


	def category_to_index(self, category):
		""" Find what the index is for the given category """
		if category == "word":
			return 0
		elif category == "color":
			return 1
		elif category == "size":
			return 2
		elif category == "object_type":
			return 3
		elif category == "location":
			return 4
		elif category == "attribute":
			return 5

	def isTarget(self, object_name):
		""" Returns whether or not the object is a target """
		if object_name in self.target_list:
			return True
		return False

	def update_target_list(self, object_name):
		""" Updates the list of targets when one is found """
		self.target_list.remove(object_name)

		if len(self.target_list) == 0:
			self.task_in_progress = False

		print (self.target_list)
		


