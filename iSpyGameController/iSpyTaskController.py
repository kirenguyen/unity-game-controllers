import csv 
import os
import random

from GameUtils import GlobalSettings

class iSpyTaskController():
	"""docstring for iSpyTaskController"""

	def __init__(self):
		# List that will hold the names of the target objects
		self.target_list = []

		self.available_quests = []

		self.task_in_progress = False

		self.load_task_list()
		self.load_object_list()

	def load_task_list(self):
		""" Loads the task_list csv file into a 2d array """
		dir_path = os.path.dirname(os.path.realpath(__file__))
		print (dir_path)

		self.task_dict = dict()

		# What one row will look like
		# {task_number: (task, category, attribute)}
		# Ex:
		# {1: ("What objects are related to weather?", object_type, weather)}
		# 
		with open(dir_path + '/../GameUtils/task_list2.csv','r') as csvfile:
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

		# What one row will look like
		# {object: (word, object_type, object_vocab, attribute, color)}
		# Ex:
		# {'bicycle': ('bicycle', 'vehicle', 'vehicle', '2wheels:road', 'emerald')}
		# WORKS WITH OBJECT_LIST2.CSV
		with open(dir_path + '/../GameUtils/object_list2.csv','r') as csvfile:
			spamreader = csv.reader(csvfile)
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

		# If the item has the attribute that matches the quest, add it to the list
		for key in self.object_dict:
			if task_attribute in self.object_dict[key][category_index]:
				self.target_list.append(key)

		self.task_in_progress = True

		task_message = self.get_task_message(str(len(self.target_list)),task_category,task_attribute)
		return {"task": task_message, "task_vocab": task_attribute, "list" : self.target_list}

	def get_task_message(self, how_many, task_category, task_attribute):
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

	def get_random_target_obj():
		'''
		randomly select an object on the target list to return
		'''
		ran_index = random.randint(0, len(self.target_list)-1)
		return self.target_list[ran_index]

		


