
"""
This is a basic class for the Game Controller
"""
import json
import time

from GameUtils.AudioRecorder import AudioRecorder
from .iSpyTaskController import iSpyTaskController
import rospy
import _thread as thread
# -*- coding: utf-8 -*-
# pylint: disable=import-error
from transitions import Machine

# from GameUtils import Curriculum
from GameUtils import GlobalSettings
from GameUtils.GlobalSettings import iSpyGameStates as gs
from GameUtils.PronunciationUtils.PronunciationUtils import PronunciationUtils

# from StudentModel import StudentModel

if GlobalSettings.USE_ROS:
	from std_msgs.msg import Header  # standard ROS msg header
	from std_msgs.msg import String
	from unity_game_msgs.msg import iSpyCommand
	from unity_game_msgs.msg import iSpyAction
else:
	pass
	# TapGameLog = GlobalSettings.TapGameLog #Mock ob-ject, used for testing in non-ROS environments
	# TapGameCommand = GlobalSettings.TapGameCommand

#Recording Time Constant
RECORD_TIME_MS = 3500

#set up ros topics for ispy game
ROS_TO_ISPY_GAME_TOPIC = 'ispy_cmd_topic'
ISPY_GAME_TO_ROS_ACTION_TOPIC = 'ispy_action_topic'
ISPY_GAME_TO_ROS_TRANSITION_TOPIC = 'ispy_transition_state_topic'
ISPY_GAME_TO_ROS_LOG_TOPIC = 'ispy_log_topic'
ROS_TO_ANDROID_MIC_TOPIC = 'android_audio'

# COMMAND CONSTANTS
RESET = 0
SHOW_PRONOUNCIATION_PANEL = 1
SHOW_OBJECT_DESCR_PANEL = 2
SEND_PRONOUNCIATION_ACCURACY_TO_UNITY = 10
SEND_TASKS_TO_UNITY = 20
GAME_FINISHED = 99
VALID_ISPY_COMMANDS = [RESET, SHOW_PRONOUNCIATION_PANEL, SHOW_PRONOUNCIATION_PANEL, SEND_PRONOUNCIATION_ACCURACY_TO_UNITY, SEND_TASKS_TO_UNITY, GAME_FINISHED]


class iSpyGameFSM: # pylint: disable=no-member
	"""
	Receives and sends out ROS messages.
	"""

	def __init__(self):
		# Keeps track of the word the child is supposed to say
		self.origText = ""

		# The object of the iSpyAudioRecorder Class
		self.recorder = None

		# Publisher to the ROS_TO_ISPY_GAME_TOPIC
		self.game_commander = None

		# Bool telling if the cmd message was heard from Unity
		self.message_received = False

		self.task_controller = iSpyTaskController()

		self.results_handler = PronunciationUtils()

		# Times entered explore or mission mode not including on game start
		self.entered_explore_mode = 0
		self.entered_mission_mode = 0
		
		# Bool stating whether or not the current mission is completed
		self.mission_completed = True

		# List of tuples of what was tapped and the time it took to tap them
		self.mission_tapped_list = []
		# List of tuples of what was tapped and the time it was tapped
		self.explore_tapped_list = []
		# List of tuples of what was tapped and how long it took to hit the pronounce button
		self.tapped_and_pronounced = []
		# List of tuples of what was tapped and how long it took to hit the cancel button
		self.tapped_and_cancelled = []

		# self.student_model = StudentModel()
		#
		# self.log_listener = None
		#

		self.states = [gs.GAME_START, gs.EXPLORATION_MODE,gs.MISSION_MODE,gs.PRONUNCIATION_PANEL,gs.PRONUNCIATION_RESULT,gs.WORD_DISPLAY]
		self.transitions = [
				{'trigger': gs.Triggers.START_BUTTON_PRESSED, 'source': gs.GAME_START, 'dest': gs.EXPLORATION_MODE},
				{'trigger': gs.Triggers.TOPLEFT_BUTTON_PRESSED, 'source': gs.EXPLORATION_MODE, 'dest': gs.MISSION_MODE},
				{'trigger': gs.Triggers.TOPLEFT_BUTTON_PRESSED, 'source': gs.MISSION_MODE, 'dest': gs.EXPLORATION_MODE},
				{'trigger': gs.Triggers.OBJECT_CLICKED, 'source': gs.MISSION_MODE, 'dest': gs.PRONUNCIATION_PANEL},
				{'trigger': gs.Triggers.CLOSE_BUTTON_PRESSED, 'source': gs.PRONUNCIATION_PANEL, 'dest': gs.MISSION_MODE},
				{'trigger': gs.Triggers.SAY_BUTTON_PRESSED, 'source': gs.PRONUNCIATION_PANEL, 'dest': gs.PRONUNCIATION_RESULT},
				{'trigger': gs.Triggers.PRACTICE_FAILED, 'source': gs.PRONUNCIATION_RESULT, 'dest': gs.PRONUNCIATION_PANEL},
				{'trigger': gs.Triggers.CLOSE_BUTTON_PRESSED, 'source':gs.PRONUNCIATION_RESULT , 'dest': gs.MISSION_MODE},
				{'trigger': gs.Triggers.PRACTICE_FINISHED, 'source':gs.PRONUNCIATION_RESULT , 'dest': gs.MISSION_MODE},
				{'trigger': gs.Triggers.OBJECT_CLICKED, 'source': gs.EXPLORATION_MODE, 'dest':gs.WORD_DISPLAY },
				{'trigger': gs.Triggers.N_SECONDS_LATER, 'source': gs.WORD_DISPLAY, 'dest': gs.EXPLORATION_MODE}
		]
		
		self.state_machine = Machine(self, states=self.states, transitions=self.transitions,
									 initial=gs.GAME_START)


		self.override_FSM_transition_callback()

	

	def override_FSM_transition_callback(self):
		'''
		override callback functions for FSM transitions.
		When entering a particular state, the callback function for that state will run
		'''
		state_list = [gs.EXPLORATION_MODE,gs.MISSION_MODE,gs.PRONUNCIATION_PANEL, gs.PRONUNCIATION_RESULT, gs.WORD_DISPLAY]
		func_list = ['onExplorationMode','onMissionMode','onPronunciationPanel','onPronunciationResult','onWordDisplay']
		prefix='self.state_machine.on_enter_'
		for istate in state_list:
			func = func_list[state_list.index(istate)]
			pcode = prefix + istate + '(self.' + func + ')'
			exec(pcode)


	def onExplorationMode(self):
		'''callback function when exploration mode just starts'''
		if self.entered_explore_mode > 0:
			print ("Entered explore mode %s times" %self.entered_explore_mode)
		else:
			print ("Entered explore mode")
		

	def onMissionMode(self):
		'''callback function when entering mission mode '''
		self.mission_time_start = time.time()

		print ("Entered mission mode %s times" %self.entered_mission_mode)

	def onPronunciationPanel(self):
		'''callback function when entering pronunciation panel'''
		print ("Entered pronunciation panel")

	def onPronunciationResult(self):
		'''callback function when entering pronunciation result mode'''
		#TODO: obtain scores from speech class
		#TODO: Visualize word accuracy
		#TODO: check whether the user pronounces the word accurately and whether the word is a target
		print ("Entered pronunciation result")
		self.tapped_and_pronounced.append((self.origText, time.time() - self.time_tapped))
		print (self.tapped_and_pronounced)

	def onWordDisplay(self):
		'''callback function when entering word display'''
		#TODO: show word bubbles for the clicked object
		#TODO: set a timer variable. If the elapsed time > 5 secs, then the bubble disappear
		print ("Entered dialogue panel")

	def on_ispy_state_info_received(self,transition_msg):
		"""
		Rospy Callback for when we get log messages
		"""
		print ("---------------------------------------------------------------")
		rospy.loginfo(rospy.get_caller_id() + "I heard " + transition_msg.data)
		print("I heard " + transition_msg.data)

		if transition_msg.data in gs.Triggers.triggers:
			if transition_msg.data == gs.Triggers.TOPLEFT_BUTTON_PRESSED:
				# If the player is switching from mission to explore mode
				if self.state == gs.MISSION_MODE:
					# Incremement how many times entered explore mode
					self.entered_explore_mode += 1
					# Start keeping track of how long in explore mode
					self.explore_time_start = time.time()

				if self.state == gs.EXPLORATION_MODE:
					# If the player is switching from explore to mission mode
					# Print how long the player was in explore mode
					self.entered_mission_mode += 1

					# When entering mission mode from exploration mode, get a random task
					# and send it to Unity
					if self.task_controller.task_in_progress == False:
						task = self.task_controller.get_random_task()

						# If there are no more available quests, you won the game
						if task == None:
							self.send_ispy_cmd(GAME_FINISHED)
						else:
							self.send_ispy_cmd(SEND_TASKS_TO_UNITY, task)

					time_in_explore_mode = time.time() - self.explore_time_start
					print("Time spent in explore mode is %s seconds" %time_in_explore_mode)

			# Starts keeping track of time in explore mode when the game starts
			elif transition_msg.data == gs.Triggers.START_BUTTON_PRESSED:
				self.explore_time_start = time.time()

			elif transition_msg.data == gs.Triggers.OBJECT_CLICKED:
				time.sleep(.1)

				# Keep track of when the object was clicked
				self.time_tapped = time.time()

				if self.state == gs.MISSION_MODE:
					# If coming from missioin mode, append to mission mode list
					self.mission_tapped_list.append((self.origText, self.time_tapped - self.mission_time_start))
					print(self.mission_tapped_list)

				elif self.state == gs.EXPLORATION_MODE:
					# If coming from explore mode, append to explore mode list
					self.explore_tapped_list.append((self.origText, self.time_tapped))
					print (self.explore_tapped_list)

			elif transition_msg.data == gs.Triggers.CLOSE_BUTTON_PRESSED:
				# If closing the pronunciation panel, append to the tapped and cancelled list
				if self.state == gs.PRONUNCIATION_PANEL:
					self.tapped_and_cancelled.append((self.origText , time.time() - self.time_tapped))
					print (self.tapped_and_cancelled)


			# If the message is in gs.Triggers, then allow the trigger
			getattr(self, transition_msg.data)()

	#################################################################################################

	def on_ispy_log_received(self, log_msg):
		print(log_msg.data)
		# If the the message was "messageReceived", that means that the publishing loop can stop
		if log_msg.data == "messageReceived":
			self.message_received = True


	#################################################################################################
	def on_ispy_action_received(self, ispy_action_msg):
		"""
		Rospy callback for when we get ispy action from the unity game over ROS
		"""
		
		def isScalingUp(boolean):			
			if boolean:
				print ("Fine at this point")
		def isScalingDown(boolean):
			if boolean:
				print ("Fine at this point")
		def pointerClick(boolean):
			if boolean:
				print ("Fine at this point")
		def pointerPosition(coord_list):
			pass
			# if coord_list:
			# 	print ("Fine at this point")
		def isDragging(boolean):
			if boolean:
				print ("Fine at this point")
		def clickedObjectName(object_name):
			if object_name:	
				self.origText = object_name		#Sets origText to the object clicked

		def onPinch(boolean):
			if boolean:
				print ("Fine at this point")

		def speakingStage(stage):
			if stage == "speakingStart":
				self.recorder.start_recording(self.origText, RECORD_TIME_MS) #TODO: Update 'test' to actual word
			elif stage == "speakingEnd":
				self.recorder.stop_recording()

	
		
		def msg_evaluator(ispy_action_msg):
			"""Calls the respective functions for each part of the action msg
			"""

			isScalingUp(ispy_action_msg.isScalingUp)
			isScalingDown(ispy_action_msg.isScalingDown)
			pointerClick(ispy_action_msg.pointerClick)
			pointerPosition(ispy_action_msg.pointerPosition)
			isDragging(ispy_action_msg.isDragging)
			onPinch(ispy_action_msg.onPinch)

			#Removes object position from ispy_action_msg
			object_name = ""
			for letter in ispy_action_msg.clickedObjectName:
				if letter == "-":
					break
				object_name += letter

			clickedObjectName(object_name)

			#Initializes a new audio recorder object if one hasn't been created
			if self.recorder == None:
				self.recorder = AudioRecorder()
	
			# print (ispy_action_msg.speakingStage)
			speakingStage(ispy_action_msg.speakingStage)


		# Evaluates the action message
		msg_evaluator(ispy_action_msg)

		# If given a word to evaluate and done recording send the information to speechace
		if self.origText and self.recorder.has_recorded % 2 == 0 and self.recorder.has_recorded != 0:
			# If you couldn't find the android audio topic, automatically pass
			# instead of using the last audio recording
			if not self.recorder.valid_recording:
				letters = list(self.origText)
				passed = ['1'] * len(letters)
				print ("NO, RECORDING SO YOU AUTOMATICALLY PASS")

			else:
				audioFile = self.recorder.WAV_OUTPUT_FILENAME_PREFIX + self.origText + '.wav'
				word_score_list = self.recorder.speechace(audioFile)

				if word_score_list:
					for word in word_score_list:
						letters, passed = self.results_handler.process_speechace_word_results(word)
					print ("Message to Unity")
					print (letters)
					print (passed)

				else:
					letters = list(self.origText)
					passed = ['1'] * len(letters)
					print ("NO, RECORDING SO YOU AUTOMATICALLY PASS")
				
			results_params = {}
			results_params["letters"] = letters
			results_params["passed"] = passed

			self.send_ispy_cmd(SEND_PRONOUNCIATION_ACCURACY_TO_UNITY, results_params)
			self.recorder.has_recorded = 0
			

	def start_ispy_action_listener(self):
		"""
		Start up the ispy action Subscriber node
		"""

		rospy.init_node('ispy_ROS_receiver', anonymous = True)
		rospy.Subscriber(ISPY_GAME_TO_ROS_ACTION_TOPIC, iSpyAction, self.on_ispy_action_received)	

		rospy.spin()

	def start_ispy_transition_listener(self):
		"""
		Start up the ispy state Subscriber node
		"""
		time.sleep(.2)
		rospy.Subscriber(ISPY_GAME_TO_ROS_TRANSITION_TOPIC, String, self.on_ispy_state_info_received)	

		rospy.spin()

	def start_ispy_log_listener(self):
		rospy.Subscriber(ISPY_GAME_TO_ROS_LOG_TOPIC, String, self.on_ispy_log_received)

	def start_ispy_cmd_publisher(self):
		"""
		Starts up the ispy command publisher node,
		which allows iSpy controller sends cmd messages over ROS to the unity game
		"""
		print('Pub Node started')
		self.game_commander = rospy.Publisher(ROS_TO_ISPY_GAME_TOPIC, iSpyCommand, queue_size=10)
		rate = rospy.Rate(10)  # spin at 10 Hz
		rate.sleep()  # sleep to wait for subscribers
		# rospy.spin()


	def send_ispy_cmd(self, command, *args):
		"""
		send a iSpyCommand to unity game
		Args are optional parameters.
		"""
		msg = iSpyCommand()
		# add header
		msg.header = Header()
		msg.header.stamp = rospy.Time.now()

		# Bool for if the word is pronounced completely correctly
		# Used to reuse the origText variable to try pronouncing again
		perfect_word = True

		#TODO: may need to convert args to json before passing it to msg.params
		#if command is SEND_PRONOUNCIATION_ACCURACY_TO_UNITY, then convert accuracy result to JSOn message first
		#use function in Util.py

		if command in VALID_ISPY_COMMANDS:
			# fill in command and any params:
			msg.command = command
			if len(args) > 0:
				if command == SEND_PRONOUNCIATION_ACCURACY_TO_UNITY:
					# Checks each letter and if one letter is False then the word is not perfectly said
					passed = args[0]["passed"]
					for i in passed:
						if i == '0':
							perfect_word = False
							break

					# If the word was pronounced perfectly then reset origText
					if perfect_word:
						if self.task_controller.isTarget(self.origText):
							self.task_controller.update_target_list(self.origText)

						self.origText = ""

					
					msg.properties = json.dumps(args[0])

				elif command == SEND_TASKS_TO_UNITY:
					converted_result = json.dumps(args[0])
					msg.properties = converted_result

			# send message to tablet game
			if self.game_commander is None:
				self.start_ispy_cmd_publisher()

			# Keep sending the message until hearing that it was received
			while self.message_received == False:
				self.game_commander.publish(msg)
				time.sleep(.15)

			self.message_received = False
			rospy.loginfo(msg)

		else:
			print ("Not a valid command")

if __name__ == '__main__':
	control = iSpyGameFSM()
	print ("FSM Started!")
	thread.start_new_thread(control.start_ispy_transition_listener, ())
	thread.start_new_thread(control.start_ispy_log_listener, ())
	control.start_ispy_action_listener()

