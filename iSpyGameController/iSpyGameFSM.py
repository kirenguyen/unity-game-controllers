
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
from GameUtils.GlobalSettings import iSpyRobotInteractionStates as ris
from GameUtils.PronunciationUtils.PronunciationUtils import PronunciationUtils



from .ROSNodeMgr import ROSNodeMgr
#from .StudentModel import StudentModel
from .RobotBehaviorList.RobotBehaviorList import RobotBehaviors
from .RobotBehaviorList.RobotBehaviorList import RobotRoles
from .RobotBehaviorList.RobotBehaviorList import RobotRolesBehaviorsMap

from .RoleSwitchingPrj.ChildRobotInteractionFSM import ChildRobotInteractionFSM
from .GameModeFSMs import AlwaysMissionModeFSM,CompleteModeFSM,AlwaysExploreModeFSM

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

# COMMAND CONSTANTS
RESET = 0
SHOW_PRONOUNCIATION_PANEL = 1
SHOW_OBJECT_DESCR_PANEL = 2
ROBOT_EXPERT_ROLE = 3
SEND_PRONOUNCIATION_ACCURACY_TO_UNITY = 10
SEND_TASKS_TO_UNITY = 20
GAME_FINISHED = 99
BUTTON_DISABLED=31
VALID_ISPY_COMMANDS = [RESET, SHOW_PRONOUNCIATION_PANEL, SHOW_PRONOUNCIATION_PANEL, SEND_PRONOUNCIATION_ACCURACY_TO_UNITY, SEND_TASKS_TO_UNITY, GAME_FINISHED,BUTTON_DISABLED]






class iSpyGameFSM: # pylint: disable=no-member
	"""
	Receives and sends out ROS messages.
	"""


	def __init__(self):

		self.ros_node_mgr = ROSNodeMgr()
		self.ros_node_mgr.init_ros_node()

		

		# Keeps track of the word the child is supposed to say
		self.origText = ""

		# The object of the iSpyAudioRecorder Class
		self.recorder = None

		# Publisher to the ROS_TO_ISPY_GAME_TOPIC
		self.game_commander = None

		# Bool telling if the cmd message was heard from Unity
		self.ros_node_mgr.message_received = False

		self.task_controller = iSpyTaskController()

		self.results_handler = PronunciationUtils()

		self.interaction = ChildRobotInteractionFSM(self.ros_node_mgr,self.task_controller)


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

		# choose which game FSM to call
		# AlwaysMissionModeFSM(self.ros_node_mgr) # CompleteModeFSM() # AlwaysExploreModeFSM(self.ros_node_mgr)
		self.FSM = AlwaysMissionModeFSM(self.ros_node_mgr)

		self.override_FSM_transition_callback()

		


	def override_FSM_transition_callback(self):
		'''
		override callback functions for FSM transitions.
		When entering a particular state, the callback function for that state will run
		'''
		state_list = [gs.EXPLORATION_MODE,gs.MISSION_MODE,gs.PRONUNCIATION_PANEL, gs.PRONUNCIATION_RESULT, gs.WORD_DISPLAY]
		func_list = ['onExplorationMode','onMissionMode','onPronunciationPanel','onPronunciationResult','onWordDisplay']
		prefix='self.FSM.state_machine.on_enter_'
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
		self._run_game_task()

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
		print("I heard " + transition_msg.data)
		if transition_msg.data in gs.Triggers.triggers:
			if transition_msg.data == gs.Triggers.TOPLEFT_BUTTON_PRESSED:
				# robot celebrate 
				self.interaction.react(gs.Triggers.TOPLEFT_BUTTON_PRESSED)


				# If the player is switching from mission to explore mode
				if self.FSM.get_state() == gs.MISSION_MODE:
					# Incremement how many times entered explore mode
					self.entered_explore_mode += 1
					# Start keeping track of how long in explore mode
					self.explore_time_start = time.time()


				if self.FSM.get_state() == gs.EXPLORATION_MODE:
					# If the player is switching from explore to mission mode
					# Print how long the player was in explore mode
					self.entered_mission_mode += 1


			elif transition_msg.data == gs.Triggers.OBJECT_CLICKED:
				time.sleep(.1)
				self._on_obj_clicked()
				

			elif transition_msg.data == gs.Triggers.PRONUNCIATION_PANEL_CLOSED:
				# If the user closes the pronunciation panel, append to the tapped and cancelled list

				if self.FSM.get_state() == gs.PRONUNCIATION_PANEL:
					self.tapped_and_cancelled.append((self.origText , time.time() - self.time_tapped))
					print (self.tapped_and_cancelled)
				elif self.FSM.get_state() == gs.PRONUNCIATION_RESULT:
					self.interaction.react(gs.Triggers.PRONUNCIATION_PANEL_CLOSED)
					self.interaction.turn_taking()

			elif transition_msg.data == gs.Triggers.TARGET_OBJECT_COLLECTED:
				# one of the target objects is successfully collected. give the turn to the other player now
				
				self.interaction.react(gs.Triggers.TARGET_OBJECT_COLLECTED)
				self.interaction.turn_taking()

			elif transition_msg.data == gs.Triggers.SAY_BUTTON_PRESSED:
				if self.task_controller.isTarget(self.origText):
					self.interaction.react(gs.Triggers.SAY_BUTTON_PRESSED)
				else:
					self.interaction.react(gs.Triggers.SAY_BUTTON_PRESSED, 1)

			elif transition_msg.data == gs.Triggers.SCREEN_MOVED:
				self.interaction.react(gs.Triggers.SCREEN_MOVED)

			# If the message is in gs.Triggers, then allow the trigger

			if transition_msg.data != gs.Triggers.SCREEN_MOVED:
				self.FSM.start_trigger(transition_msg.data)

	#################################################################################################

	def on_ispy_log_received(self, log_msg):
		print(log_msg.data)
		# If the the message was "messageReceived", that means that the publishing loop can stop
		if log_msg.data == "messageReceived":
			self.ros_node_mgr.message_received = True


	#################################################################################################
	def on_ispy_action_received(self, ispy_action_msg):
		"""
		Rospy callback for when we get ispy action from the unity game over ROS
		"""

		print("ispy action msg")
		#print(ispy_action_msg)
		
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

		self._speechace_analysis()

		
			
	def _speechace_analysis(self):
		'''
		speech ace analysis
		'''
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

			# Checks each letter and if one letter is False then the word is not perfectly said
			perfect_word = True
			for i in passed:
				if i == '0':
					perfect_word = False
					break
			# If the word was pronounced perfectly then reset origText
			if perfect_word:
				if self.task_controller.isTarget(self.origText):
					self.task_controller.update_target_list(self.origText)
					self.origText = ""

			self.ros_node_mgr.send_ispy_cmd(SEND_PRONOUNCIATION_ACCURACY_TO_UNITY, results_params)
			self.recorder.has_recorded = 0
	
	
	def _run_game_task(self):
		# When entering mission mode from exploration mode, get a random task
		# and send it to Unity
		print("!!!!!! _run game task...")
		if self.task_controller.task_in_progress == False:
			print("get a random task!!!")
			task = self.task_controller.get_random_task()

			# If there are no more available quests, you won the game
			if task == None:
				self.ros_node_mgr.send_ispy_cmd(GAME_FINISHED)
			else:
				self.ros_node_mgr.send_ispy_cmd(SEND_TASKS_TO_UNITY, task)
				self.interaction.get_turn_taking_actions()


	def _on_obj_clicked(self):
		# Keep track of when the object was clicked
		self.time_tapped = time.time()

		if self.FSM.get_state() == gs.MISSION_MODE:
			# If coming from missioin mode, append to mission mode list
			self.mission_tapped_list.append((self.origText, self.time_tapped - self.mission_time_start))
			print(self.mission_tapped_list)

			if self.task_controller.isTarget(self.origText):
				self.interaction.react(gs.Triggers.OBJECT_CLICKED)
			else:
				self.interaction.react(gs.Triggers.OBJECT_CLICKED, 1)

		elif self.FSM.get_state() == gs.EXPLORATION_MODE:
			# If coming from explore mode, append to explore mode list
			self.explore_tapped_list.append((self.origText, self.time_tapped))
			print (self.explore_tapped_list)
	



