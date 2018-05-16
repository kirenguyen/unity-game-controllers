
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
import threading



# from GameUtils import Curriculum
from GameUtils import GlobalSettings
from GameUtils.GlobalSettings import iSpyGameStates as gs
from GameUtils.GlobalSettings import iSpyRobotInteractionStates as ris
from GameUtils.PronunciationUtils.PronunciationUtils import PronunciationUtils



from .ROSNodeMgr import ROSNodeMgr
from .iSpyDataTracking import iSpyDataTracking
#from .StudentModel import StudentModel
from .RobotBehaviorList.RobotBehaviorList import RobotBehaviors 
from .RobotBehaviorList.RobotBehaviorList import RobotRoles
from .RobotBehaviorList.RobotBehaviorList import RobotRolesBehaviorsMap

from .AffdexAnalysis.node_AffdexResponse import AffdexAnalysis

from .RoleSwitchingPrj.ChildRobotInteractionFSM import ChildRobotInteractionFSM

from .GameModeFSMs import AlwaysMissionModeFSM,CompleteModeFSM,AlwaysExploreModeFSM

from multiprocessing import Process

import datetime 
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
TASK_COMPLETED = 32
VALID_ISPY_COMMANDS = [RESET, SHOW_PRONOUNCIATION_PANEL, SHOW_PRONOUNCIATION_PANEL, SEND_PRONOUNCIATION_ACCURACY_TO_UNITY, SEND_TASKS_TO_UNITY, GAME_FINISHED,BUTTON_DISABLED]
SET_GAME_SCENE = 34





class iSpyGameFSM: # pylint: disable=no-member
	"""
	Receives and sends out ROS messages.
	"""


	def __init__(self,participant_id, experimenter, session_number):

		self.ros_node_mgr = ROSNodeMgr()
		self.ros_node_mgr.init_ros_node()

		self.session_number = session_number

		# Keeps track of the word the child is supposed to say
		self.origText = ""

		# check whether the clicked object is a target object
		self.correct_obj = False

		# The object of the iSpyAudioRecorder Class
		self.recorder = None

		# Publisher to the ROS_TO_ISPY_GAME_TOPIC
		self.game_commander = None

		# Bool telling if the cmd message was heard from Unity
		self.ros_node_mgr.message_received = False

		self.task_controller = iSpyTaskController(session_number)

		self.results_handler = PronunciationUtils()

		self.interaction = ChildRobotInteractionFSM(self.ros_node_mgr,self.task_controller,self, participant_id,session_number)

		self.iSpyDataTracking = iSpyDataTracking(self.interaction,self.ros_node_mgr, participant_id, experimenter, session_number)

		# Bool stating whether or not the current mission is completed
		self.mission_completed = True


		# choose which game FSM to call
		# AlwaysMissionModeFSM(self.ros_node_mgr) # CompleteModeFSM() # AlwaysExploreModeFSM(self.ros_node_mgr)
		self.FSM = AlwaysMissionModeFSM(self.ros_node_mgr,session_number)


		if session_number != "practice":
			self.affdexAnalysis = AffdexAnalysis(self,self.ros_node_mgr,participant_id,experimenter, session_number)

		self.kill_received = False # for stopping the update() thread


		self.current_task_index = 0

		# start a thread to check the game update
		#self.t = threading.Thread(target=self.update)
		#self.t.start()
		
		
	def update(self):
		while self.kill_received == False:
			if self.FSM.state != gs.MISSION_MODE:
				self.iSpyDataTracking.stop_tracking_child_interaction()
			if self.interaction.state != ris.CHILD_TURN and self.interaction.state != ris.ROBOT_TURN+'_'+ris.CHILD_HELP:
				self.iSpyDataTracking.stop_tracking_child_interaction()

			if self.kill_received == True:
				break

	def _reach_max_task_time(self): # if the condition is in "fixed novice": set a max elapsed time
			if self.interaction.subj_cond != "novice": return
			
			max_elapsed_time = datetime.timedelta(seconds=4.5*60) # 5 mins
			max_elapsed_time2 = datetime.timedelta(seconds=6.5*60) # 5 mins
			if datetime.datetime.now() - self.task_controller.get_task_time()['start'] > max_elapsed_time:
				if self.task_controller.num_finished_words <= 2: self.task_controller.reset_for_new_task()
			
			if datetime.datetime.now() - self.task_controller.get_task_time()['start'] > max_elapsed_time2:
				if self.task_controller.num_finished_words <= 3: self.task_controller.reset_for_new_task()


	def on_ispy_state_info_received(self,transition_msg):
		"""
		Rospy Callback for when we get log messages from ispy game
		"""
		def check_task_completion():
			if not self.task_controller.task_in_progress:
				# let the game knows the task is completed
				self.ros_node_mgr.send_ispy_cmd(TASK_COMPLETED)	
				self.current_task_index += 1
				if self.current_task_index != 0: 
					action_number = self.current_task_index 
					self.interaction.start_task_end_behavior(action_number)

		#print("State Transition: "+transition_msg.data)

		if transition_msg.data in gs.Triggers.triggers:
			#time.sleep(.1)

			if self.FSM.state != gs.EXPLORATION_MODE and self.FSM.state != gs.WORD_DISPLAY: # if the game is still in explore mode
				self.interaction.react(transition_msg.data,self.origText) # the robot reacts


			if transition_msg.data == gs.Triggers.TOPLEFT_BUTTON_PRESSED:
				self.iSpyDataTracking.start_stopwatch()
				self.interaction.turn_start_time = datetime.datetime.now()
				self._run_game_task()

			elif transition_msg.data == gs.Triggers.CONNECT_BUTTON_PRESSED:
				self.ros_node_mgr.send_ispy_cmd(34, self.session_number) #SET_GAME_SCNE = 34
				print("CONNECT_BUTTON_PRESSED : "+self.session_number)
				self.ros_node_mgr.send_robot_cmd(RobotBehaviors.ROBOT_HAPPY_DANCE)
				
			elif transition_msg.data == gs.Triggers.HINT_BUTTON_PRESSED:
				self.interaction.numHintButtonPressedForTask += 1


			elif transition_msg.data == gs.Triggers.OBJECT_CLICKED:
				pass
		
				
			elif transition_msg.data == gs.Triggers.NONTARGET_OBJECT_COLLECTED or transition_msg.data == gs.Triggers.TARGET_OBJECT_COLLECTED:
				self._reach_max_task_time()
				check_task_completion()
				self.interaction.turn_taking()
				
			# elif transition_msg.data == gs.Triggers.TARGET_OBJECT_COLLECTED:
			# 	self._reach_max_task_time()
			# 	check_task_completion()
			# 	self.interaction.turn_taking()
				
			elif transition_msg.data == gs.Triggers.PRONUNCIATION_PANEL_CLOSED:
				if self.interaction.state == ris.CHILD_TURN: # when a new turn is child's, then start tracking the child's interaction
					t = threading.Timer(3.0, self.interaction.start_tracking_child_interaction).start() # checking for timeout 
			
			elif transition_msg.data == gs.Triggers.SCREEN_MOVED:
				self.interaction.stop_tracking_child_interaction()

			# If the message is in gs.Triggers, then allow the trigger
			if transition_msg.data != gs.Triggers.SCREEN_MOVED:
				self.FSM.start_trigger(transition_msg.data)
				
			

	#################################################################################################

	def on_ispy_log_received(self, log_msg):
		
		# If the the message was "messageReceived", that means that the publishing loop can stop
		if log_msg.data == "messageReceived":
			self.ros_node_mgr.message_received = True


	#################################################################################################
	def on_ispy_action_received(self, ispy_action_msg):
		"""
		Rospy callback for when we get ispy action from the unity game over ROS
		"""


		if self.FSM.state == gs.EXPLORATION_MODE or self.FSM.state == gs.WORD_DISPLAY: 
		# if the game is still in explore mode
			return


		time.sleep(0.1) # wait for on_ispy_state_info_received() to finish and FSM to transition first
		#self.interaction.stop_tracking_child_interaction() # start tracking the elapsed time of child's lack of tablet interaction

		
		self.iSpyDataTracking.on_ispy_action_received(ispy_action_msg)

		def speakingStage(stage):
			if stage == "speakingStart":
				self.recorder.start_recording(self.origText, RECORD_TIME_MS, self.interaction.state) #TODO: Update 'test' to actual word
			elif stage == "speakingEnd":
				self.recorder.stop_recording()
	
		
		def msg_evaluator(ispy_action_msg):
			"""Calls the respective functions for each part of the action msg
			"""

			#Removes object position from ispy_action_msg
			if ispy_action_msg.clickedObjectName != "":
				object_name = ""
				for letter in ispy_action_msg.clickedObjectName:
					if letter == "-":
						break
					object_name += letter
				self.origText = object_name


			#Initializes a new audio recorder object if one hasn't been created
			if self.recorder == None: self.recorder = AudioRecorder()
	
			speakingStage(ispy_action_msg.speakingStage)

		print("\n")
		# Evaluates the action message
		msg_evaluator(ispy_action_msg)



		self._speechace_analysis()


		if self.interaction.state == ris.CHILD_TURN or self.interaction.state == ris.ROBOT_TURN+'_'+ris.CHILD_HELP:
			pass
			# if self.FSM.state == gs.MISSION_MODE: 
			# 	threading.Timer(0.5, self.interaction.start_tracking_child_interaction).start() # start tracking the elapsed time of child's lack of tablet interaction

		self.isDragging = ispy_action_msg.isDragging
		self.pointerClick = ispy_action_msg.pointerClick
		self.onPinch = ispy_action_msg.onPinch
		self.isScalingUp = ispy_action_msg.isScalingUp
		self.isScalingDown = ispy_action_msg.isScalingDown

		self.interaction._ros_publish_data("","", True)
		
			
	def _speechace_analysis(self):
		'''
		speech ace analysis
		'''
		print("+++++speech analay: {}".format(self.origText))
		# If given a word to evaluate and done recording send the information to speechace
		if self.origText and self.recorder.has_recorded % 2 == 0 and self.recorder.has_recorded != 0:
			# If you couldn't find the android audio topic, automatically pass
			# instead of using the last audio recording

			print("clicked object is: "+self.origText)

			self.correct_obj = self.task_controller.isTarget(self.origText) 
			
			
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

			
			#print(results_params)
			self.ros_node_mgr.send_ispy_cmd(SEND_PRONOUNCIATION_ACCURACY_TO_UNITY, results_params)
			self.recorder.has_recorded = 0


	
	
	def _run_game_task(self):
		# When entering mission mode from exploration mode, get a random task
		# and send it to Unity

		if self.task_controller.task_in_progress == False:
		
			task = self.task_controller.get_next_task()

			# If there are no more available quests, you won the game
			if task == None:
			
				self.ros_node_mgr.send_ispy_cmd(GAME_FINISHED)
				self.interaction.child_states.done()
			else:
				self.ros_node_mgr.send_ispy_cmd(SEND_TASKS_TO_UNITY, task)
				self.interaction.reset_turn_taking()
				self.interaction.get_robot_general_response()
				
				t = threading.Timer(3.0,self.interaction.start_tracking_child_interaction).start()
				threading.Timer(10.0, self.interaction.on_child_max_elapsed_time).start()

	
	


	

