
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


from .AgentModel import AgentModel
from .ROSNodeMgr import ROSNodeMgr
from .StudentModel import StudentModel
from .RobotBehaviorList import RobotBehaviors
from .RobotBehaviorList import RobotRoles
from .RobotBehaviorList import RobotRolesBehaviorsMap

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
VALID_ISPY_COMMANDS = [RESET, SHOW_PRONOUNCIATION_PANEL, SHOW_PRONOUNCIATION_PANEL, SEND_PRONOUNCIATION_ACCURACY_TO_UNITY, SEND_TASKS_TO_UNITY, GAME_FINISHED]






class iSpyGameFSM: # pylint: disable=no-member
	"""
	Receives and sends out ROS messages.
	"""


	class ChildRobotInteractionFSM:
		def __init__(self,ros_node_mgr):
			##MAGGIE TODO: after add ROSNodeMrg.py, pass it to this class from its constructor

			self.states = [ ris.ROBOT_TURN, ris.CHILD_TURN ]
			self.transitions = [
				{'trigger': ris.Triggers.CHILD_TURN_DONE, 'source': ris.CHILD_TURN, 'dest': ris.ROBOT_TURN },
				{'trigger': ris.Triggers.ROBOT_TURN_DONE, 'source': ris.ROBOT_TURN, 'dest': ris.CHILD_TURN},
			]
			self.state_machine = Machine(self, states=self.states, transitions=self.transitions,
									 initial=ris.ROBOT_TURN)

			self.ros_node_mgr = ros_node_mgr

			self.agent_model = AgentModel()

			self.role_behavior_mapping = RobotRolesBehaviorsMap()


		def test(self):
			print("current state: "+self.state)
			self.Robot_Turn_Done()
			print("after child turn done")
			print("current state: "+self.state)

		def respond(self):
			'''
			check the current interaction FSM to decide whether the robot should respond
			then, use agent model to decide how the robot should respond if it needs to respond
			'''
			physical_action = ""
			virtual_action = ""

			if self.state == ris.ROBOT_TURN:
				# choose an action for robot
				robot_role = self.get_role()
				actions = self.get_behaviors(robot_role)
				physical_action = actions['physical'] 
				virtual_action = actions['virtual']

			elif self.state == ris.CHILD_TURN:
				# no need to respond at this point 
				pass

			if physical_action:
				self.perform_robot_physical_action(physical_action)
			if virtual_action:
				self.perform_robot_virtual_action(virtual_action)

		def get_behaviors(self,role):
			'''
			Get corresponding virtual and physical actions for a given input robot's role
			'''
			return self.role_behavior_mapping.get_actions(role)
			

		def perform_robot_physical_action(self,action):
			'''
			send the physical action message via ROS to the robot
			'''
			##MAGGIE TODO: send the physical action to Jibo via ROSNodeMgr
			#self.ros_node_mgr.send_robot_cmd(Rob)
			pass

		def perform_robot_virtual_action(self,action):
			'''
			send the virtual action message via ROS to the tablet 
			'''
			##MAGGIE TODO: send the virtual action to ispy game here via ROSNodeMgr
			#self.ros_node_mgr.send_ispy_cmd(iSpyCommand.ROBOT_VIRTUAL_ACTIONS,action)
			

		def get_role(self):
			'''
			get the most approriate role from agent model
			'''
			##MAGGIE TODO: at this point, just make the role equal to robot_expert_role
			role = self.agent_model.get_next_robot_role()
			return role

	def __init__(self):

		self.ros_node_mgr = ROSNodeMgr()
		self.ros_node_mgr.init_ros_node()

		self.interaction = self.ChildRobotInteractionFSM(self.ros_node_mgr)

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
							self.ros_node_mgr.send_ispy_cmd(GAME_FINISHED)
						else:
							self.ros_node_mgr.send_ispy_cmd(SEND_TASKS_TO_UNITY, task)

					time_in_explore_mode = time.time() - self.explore_time_start
					print("Time spent in explore mode is %s seconds" %time_in_explore_mode)

					# Check how the robot's should respond (physically and virtually through ispy game)
					self.interaction.respond()

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
			self.ros_node_mgr.message_received = True


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

			self.ros_node_mgr.send_ispy_cmd(SEND_PRONOUNCIATION_ACCURACY_TO_UNITY, results_params)
			self.recorder.has_recorded = 0
			

	


# if __name__ == '__main__':
# 	control = iSpyGameFSM()
# 	print ("FSM Started!")
# 	#thread.start_new_thread(control.start_ispy_transition_listener, ())
# 	thread.start_new_thread(control.ros_node_mgr.start_log_listener, ())
# 	#control.start_ispy_action_listener()

