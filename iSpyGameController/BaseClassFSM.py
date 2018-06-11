import time
import json

from abc import ABC, abstractmethod

from .RobotBehaviorList.RobotBehaviorList import RobotBehaviors
from .RobotBehaviorList.RobotBehaviorList import RobotRoles
from .RobotBehaviorList.RobotBehaviorList import RobotRolesBehaviorsMap
from .RobotBehaviorList.RobotBehaviorList import RobotActionSequence as ras
from .RoleSwitchingPrj.ChildStates import ChildStates

# from GameUtils import Curriculum
from GameUtils import GlobalSettings
from GameUtils.GlobalSettings import iSpyGameStates as gs
from GameUtils.GlobalSettings import iSpyRobotInteractionStates as ris
from GameUtils.PronunciationUtils.PronunciationUtils import PronunciationUtils

if GlobalSettings.USE_ROS:
	from std_msgs.msg import Header  # standard ROS msg header
	from std_msgs.msg import String
	from unity_game_msgs.msg import iSpyCommand
	from unity_game_msgs.msg import iSpyAction

import random

from unity_game_msgs.msg import iSpyChildRobotInteraction
import threading
from datetime import datetime, timedelta

import os
import timestring
import rospy

#from transitions import Machine
from transitions.extensions import HierarchicalMachine as Machine

class BaseClassFSM:

		@abstractmethod
		def __init__(self,ros_node_mgr,task_controller,game_controller,participant_id,game_round):
			
			self.state_machine = Machine(self, states=self.states, transitions=self.transitions,
									 initial=ris.CHILD_TURN)

			self.ros_node_mgr = ros_node_mgr

			self.task_controller = task_controller

			self.game_controller = game_controller

			# load assigned condition the participant is in
			subj_assign_dict = json.loads(open("iSpyGameController/res/participant_assignment.json").read())
			self.subj_cond = subj_assign_dict[participant_id]

			self.child_states = ChildStates(participant_id,self.subj_cond,task_controller)

			self.role_behavior_mapping = RobotRolesBehaviorsMap(game_round)

			# robot's physical actions
			self.physical_actions ={}
			# robot's virtual actions on the tablet
			self.virtual_action = ""

			self.explore_action = ""

			#self.robot_response = self.role_behavior_mapping.get_robot_general_responses()

			self.role = "novice" #default to novice at the beginning (backup)

			self.robot_clickedObj=""

			self.ros_node_mgr.start_tega_state_listener(self.on_tega_state_received)

			self.ros_node_mgr.start_tega_asr(self.on_tega_new_asr_result)
			
			self.tega_is_playing_sound = False

			self.asr_input = ""

			self.current_task_turn_index = 0 # for the current task, current turn index

			self.curr_robot_action = "NA"

			self.turn_start_time = None

			self.turn_end_time = None

			self.turn_duration = ""

			self.child_click_cancel_num = 0

			self.numHintButtonPressedForTask = 0

			self.continue_robot_help = True

			self.reset_elapsed_time = False;

			self.elapsed = ""

		@abstractmethod
		def check_existence_of_asr_rostopic(self):
			print("Some implementation!")

		@abstractmethod
		def on_child_max_elapsed_time(self):
			print("Some implementation!")

		@abstractmethod
		def on_enter_childTURN(self):
			self.turn_start_time = datetime.now()
			self.turn_end_time = None
			self.turn_duration = ""
			self.current_task_turn_index += 1
			
			self.robot_clickedObj = "" # reset robot's clicked obj
			self.explore_action = ""
			self.virtual_action = ""
			self._ros_publish_data()
			threading.Timer(3.0, self.start_tracking_child_interaction).start()
			threading.Timer(10.0, self.on_child_max_elapsed_time).start()
			self.reset_elapsed_time = False

		@abstractmethod
		def on_enter_robotTURN(self):
			print("Some implementation!")

		@abstractmethod
		def on_enter_childTURN_listenChildSpeechResponse(self):
			print("Some implementation!")

		@abstractmethod
		def on_enter_robotTURN_listenChildSpeechResponse(self):
			print("Some implementation!")
		
		@abstractmethod
		def on_enter_childTURN_robotHelp(self):
			print("Some implementation!")

		@abstractmethod
		def on_enter_robotTURN_childHelp(self):
			print("Some implementation!")

		@abstractmethod
		def listen_child_speech(self):
			print("Some implementation!")

		@abstractmethod
		def start_tracking_child_interaction(self):
			print("Some implementation!")
		
		@abstractmethod
		def stop_tracking_child_interaction(self):
			print("Some implementation!")

		@abstractmethod
		def on_no_ispy_action_alert(self,attempt):
			print("Some implementation!")

		@abstractmethod
		def on_tega_state_received(self,data):
			print("Some implementation!")

		@abstractmethod
		def on_tega_new_asr_result(self,data):
			print("Some implementation!")

		@abstractmethod
		def reset_turn_taking(self):
			print("Some implementation!")

		@abstractmethod
		def turn_taking(self,max_time=False):
			self.ros_node_mgr.send_ispy_cmd(iSpyCommand.WHOSE_TURN, {"whose_turn":self.state})
			# update the number of available objects for child's learning states
			self.child_states.set_num_available_objs(self.task_controller.get_num_available_target_objs())

		@abstractmethod
		def react(self,gameStateTrigger,  clicked_obj_name = ""):
			print("Some implementation!")

		@abstractmethod
		def _robot_virutal_action_wait(self):
			print("Some implementation!")

		@abstractmethod
		def _wait_until(self):
			print("Some implementation!")

		@abstractmethod
		def _wait_until_all_audios_done(self):
			print("Some implementation!")

		@abstractmethod
		def get_turn_taking_actions(self):
			print("Some implementation!")

		@abstractmethod
		def get_robot_general_response(self):
			print("Some implementation!")

		@abstractmethod
		def start_task_end_celebration(self, action_number):
			print("Some implementation!")

		@abstractmethod
		def start_task_end_assessment(self, action_number):
			print("Some implementation!")
		
		@abstractmethod
		def start_task_end_behavior(self, action_number):
			print("Some implementation!")

		@abstractmethod
		def _perform_robot_physical_actions(self,action_type):
			print("Some implementation!")

		@abstractmethod
		def _ros_publish_data(self,action="", v_action = "", ispy_action=False):
			print("Some implementation!")

		@abstractmethod
		def _robot_question_asking(self,question_cmd):
			print("Some implementation!")

		@abstractmethod
		def _perform_robot_virtual_action(self,action):
			print("Some implementation!")

		
