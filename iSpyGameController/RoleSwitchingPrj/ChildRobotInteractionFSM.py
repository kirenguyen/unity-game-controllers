import time

from transitions import Machine
from .AgentModel import AgentModel

from ..RobotBehaviorList.RobotBehaviorList import RobotBehaviors
from ..RobotBehaviorList.RobotBehaviorList import RobotRoles
from ..RobotBehaviorList.RobotBehaviorList import RobotRolesBehaviorsMap
from ..RobotBehaviorList.RobotBehaviorList import RobotActionSequence as ras
from .ChildStates import ChildStates

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


class ChildRobotInteractionFSM:
		'''
		child robot interaction FSM for robot's role switching project
		It communicates with iSpyGameFSM, reinforcemnet learning agent model of the robot, robot's behaviors
		and ChildStates
		'''

		def __init__(self,ros_node_mgr,task_controller):
			
			self.states = [ ris.ROBOT_TURN, ris.CHILD_TURN ]
			self.transitions = [
				{'trigger': ris.Triggers.CHILD_TURN_DONE, 'source': ris.CHILD_TURN, 'dest': ris.ROBOT_TURN },
				{'trigger': ris.Triggers.ROBOT_TURN_DONE, 'source': ris.ROBOT_TURN, 'dest': ris.CHILD_TURN},
			]

			self.state_machine = Machine(self, states=self.states, transitions=self.transitions,
									 initial=ris.CHILD_TURN)

			self.ros_node_mgr = ros_node_mgr

			self.task_controller = task_controller

			self.child_states = ChildStates()

			self.agent_model = AgentModel()

			self.role_behavior_mapping = RobotRolesBehaviorsMap()

			# robot's physical actions
			self.physical_actions ={}
			# robot's virtual actions on the tablet
			self.virtual_action = ""

			self.robot_response = self.role_behavior_mapping.get_robot_general_responses()

			self.role = ""

			self.robot_clickedObj=""

			

		def turn_taking(self):
			# check whether it is robot's turn or child's turn in the game play
			if self.state == ris.ROBOT_TURN:
				# stop tracking the previous turn's rewards
				rewards = self.child_states.stop_tracking_rewards(self.state)

				# then, next turn is child's 
				getattr(self, ris.Triggers.ROBOT_TURN_DONE)() # convert the variabel to string, which is the name of the called function
				self.child_states.start_tracking_rewards(self.state)

			elif self.state == ris.CHILD_TURN:

				rewards = self.child_states.stop_tracking_rewards(self.state)
				self.agent_model.onRewardsReceived(rewards) # update the RL model 
				# then, next turn is robot's
				getattr(self, ris.Triggers.CHILD_TURN_DONE)()
				# start tracking rewards (engagement) for the robot's role during child's turn
				self.child_states.start_tracking_rewards(self.state)
	
			print("==========TURN TAKING===============: "+self.state)

			
			# update the number of available objects for child's learning states
			self.child_states.set_num_available_objs(self.task_controller.get_num_available_target_objs())
			# robot's response 
			self.get_turn_taking_actions()

		
		def react(self,gameStateTrigger, command=0):
			'''
			react to ispy game state change
			'''
			if gameStateTrigger == gs.Triggers.TARGET_OBJECT_COLLECTED:
				self._perform_robot_physical_action(self.physical_actions[ras.PRONOUNCE_CORRECT])
				self._perform_robot_physical_action(self.physical_actions[ras.TURN_FINISHED])
				if self.state == ris.CHILD_TURN:
					# the child finds the correct object
					self.child_states.update_child_turn_result(True)

			elif gameStateTrigger  == gs.Triggers.OBJECT_CLICKED:
				if self.state == ris.ROBOT_TURN:
					if random.random() < 0.75:
						self._perform_robot_physical_action(self.physical_actions[ras.OBJECT_FOUND])
					else:
						self._perform_robot_physical_action(self.physical_actions[ras.OBJECT_CLICKED])
				if self.state == ris.CHILD_TURN:
					# Check to see if the object is incorrect
					# If correct, given assertion 
					if command == 0:
						self._perform_robot_physical_action(self.physical_actions[ras.OBJECT_FOUND])
					# If incorrect, show doubt 
					else:
						self._perform_robot_physical_action(self.physical_actions[ras.OBJECT_CLICKED])
				
			elif gameStateTrigger  == gs.Triggers.SAY_BUTTON_PRESSED:
				self._perform_robot_physical_action(self.physical_actions[ras.OBJECT_PRONOUNCED])

				# Check to see if it is incorrect
				# If correct, will be within target_object_collected -> Happy emotion
				# If incorrect, not target_object -> Sad emotion 
				if command == 1 and random.random() < 0.75:
					time.sleep(2)
					self._perform_robot_physical_action(self.physical_actions[ras.WRONG_OBJECT_FAIL])

			elif gameStateTrigger  == gs.Triggers.PRONUNCIATION_PANEL_CLOSED:
				if self.state == ris.CHILD_TURN:
					# the child finds the correct object
					self.child_states.update_child_turn_result(False)

			elif gameStateTrigger == gs.Triggers.SCREEN_MOVED:
				self._perform_robot_physical_action(self.physical_actions[ras.SCREEN_MOVED])




		def get_turn_taking_actions(self):
			'''
			check the current interaction FSM to decide whether the robot should respond
			then, use agent model to decide how the robot should respond if it needs to respond
			'''
		
			actions = self._get_behaviors()

			physical_actions = actions['physical']
			virtual_action = actions['virtual']

			if physical_actions:
				self.physical_actions = physical_actions
				self._perform_robot_physical_action(self.physical_actions[ras.TURN_STARTED])
			if virtual_action: 
				self._perform_robot_virtual_action(virtual_action)



		def _get_behaviors(self):
			'''
			Get corresponding virtual and physical actions for a given input robot's role
			'''
			robot_turn = False
			if self.state == ris.ROBOT_TURN:
				self.role = self.agent_model.get_next_robot_role()
				robot_turn = True
				print("Robot's Role: ")
				print(self.role)
			return self.role_behavior_mapping.get_actions(self.role,robot_turn)
			

		def _perform_robot_physical_action(self,actions):
			'''
			send the physical action message via ROS to the robot
			'''

			print("perform robot physical action runs..")
			for action in actions:
				# if the action is to pronounce a word, then specify a word to pronounce
				input_data = ""
				
				
				if action == RobotBehaviors.ROBOT_SAY_WORD:
					if not self.robot_clickedObj:
						input_data = "cat"
					input_data = self.robot_clickedObj
				elif action == RobotBehaviors.BASED_ON_PROMPTS_SPEECH:
					input_data = self.task_controller.get_vocab_word()

				if action in RobotBehaviors.OPTIONAL_ACTIONS:
					print("========== action in robot behaviors optional actions")
					# if the action is in this array, then randomly decide whether to execute the action or not
					print(random.uniform(0, 1))
					if random.uniform(0, 1) >= 0.3:
						print("========== action executed...")
						self.ros_node_mgr.send_robot_cmd(action,input_data)
						time.sleep(1)
					else:
						print("========not executed")
				else:
					self.ros_node_mgr.send_robot_cmd(action,input_data)
					time.sleep(1)

		def _perform_robot_virtual_action(self,action):
			'''
			send the virtual action message via ROS to the tablet 
			'''
			print("!!!!!virtual action!!!")
			print(action)
			print("===========")

			if action == RobotBehaviors.VIRTUALLY_CLICK_CORRECT_OBJ:
				self.robot_clickedObj = self.task_controller.get_obj_for_robot(True)
				print("true object: "+self.robot_clickedObj)
			elif action == RobotBehaviors.VIRTUALLY_CLICK_WRONG_OBJ:
				self.robot_clickedObj = self.task_controller.get_obj_for_robot(False)
				print("false object: "+self.robot_clickedObj)
			

			self.ros_node_mgr.send_ispy_cmd(iSpyCommand.ROBOT_VIRTUAL_ACTIONS,{"robot_action":action,"clicked_object":self.robot_clickedObj})
			

		
