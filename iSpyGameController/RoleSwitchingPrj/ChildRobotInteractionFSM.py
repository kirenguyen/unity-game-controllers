import time
import json

#from transitions import Machine
from transitions.extensions import HierarchicalMachine as Machine
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

ROOT_TEGA_SPEECH_FOLDER = 'roleswitching18/'



class ChildRobotInteractionFSM:
		'''
		child robot interaction FSM for robot's role switching project
		It communicates with iSpyGameFSM, reinforcemnet learning agent model of the robot, robot's behaviors
		and ChildStates
		'''

		def __init__(self,ros_node_mgr,task_controller):
			# use hierachical FSM here
			self.states = [ ris.ROBOT_TURN, {'name':ris.CHILD_TURN,'children':[ris.ROBOT_HELP]} ]
			self.transitions = [
				{'trigger': ris.Triggers.CHILD_TURN_DONE, 'source': ris.CHILD_TURN, 'dest': ris.ROBOT_TURN },
				{'trigger': ris.Triggers.ROBOT_TURN_DONE, 'source': ris.ROBOT_TURN, 'dest': ris.CHILD_TURN},
				{'trigger': ris.Triggers.ROBOT_HELP_TRIGGER,'source':ris.CHILD_TURN,'dest':ris.CHILD_TURN+'_'+ris.ROBOT_HELP}
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

			self.ros_node_mgr.start_tega_state_listener(self.on_tega_state_received)

			self.ros_node_mgr.start_tega_asr(self.on_tega_new_asr_result)

			self.tega_is_playing_sound = False


			# load tega speech json file
			# parse tega_speech.json
			tega_speech_file = open("iSpyGameController/res/tega_speech.json")
			self.tega_speech_dict = json.loads(tega_speech_file.read())



		def on_no_ispy_action_alert(self,attempt):
			'''
			callback function when no ispy action within a time period is detected
			when the child is not interacting with the tablet, the robot will try to encourage the child
			called by iSpyDataTracking 
			attempt: first alert, second alert
			'''	
			
			if self.state != ris.CHILD_TURN: return
			print("!!!!!game controller ....no ispy action "+str(attempt))
			# the robot verbally encourages the child
			path=ROOT_TEGA_SPEECH_FOLDER + 'general/speech/'

			speech_file = random.choice([ path+i+".wav" for i in self.tega_speech_dict["general/speech"] if 'no_ispy_action_alert'+str(attempt)+'_response' in i])
			self.ros_node_mgr.send_robot_cmd(RobotBehaviors.ROBOT_CUSTOM_SPEECH,speech_file)

			if attempt == 2: # first alert
				# robot intervenes (robot spies an object for the child)
				# select an object and then find it for the child
				self._perform_robot_virtual_action(RobotBehaviors.VIRTUALLY_CLICK_CORRECT_OBJ)
				getattr(self, ROBOT_HELP_TRIGGER)() 
				
		def on_tega_state_received(self,data):
			## call back function when a tega state message is received from Tega phone.
			# this function should be called multiple times per second
			# robot is in motion or playing sound or not
			self.tega_is_playing_sound = data.is_playing_sound

        		
		def on_tega_new_asr_result(self,data):
			# callback function when new asr results are received from Tega
			print("new asr results...")
			
			self.asr_input = unicode(data.transcription.lower().strip(), "utf-8")
			self.asr_input_confidence = data.confidence

			print("\n#####Tega Heard: {0}\n".format(self.asr_input))

        	# if all(word in SETTINGS.STOPWORDS_SINGLE for word in self.asr_input):
        	# 	print "speech filtered per STOPWORDS_SINGLE"
        	# 	return
        	# elif any(word in self.asr_input for word in SETTINGS.STOPWORDS):
        	# 	print "speech filtered per STOPWORDS"
        	# 	return



		def reset_turn_taking(self):

			self.state = ris.CHILD_TURN
			
		def turn_taking(self):
			
			if self.task_controller.task_in_progress:
				# check whether it is robot's turn or child's turn in the game play
				if ris.ROBOT_TURN in self.state:
					# stop tracking the previous turn's rewards
					rewards = self.child_states.stop_tracking_rewards(self.state)

					# then, next turn is child's 
					getattr(self, ris.Triggers.ROBOT_TURN_DONE)() # convert the variabel to string, which is the name of the called function
					self.child_states.start_tracking_rewards(self.state)

				elif ris.CHILD_TURN in self.state:

					rewards = self.child_states.stop_tracking_rewards(self.state)
					self.agent_model.onRewardsReceived(rewards) # update the RL model 
					# then, next turn is robot's
					getattr(self, ris.Triggers.CHILD_TURN_DONE)()
					# start tracking rewards (engagement) for the robot's role during child's turn
					self.child_states.start_tracking_rewards(self.state)
	
				print("==========TURN TAKING===============: "+self.state)
				# send the turn info (child/robot) to tablet via ROS

				self.ros_node_mgr.send_ispy_cmd(iSpyCommand.WHOSE_TURN, {"whose_turn":self.state})

			
				# update the number of available objects for child's learning states
				self.child_states.set_num_available_objs(self.task_controller.get_num_available_target_objs())
				# robot's response 
				self.get_turn_taking_actions()

		
		def react(self,gameStateTrigger, command=0):
			'''
			react to ispy game state change
			'''
			if gameStateTrigger == gs.Triggers.TARGET_OBJECT_COLLECTED:
				self._perform_robot_physical_action(ras.PRONOUNCE_CORRECT)
				self._perform_robot_physical_action(ras.TURN_FINISHED)
				if self.state == ris.CHILD_TURN:
					# the child finds the correct object
					self.child_states.update_child_turn_result(True)

			elif gameStateTrigger  == gs.Triggers.OBJECT_CLICKED:
				if self.state == ris.ROBOT_TURN or self.state == ris.CHILD_TURN+'_'+ROBOT_HELP:
					if random.random() < 0.75:
						print("===== object found")
						self._perform_robot_physical_action(ras.OBJECT_FOUND)
					else:
						print("===== object clicked")
						self._perform_robot_physical_action(ras.OBJECT_CLICKED)

					self._wait_until() # wait until robot is done speaking. then click and pronounce the word
					self._perform_robot_virtual_action(RobotBehaviors.VIRTUALLY_CLICK_SAY_BUTTON)

				if self.state == ris.CHILD_TURN:
					# Check to see if the object is incorrect
					# If correct, given assertion 
					if command == 0:
						self._perform_robot_physical_action(ras.OBJECT_FOUND)
					# If incorrect, show doubt 
					else:
						self._perform_robot_physical_action(ras.OBJECT_CLICKED)
				
			elif gameStateTrigger  == gs.Triggers.SAY_BUTTON_PRESSED:
				self._perform_robot_physical_action(ras.OBJECT_PRONOUNCED)

				# Check to see if it is incorrect
				# If correct, will be within target_object_collected -> Happy emotion
				# If incorrect, not target_object -> Sad emotion 
				if command == 1 and random.random() < 0.75:
					time.sleep(2)
					self._perform_robot_physical_action(ras.WRONG_OBJECT_FAIL)

			elif gameStateTrigger  == gs.Triggers.PRONUNCIATION_PANEL_CLOSED:
				if self.state == ris.CHILD_TURN:
					# the child finds the correct object
					self.child_states.update_child_turn_result(False)

			elif gameStateTrigger == gs.Triggers.SCREEN_MOVED:
				self._perform_robot_physical_action(ras.SCREEN_MOVED)

				if self.state == ris.ROBOT_TURN:
					# wait until all physical actions in SCREEN_MOVED is done, then perform robot's virtual move for finding an object
					time.sleep(1)
					self._wait_until()
					self._perform_robot_virtual_action(self.virtual_action)

		def _wait_until(self):
			'''
			wait until tega is done speaking
			'''
			while self.tega_is_playing_sound==True:
					continue
					if self.tega_is_playing_sound==False:
						break

		def get_turn_taking_actions(self):
			'''
			check the current interaction FSM to decide whether the robot should respond
			then, use agent model to decide how the robot should respond if it needs to respond
			'''
			
			actions = self._get_behaviors()

			physical_actions = actions['physical']
			self.virtual_action = actions['virtual']

			

			if physical_actions:
				self.physical_actions = physical_actions
				self._perform_robot_physical_action(ras.TURN_STARTED)
				# wait until robot's actions for TURN_STARTED to complete. the robot first explores the scene
				time.sleep(3)
				if self.state == ris.ROBOT_TURN: self._perform_robot_virtual_action(RobotBehaviors.VIRTUALLY_EXPLORE)


		def get_robot_general_response(self):
			
			actions = self.role_behavior_mapping.get_actions("",False)
			physical_actions = actions['physical']
			virtual_action = actions['virtual']

			if physical_actions:
				self.physical_actions = physical_actions
				self._perform_robot_physical_action(ras.TURN_STARTED)
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
				
			return self.role_behavior_mapping.get_actions(self.role,robot_turn)
			
		def _get_tega_speech(self,action_type):
			'''
			get tega speech audios based on current action type, robot's role
			'''

			if isinstance(self.role,str):
				return []
			else:
				speech_audio_path = '/'.join(['general',self.role.name.lower(), self.state.replace('TURN','')])
				try:
					all_audio_arrs= self.tega_speech_dict[speech_audio_path]
					speech_audios = [ ROOT_TEGA_SPEECH_FOLDER+speech_audio_path+"/"+i+'.wav' for i in all_audio_arrs if action_type in i]
					
					return [random.choice(speech_audios)]
				except:
					print("ERROR: Couldn't find audio files in tega speech json files")
					return []
		
				

		def _perform_robot_physical_action(self,action_type):
			'''
			send the physical action message via ROS to the robot
			'''

			actions = self.physical_actions[action_type]

			speech_actions = self._get_tega_speech(action_type)

			# send physical moition commands
			for action in actions:
				# if the action is to pronounce a word, then specify a word to pronounce

				while self.tega_is_playing_sound==True:
					continue
					if self.tega_is_playing_sound==False:
						break

				input_data = ""
				
				if action == RobotBehaviors.ROBOT_SAY_WORD:
					if not self.robot_clickedObj:
						input_data = "cat"
					input_data = self.robot_clickedObj
				elif action == RobotBehaviors.BASED_ON_PROMPTS_SPEECH:
					input_data = self.task_controller.get_vocab_word()

				if action in RobotBehaviors.OPTIONAL_ACTIONS:
					
					# if the action is in this array, then randomly decide whether to execute the action or not
					if random.uniform(0, 1) >= 0.25:
						self.ros_node_mgr.send_robot_cmd(action,input_data)
						time.sleep(1)
					
				else:
					self.ros_node_mgr.send_robot_cmd(action,input_data)
					time.sleep(1)

			# send speech commands
			for speech_file in speech_actions:
				while self.tega_is_playing_sound==True:
					continue
					if self.tega_is_playing_sound==False:
						break
				self.ros_node_mgr.send_robot_cmd(RobotBehaviors.ROBOT_CUSTOM_SPEECH,speech_file)

		def _perform_robot_virtual_action(self,action):
			'''
			send the virtual action message via ROS to the tablet 
			'''

			if action == RobotBehaviors.VIRTUALLY_CLICK_CORRECT_OBJ:
				self.robot_clickedObj = self.task_controller.get_obj_for_robot(True)
				
			elif action == RobotBehaviors.VIRTUALLY_CLICK_WRONG_OBJ:
				self.robot_clickedObj = self.task_controller.get_obj_for_robot(False)
				
			
			self.ros_node_mgr.send_ispy_cmd(iSpyCommand.ROBOT_VIRTUAL_ACTIONS,{"robot_action":action,"clicked_object":self.robot_clickedObj})
			

		
