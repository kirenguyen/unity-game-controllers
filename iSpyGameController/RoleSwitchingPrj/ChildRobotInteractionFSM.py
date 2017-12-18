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
			# use hierachical FSM here. The python package can be found here: https://github.com/pytransitions/transitions
			self.states = [ {'name': ris.ROBOT_TURN, 'children':[ris.QUESTION_ASKING,ris.CHILD_HELP]}, {'name':ris.CHILD_TURN,'children':[ ris.QUESTION_ASKING ,ris.ROBOT_HELP]} ]
			self.transitions = [
				{'trigger': ris.Triggers.CHILD_TURN_DONE, 'source': ris.CHILD_TURN, 'dest': ris.ROBOT_TURN },
				{'trigger': ris.Triggers.ROBOT_TURN_DONE, 'source': ris.ROBOT_TURN, 'dest': ris.CHILD_TURN},
				{'trigger': ris.Triggers.ROBOT_HELP_TRIGGER,'source':ris.CHILD_TURN,'dest': ris.CHILD_TURN+'_'+ris.ROBOT_HELP},
				{'trigger':	ris.Triggers.CHILD_HELP_TRIGGER, 'source': [ris.ROBOT_TURN, ris.ROBOT_TURN + '_' + ris.QUESTION_ASKING], 'dest': ris.ROBOT_TURN+'_'+ris.CHILD_HELP},
				{'trigger':	ris.Triggers.ROBOT_QUESTION, 'source': ris.ROBOT_TURN, 'dest': ris.ROBOT_TURN+'_'+ris.QUESTION_ASKING}
		
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

			#self.robot_response = self.role_behavior_mapping.get_robot_general_responses()

			self.role = ""

			self.robot_clickedObj=""

			self.ros_node_mgr.start_tega_state_listener(self.on_tega_state_received)

			self.ros_node_mgr.start_tega_asr(self.on_tega_new_asr_result)

			self.tega_is_playing_sound = False


			# load tega speech json file
			# parse tega_speech.json
			tega_speech_file = open("iSpyGameController/res/tega_speech.json")
			self.tega_speech_dict = json.loads(tega_speech_file.read())

			question_answer_file = open("iSpyGameController/res/question_answer.json")
			self.question_answer_dict = json.loads(question_answer_file.read())


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
			
				action = RobotBehaviors.ROBOT_OFFER_HELP

				question_query_path = '/'.join([self.role.name.lower(), self.state.replace('TURN',''), action])
				print("question query!!")
				print(question_query_path)

				question_query = self.question_answer_dict[question_query_path]
				question_speech_file = ROOT_TEGA_SPEECH_FOLDER + "questions/" +question_query['question'][0]+".wav"
					
				print(question_speech_file)
				self.ros_node_mgr.send_robot_cmd(RobotBehaviors.ROBOT_CUSTOM_SPEECH, question_speech_file)
				
				getattr(self, ris.Triggers.ROBOT_HELP_TRIGGER)()


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

			print("turn taking...")
			
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



		
		def react(self,gameStateTrigger, clicked_right_obj=False):
			'''
			react to ispy game state change
			'''
			if gameStateTrigger == gs.Triggers.TARGET_OBJECT_COLLECTED:
				pass

			elif gameStateTrigger  == gs.Triggers.OBJECT_CLICKED:
				if self.state == ris.ROBOT_TURN or self.state == ris.CHILD_TURN+'_'+ris.ROBOT_HELP:
					#if random.random() < 0.75:
						
					self._perform_robot_physical_action(ras.OBJECT_FOUND)
					# else:
						
					# 	self._perform_robot_physical_action(ras.OBJECT_CLICKED)

					self._wait_until() # wait until robot is done speaking. then click and pronounce the word
					self._perform_robot_virtual_action(RobotBehaviors.VIRTUALLY_CLICK_SAY_BUTTON)

				if (self.state == ris.CHILD_TURN or self.state == ris.ROBOT_TURN+'_'+ris.CHILD_HELP) and self.role == RobotRoles.EXPERT:
					# Check to see if the object is incorrect
					# If correct, given assertion 
					if clicked_right_obj == True:
						self._perform_robot_physical_action(ras.OBJECT_FOUND)
					# If incorrect, show doubt 
					else:
						self._perform_robot_physical_action(ras.OBJECT_CLICKED)
				
			elif gameStateTrigger  == gs.Triggers.SAY_BUTTON_PRESSED:
				self._perform_robot_physical_action(ras.OBJECT_PRONOUNCED)

				

			elif gameStateTrigger  == gs.Triggers.PRONUNCIATION_PANEL_CLOSED:
				# Check to see if it is incorrect
				# If correct, will be within target_object_collected -> Happy emotion
				# If incorrect, not target_object -> Sad emotion 
				if clicked_right_obj == False:
					self._perform_robot_physical_action(ras.WRONG_OBJECT_FAIL)
					self._perform_robot_physical_action(ras.TURN_FINISHED)
					if self.state == ris.CHILD_TURN or self.state == ris.ROBOT_TURN+'_'+ris.CHILD_HELP:
						# the child finds the correct object
						self.child_states.update_child_turn_result(False)

				elif clicked_right_obj == True:
					self._perform_robot_physical_action(ras.PRONOUNCE_CORRECT)
					self._perform_robot_physical_action(ras.TURN_FINISHED)
					if self.state == ris.CHILD_TURN or self.state == ris.ROBOT_TURN+'_'+ris.CHILD_HELP:
						# the child finds the correct object
						self.child_states.update_child_turn_result(True)

					

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
			
			if self.state == ris.ROBOT_TURN: self.role = self.agent_model.get_next_robot_role()
				
			physical_actions = self.role_behavior_mapping.get_actions(self.role,self.state,'physical')
			self.virtual_action = self.role_behavior_mapping.get_actions(self.role,self.state,'virtual')
			if self.virtual_action:
				self.virtual_action = self.virtual_action[0]

			if physical_actions:
				self.physical_actions = physical_actions
				self._perform_robot_physical_action(ras.TURN_STARTED)
				# wait until robot's actions for TURN_STARTED to complete. the robot first explores the scene
				time.sleep(3)
				if self.state == ris.ROBOT_TURN: self._perform_robot_virtual_action(RobotBehaviors.VIRTUALLY_EXPLORE)


		def get_robot_general_response(self):
			
			
			physical_actions = self.role_behavior_mapping.get_actions("BACKUP",self.state,'physical')
			virtual_action = self.role_behavior_mapping.get_actions("BACKUP",self.state,'virtual')



			if physical_actions:
				self.physical_actions = physical_actions
				self._perform_robot_physical_action(ras.TURN_STARTED)
			if virtual_action: 
				self._perform_robot_virtual_action(virtual_action[0])
		

		
			
		def _get_tega_speech(self,action_type,speech_type = 'general'):
			'''
			get tega speech audios based on current action type, robot's role
			'''

			if isinstance(self.role,str):
				return []
			else:
				speech_audio_path = '/'.join([speech_type,self.role.name.lower(), self.state.replace('TURN','')])
				print(speech_audio_path)
				print(action_type)
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

			actions = self.physical_actions[action_type]['general']

			speech_actions = self._get_tega_speech(action_type)

			# send physical moition commands
			for action in actions:

				action = self.role_behavior_mapping.get_action_name(action) # get the correct name
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

				
				self.ros_node_mgr.send_robot_cmd(action,input_data)
				time.sleep(1)

			# send speech commands
			for speech_file in speech_actions:
				while self.tega_is_playing_sound==True:
					continue
					if self.tega_is_playing_sound==False:
						break
				self.ros_node_mgr.send_robot_cmd(RobotBehaviors.ROBOT_CUSTOM_SPEECH,speech_file)

			self._perform_question_asking(action_type)


		def _perform_question_asking(self,action_type):
			'''
			robot asks a question and waits for the child to answer
			'''
			if 'question' in self.physical_actions[action_type].keys(): #check whether 'question' key exists. If so, perform it
				
				actions = self.physical_actions[action_type]['question']
				for action in actions:
					ran = random.random() 
					print("random: "+str(ran))
					if (action == RobotBehaviors.ROBOT_ASK_HELP and ran <0.2) or action != RobotBehaviors.ROBOT_ASK_HELP:
						action = self.role_behavior_mapping.get_action_name(action)

						question_query_path = '/'.join([self.role.name.lower(), self.state.replace('TURN',''), action])
						print("question query!!")
						print(question_query_path)

						question_query = self.question_answer_dict[question_query_path]
						question_speech_file = ROOT_TEGA_SPEECH_FOLDER + "questions/" +question_query['question'][0]+".wav"
					
						print(question_speech_file)
						self.ros_node_mgr.send_robot_cmd(RobotBehaviors.ROBOT_CUSTOM_SPEECH, question_speech_file)
						print("action: "+action)
						
						if action == RobotBehaviors.ROBOT_ASK_HELP:
							# robot asks the child to help find an object
							# send a ros command to enable child's interaction with the tablet
							self.ros_node_mgr.send_ispy_cmd(iSpyCommand.ROBOT_VIRTUAL_ACTIONS,{"robot_action":RobotBehaviors.ROBOT_ASK_HELP,"clicked_object":""})
							getattr(self, ris.Triggers.CHILD_HELP_TRIGGER)()
							print("current state: "+self.state)
			

		def _perform_robot_virtual_action(self,action):
			'''
			send the virtual action message via ROS to the tablet 
			'''

			action = self.role_behavior_mapping.get_action_name(action) # get the correct name

			if action == RobotBehaviors.VIRTUALLY_CLICK_CORRECT_OBJ:
				self.robot_clickedObj = self.task_controller.get_obj_for_robot(True)
				
			elif action == RobotBehaviors.VIRTUALLY_CLICK_WRONG_OBJ:
				self.robot_clickedObj = self.task_controller.get_obj_for_robot(False)
			
			
			self.ros_node_mgr.send_ispy_cmd(iSpyCommand.ROBOT_VIRTUAL_ACTIONS,{"robot_action":action,"clicked_object":self.robot_clickedObj})
			

		
