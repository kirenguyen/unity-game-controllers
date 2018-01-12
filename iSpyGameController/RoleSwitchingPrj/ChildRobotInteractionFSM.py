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

from unity_game_msgs.msg import iSpyChildRobotInteraction
import threading


ROOT_TEGA_SPEECH_FOLDER = 'roleswitching18/'


class ChildRobotInteractionFSM:
		'''
		child robot interaction FSM for robot's role switching project
		It communicates with iSpyGameFSM, reinforcemnet learning agent model of the robot, robot's behaviors
		and ChildStates
		'''

		def __init__(self,ros_node_mgr,task_controller):
			# use hierachical FSM here. The python package can be found here: https://github.com/pytransitions/transitions
			self.states = [ {'name': ris.ROBOT_TURN, 'children':[ris.QUESTION_ASKING,ris.LISTEN_CHILD_SPEECH_RESPONSE, ris.PARSE_CHILD_SPEECH_RESPONSE, ris.CHILD_HELP]}, {'name':ris.CHILD_TURN,'children':[ ris.NO_INTERACTION_1, ris.QUESTION_ASKING ,ris.LISTEN_CHILD_SPEECH_RESPONSE, ris.PARSE_CHILD_SPEECH_RESPONSE, ris.ROBOT_HELP]} ]
			self.transitions = [
				{'trigger': ris.Triggers.CHILD_TURN_DONE, 'source': ris.CHILD_TURN, 'dest': ris.ROBOT_TURN },
				{'trigger': ris.Triggers.ROBOT_TURN_DONE, 'source': ris.ROBOT_TURN, 'dest': ris.CHILD_TURN},
				{'trigger': ris.Triggers.ROBOT_HELP_TRIGGER,'source':ris.CHILD_TURN,'dest': ris.CHILD_TURN+'_'+ris.ROBOT_HELP},
				{'trigger':	ris.Triggers.CHILD_HELP_TRIGGER, 'source': [ris.ROBOT_TURN, ris.ROBOT_TURN + '_' + ris.QUESTION_ASKING], 'dest': ris.ROBOT_TURN+'_'+ris.CHILD_HELP},
				

				# enter Q&A activity here
				{'trigger':	ris.Triggers.ROBOT_QUESTION, 'source': ris.ROBOT_TURN, 'dest': ris.ROBOT_TURN+'_'+ris.QUESTION_ASKING },
				{'trigger':	ris.Triggers.ROBOT_QUESTION, 'source': ris.CHILD_TURN, 'dest': ris.CHILD_TURN+'_'+ris.QUESTION_ASKING },
				

				# when the child does not interact with the tablet
				{'trigger': ris.Triggers.NO_INTERACTION_ALERT, 'source': ris.CHILD_TURN, 'dest':ris.CHILD_TURN+"_"+ris.NO_INTERACTION_1 },
				
				# robot question asking activity related trisitions
				{'trigger': ris.Triggers.LISTEN_RESPONSE, 'source': ris.ROBOT_TURN+'_'+ris.QUESTION_ASKING, 'dest': ris.ROBOT_TURN+'_'+ris.LISTEN_CHILD_SPEECH_RESPONSE},
				{'trigger': ris.Triggers.SPEECH_RECEIVED, 'source': ris.ROBOT_TURN+'_'+ris.LISTEN_CHILD_SPEECH_RESPONSE, 'dest': ris.ROBOT_TURN + '_'+ ris.PARSE_CHILD_SPEECH_RESPONSE},
				
				# child's turn: question asking activity
				{'trigger': ris.Triggers.LISTEN_RESPONSE, 'source': ris.CHILD_TURN+'_'+ris.QUESTION_ASKING, 'dest': ris.CHILD_TURN+'_'+ris.LISTEN_CHILD_SPEECH_RESPONSE},
				{'trigger': ris.Triggers.SPEECH_RECEIVED, 'source': ris.CHILD_TURN+'_'+ris.LISTEN_CHILD_SPEECH_RESPONSE, 'dest': ris.CHILD_TURN + '_'+ ris.PARSE_CHILD_SPEECH_RESPONSE},
				
				# when q & a activity is done
				{'trigger': ris.Triggers.QA_FINISHED, 'source':ris.ROBOT_TURN+'_'+ris.PARSE_CHILD_SPEECH_RESPONSE, 'dest':ris.ROBOT_TURN },
				{'trigger': ris.Triggers.QA_FINISHED, 'source':ris.CHILD_TURN+'_'+ris.PARSE_CHILD_SPEECH_RESPONSE, 'dest':ris.CHILD_TURN }
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

			self.check_existence_of_asr_rostopic()

		def check_existence_of_asr_rostopic(self):
			'''
			check whether google asr rostopic exists
			'''
			import rospy
			topics = rospy.get_published_topics()
			self.asr_result_topic = False
			
			if '/asr_result' in [ i[0] for i in topics]:
				print("asr result publisher exists")
				self.asr_result_topic = True
			else:
				print("WARNING: asr result publisher does not exist. Remember to start ros_asr.py")

		


		def on_enter_childTURN_listenChildSpeechResponse(self):
			'''
			callback function. called when the FSM enters "listenChildSpeechResponse" state
			'''
			# start ASR listening mode
			print("on enter child turn listen child speech response")
			self.ros_node_mgr.start_asr_listening()
			time.sleep(5)
			self.ros_node_mgr.stop_asr_listening()
			print("stop asr listening!!!")

			if not self.asr_result_topic: # if the asr result topic publsiher doesn't exist
				# manually call the asr result callback function
				print("manually call tega new asr result")
				self.on_tega_new_asr_result("")


		def on_enter_childTURN_parseChildSpeechResponse(self):
			'''
			callback function. called when the FSM enters "parseChildSpeechResponse" state
			'''
			print("on enter child turn parse child speech response")
			


		def on_enter_robotTURN_listenChildSpeechResponse(self):
			'''
			callback function. called when the FSM enters "listenChildSpeechResponse" state
			'''
			# start ASR listening mode
			print("on enter robot turn listen child speech response")
			self.ros_node_mgr.start_asr_listening()

		def on_enter_robotTURN_parseChildSpeechResponse(self):
			'''
			callback function. called when the FSM enters "parseChildSpeechResponse" state
			'''
			print("on enter robot turn parse child speech response")
			#self.ros_node_mgr.stop_asr_listening()



		def on_no_ispy_action_alert(self,attempt):
			'''
			callback function when no ispy action within a time period is detected
			when the child is not interacting with the tablet, the robot will try to encourage the child
			called by iSpyDataTracking 
			attempt: first alert, second alert
			'''
			# check whether the current state is child_turn_no_interaction_1
			if self.state != ris.CHILD_TURN:
				return

			else:

				getattr(self, ris.Triggers.NO_INTERACTION_ALERT)() # trigger the FSM transition 

				print("!!!!!game controller ....no ispy action "+str(attempt))
				# the robot verbally encourages the child
				path=ROOT_TEGA_SPEECH_FOLDER + 'general/others/'

				speech_file = random.choice([ path+i+".wav" for i in self.tega_speech_dict["general/others"] if 'no_ispy_action_alert'+str(attempt)+'_response' in i])
				self.ros_node_mgr.send_robot_cmd(RobotBehaviors.ROBOT_CUSTOM_SPEECH,speech_file)

				# if attempt == 2: # first alert
				# 	# robot intervenes (robot spies an object for the child)
				# 	# select an object and then find it for the child
				# 	self._perform_robot_virtual_action(RobotBehaviors.VIRTUALLY_CLICK_CORRECT_OBJ)
			
				# 	action = RobotBehaviors.ROBOT_OFFER_HELP

				# 	question_query_path = '/'.join([self.role.name.lower(), self.state.replace('TURN',''), action])
				# 	print("question query!!")
				# 	print(question_query_path)

				# 	question_query = self.question_answer_dict[question_query_path]
				# 	question_speech_file = ROOT_TEGA_SPEECH_FOLDER + "questions/" +question_query['question'][0]+".wav"
					
				# 	print(question_speech_file)
				# 	self.ros_node_mgr.send_robot_cmd(RobotBehaviors.ROBOT_CUSTOM_SPEECH, question_speech_file)
				
				# 	getattr(self, ris.Triggers.ROBOT_HELP_TRIGGER)()


		def on_tega_state_received(self,data):
			## call back function when a tega state message is received from Tega phone.
			# this function should be called multiple times per second
			# robot is in motion or playing sound or not
			self.tega_is_playing_sound = data.is_playing_sound

        		
		def on_tega_new_asr_result(self,data):
			# callback function when new asr results are received from Tega
			print("!!!new asr results...!!!!")

			if not ris.LISTEN_CHILD_SPEECH_RESPONSE in self.state:
				# FSM is not in the question's answer listening mode
				print("FSM not in the answer listening mode. RETURN")
				return 

			getattr(self, ris.Triggers.SPEECH_RECEIVED)()
			print(self.state)
			self.asr_input = data.transcription.lower().strip()
			self.asr_input_confidence = data.confidence

			print("\n#####Tega Heard: {0}\n".format(self.asr_input))

			# get robot's contigent response based on child's speech content
			robot_actions = self.role_behavior_mapping.get_robot_response_to_answer(self.asr_input)
			
			for action in robot_actions:
				print("action: "+action)
				self.ros_node_mgr.send_robot_cmd(action)

			self._wait_until_all_audios_done()
			print("trigger QA finished")
			getattr(self, ris.Triggers.QA_FINISHED)() # q & a activiity is done
			print(self.state)


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
				
				print("wait until all audios done....")
				self._wait_until_all_audios_done()

				# robot's response 
				self.get_turn_taking_actions()

		def react(self,gameStateTrigger,  clicked_obj_name = ""):
			'''
			react to ispy game state change
			'''
			# variables created for sending interaction data to ROS
			self.clicked_right_obj = self.task_controller.isTarget(clicked_obj_name)
			self.clicked_obj_name = clicked_obj_name
			self.gameStateTrigger = gameStateTrigger

			if gameStateTrigger == gs.Triggers.TARGET_OBJECT_COLLECTED:
				self._perform_robot_physical_action(ras.PRONOUNCE_CORRECT)
				self._perform_robot_physical_action(ras.TURN_FINISHED)
				if self.state == ris.CHILD_TURN or self.state == ris.ROBOT_TURN+'_'+ris.CHILD_HELP:
					# the child finds the correct object
					self.child_states.update_child_turn_result(True)

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
					if self.clicked_right_obj == True:
						self._perform_robot_physical_action(ras.OBJECT_FOUND)
					# If incorrect, show doubt 
					else:
						self._perform_robot_physical_action(ras.OBJECT_CLICKED)
				
			elif gameStateTrigger  == gs.Triggers.SAY_BUTTON_PRESSED:
				self._perform_robot_physical_action(ras.OBJECT_PRONOUNCED)

				

			elif gameStateTrigger  == gs.Triggers.NONTARGET_OBJECT_COLLECTED:
				# Check to see if it is incorrect
				# If correct, will be within target_object_collected -> Happy emotion
				# If incorrect, not target_object -> Sad emotion 
				
				self._perform_robot_physical_action(ras.WRONG_OBJECT_FAIL)
				self._perform_robot_physical_action(ras.TURN_FINISHED)
				if self.state == ris.CHILD_TURN or self.state == ris.ROBOT_TURN+'_'+ris.CHILD_HELP:
					# the child finds the correct object
					self.child_states.update_child_turn_result(False)
					

			elif gameStateTrigger == gs.Triggers.SCREEN_MOVED:
				self._perform_robot_physical_action(ras.SCREEN_MOVED)

				if ris.ROBOT_TURN in self.state:		
					t = threading.Thread(target=self._wait_for_robot_virutal_action)
					t.start()

		def _wait_for_robot_virutal_action(self):
			'''
			create a thread. wait for all child-robot interaction is over before letting the robot click an obj
			'''
			
			flag = True
			while flag:
				if self.state == ris.ROBOT_TURN:
					flag = False
					break
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

		def _wait_until_all_audios_done(self):
			'''
			wait until tega is done speaking all speeches from last turn
			'''
			all_done = False
			while all_done == False:
				if self.tega_is_playing_sound==False:
					time.sleep(0.5)
					if self.tega_is_playing_sound==False:
						break
					else:
					
						continue
				else:
					continue

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
		

		def _perform_robot_physical_action(self,action_type):
			'''
			send the physical action message via ROS to the robot
			'''
			def _get_tega_speech(action_type,speech_type = 'general'):
				'''
				get tega speech audios based on current action type, robot's role
				'''

				if isinstance(self.role,str):
					return ""
				else:
					speech_audio_path = '/'.join([speech_type,self.role.name.lower(), self.state.replace('TURN','')])
					print("path: "+speech_audio_path)
					try:
						all_audio_arrs= self.tega_speech_dict[speech_audio_path]
						
						speech_audios = [ ROOT_TEGA_SPEECH_FOLDER+speech_audio_path+"/"+i+'.wav' for i in all_audio_arrs if action_type in i]
					
						return random.choice(speech_audios)
					except KeyError as k:
						print("ERROR: Couldn't find audio file path in tega speech json files")
						return ""
					except IndexError:
						print("No audio files are available.")
						return ""
		

			general_actions = self.physical_actions[action_type]['general']

			try:
				optional_actions = self.physical_actions[action_type]['optional']
			except:
				optional_actions = {}

			
			actions = { i:1.0 for i in general_actions}
			actions.update(optional_actions)

			#speech_actions = self._get_tega_speech(action_type)

			# send physical moition commands
			for action, prob in actions.items():
				
				if random.random() > prob:
					continue

				action = self.role_behavior_mapping.get_action_name(action) # get the correct name
				# if the action is to pronounce a word, then specify a word to pronounce

				while self.tega_is_playing_sound==True:
					continue
					if self.tega_is_playing_sound==False:
						break

				if "Q_" in action: # action is a question asking action
					self._robot_question_asking(action)

				else: 
					input_data = ""
				
					if action == RobotBehaviors.ROBOT_SAY_WORD:
						input_data = self.robot_clickedObj
					elif action == RobotBehaviors.BASED_ON_PROMPTS_SPEECH or action == RobotBehaviors.HINT_SPEECH or action == RobotBehaviors.KEYWORD_DEFINITION_SPEECH:
						input_data = self.task_controller.get_vocab_word()
					elif action == RobotBehaviors.VOCAB_EXPLANATION_SPEECH:
						input_data = [self.task_controller.get_vocab_word(), self.robot_clickedObj]
					elif action == RobotBehaviors.ROBOT_CUSTOM_SPEECH:
						input_data = _get_tega_speech(action_type) # speech file name

				
					self.ros_node_mgr.send_robot_cmd(action,input_data)
					time.sleep(1)

					# publish data of child-robot interaction via ROS
				self._ros_publish_data(action)	

		def _ros_publish_data(self,action):
			'''
			public ros data on child-robot interaction
			'''
			msg = iSpyChildRobotInteraction()
			
			# current turn: child or robot?
			msg.whoseTurn = self.state

			# robot's current role: expert or novice?
			msg.robotRole = self.role.name if not isinstance(self.role,str) else self.role

			# robot's current behavior (action): question asking, feedback, hints? 
			msg.robotBehavior= action

			# robot's clicked object
			msg.robotClickedObj = self.robot_clickedObj

			# clicked object (either robot's or child's) right or wrong (bool)
			msg.clickedRightObject  = self.clicked_right_obj 

			# clicked object name (either robot's or child's)
			msg.clickedObjName = self.clicked_obj_name 

			# current game task index: 
			msg.gameTask = self.task_controller.current_task_index 

			# vocab word in the current game task
			msg.taskVocab = self.task_controller.get_vocab_word()

			# number of retrieved object for the given task
			msg.numFinishedObjects = self.task_controller.num_finished_words

			# number of questions the robot asked
			msg.numQsAsked = self.child_states.num_robot_questions_asked
 
			# number of questions the child answered
			msg.numQsAnswered = self.child_states.num_robot_questions_answered

			# number of child's attempts of retrieving an object
			msg.numChildAttempts = self.child_states.current_num_trials

			# number of child's correct attempts (num of objs collected by the child)
			msg.numChildCorrectAttempts = self.child_states.current_num_correcft_trials

			# game state trigger (e.g., object clicked, object found, object pronounced)
			msg.gameStateTrigger = self.gameStateTrigger

			self.ros_node_mgr.pub_child_robot_interaction.publish(msg)

		def _robot_question_asking(self,question_cmd):
			'''
			robot asks a question and waits for the child to answer
			'''
			
			# FSM transitions to QA activity
			getattr(self, ris.Triggers.ROBOT_QUESTION)() # trigger the FSM transition
			

			if question_cmd == RobotBehaviors.Q_END_OF_TURN:
				self.ros_node_mgr.send_robot_cmd(question_cmd, self.task_controller.get_vocab_word())
			else:
				question_speech_file = self.role_behavior_mapping.get_robot_question(question_cmd)
				if question_speech_file == "":
					return
				self.ros_node_mgr.send_robot_cmd(question_cmd, question_speech_file)

			# when tega is done speaking, enter asr listening mode
			self._wait_until()

		
			getattr(self, ris.Triggers.LISTEN_RESPONSE)()

			if question_cmd == RobotBehaviors.ROBOT_ASK_HELP:
				# robot asks the child to help find an object
				# send a ros command to enable child's interaction with the tablet
				self.ros_node_mgr.send_ispy_cmd(iSpyCommand.ROBOT_VIRTUAL_ACTIONS,{"robot_action":RobotBehaviors.ROBOT_ASK_HELP,"clicked_object":""})
				getattr(self, ris.Triggers.CHILD_HELP_TRIGGER)()
				print("current state: "+self.state)

			# waiting for the child's answer. robot responds to the child's answer
			# robot_response_arr = get_robot_response_to_answer(child_answer)
			# for each_response in robot_response_arr:
			# 	self.ros_node_mgr.send_robot_cmd(each_response)


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
			

		
