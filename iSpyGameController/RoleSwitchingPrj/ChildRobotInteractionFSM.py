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
from datetime import datetime

import os
import timestring


ROOT_TEGA_SPEECH_FOLDER = 'roleswitching18/'


class ChildRobotInteractionFSM:
		'''
		child robot interaction FSM for robot's role switching project
		It communicates with iSpyGameFSM, reinforcemnet learning agent model of the robot, robot's behaviors
		and ChildStates
		'''

		def __init__(self,ros_node_mgr,task_controller,game_controller):
			# use hierachical FSM here. The python package can be found here: https://github.com/pytransitions/transitions
			self.states = [ {'name': ris.ROBOT_TURN, 'children':[ris.QUESTION_ASKING,ris.LISTEN_CHILD_SPEECH_RESPONSE, ris.PARSE_CHILD_SPEECH_RESPONSE, ris.CHILD_HELP]}, {'name':ris.CHILD_TURN,'children':[ ris.NO_INTERACTION_1, ris.QUESTION_ASKING ,ris.LISTEN_CHILD_SPEECH_RESPONSE, ris.PARSE_CHILD_SPEECH_RESPONSE, ris.ROBOT_HELP]} ]
			self.transitions = [
				{'trigger': ris.Triggers.CHILD_TURN_DONE, 'source': ris.CHILD_TURN, 'dest': ris.ROBOT_TURN },
				{'trigger': ris.Triggers.ROBOT_TURN_DONE, 'source': ris.ROBOT_TURN, 'dest': ris.CHILD_TURN},

				# help trigger. either robot asks for help. or child asks for help
				{'trigger': ris.Triggers.HELP_TRIGGER, 'source':ris.CHILD_TURN+'_'+ris.PARSE_CHILD_SPEECH_RESPONSE, 'dest': ris.CHILD_TURN+'_'+ris.ROBOT_HELP},
				{'trigger':	ris.Triggers.HELP_TRIGGER, 'source': ris.ROBOT_TURN+'_'+ris.PARSE_CHILD_SPEECH_RESPONSE, 'dest': ris.ROBOT_TURN+'_'+ris.CHILD_HELP},
				

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
				{'trigger': ris.Triggers.QA_FINISHED, 'source':ris.CHILD_TURN+'_'+ris.PARSE_CHILD_SPEECH_RESPONSE, 'dest':ris.CHILD_TURN },
				
				# when receiving no response for the first time
				{'trigger': ris.Triggers.RETRY_QA, 'source':ris.CHILD_TURN+'_'+ris.PARSE_CHILD_SPEECH_RESPONSE, 'dest': ris.CHILD_TURN+'_'+ris.LISTEN_CHILD_SPEECH_RESPONSE },
				{'trigger': ris.Triggers.RETRY_QA, 'source':ris.ROBOT_TURN+'_'+ris.PARSE_CHILD_SPEECH_RESPONSE, 'dest': ris.ROBOT_TURN+'_'+ris.LISTEN_CHILD_SPEECH_RESPONSE }
				
			]

			self.state_machine = Machine(self, states=self.states, transitions=self.transitions,
									 initial=ris.CHILD_TURN)

			self.ros_node_mgr = ros_node_mgr

			self.task_controller = task_controller

			self.game_controller = game_controller

			self.child_states = ChildStates()

			self.agent_model = AgentModel()

			self.role_behavior_mapping = RobotRolesBehaviorsMap()

			# robot's physical actions
			self.physical_actions ={}
			# robot's virtual actions on the tablet
			self.virtual_action = ""

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
				print("=========asr result publisher exists=========")
				self.asr_result_topic = True
			else:
				print("======WARNING: asr result publisher does not exist. Remember to start ros_asr.py======")

		

		# def on_exit_childTURN(self):
			
		# 	self.turn_end_time = datetime.now()
		# 	print(self.turn_end_time)
		# 	print(self.turn_start_time)
		# 	self.turn_duration = str(self.turn_end_time - self.turn_start_time)
		# 	self._ros_publish_data()

		# def on_exit_robotTURN(self):
		
		# 	self.turn_end_time = datetime.now()
		# 	self.turn_duration = str(self.turn_end_time - self.turn_start_time)
			
		# 	self._ros_publish_data()

		def on_enter_childTURN(self):
			self.turn_start_time = datetime.now()
			self.turn_end_time = None
			self.turn_duration = ""
			self.current_task_turn_index += 1
			
			self.robot_clickedObj = "" # reset robot's clicked obj
			self.virtual_action = ""
			self._ros_publish_data()
			threading.Timer(3.0, self.start_tracking_child_interaction).start()

		def on_enter_robotTURN(self):
			self.turn_start_time = datetime.now()
			self.turn_end_time = None
			self.turn_duration = ""
			self.current_task_turn_index += 1
		
			self.robot_clickedObj = ""
			self.virtual_action = ""
			self._ros_publish_data()


		def on_enter_childTURN_listenChildSpeechResponse(self):
			'''
			callback function. called when the FSM enters "listenChildSpeechResponse" state
			'''
			self._ros_publish_data()
			self.listen_child_speech()

		def on_enter_robotTURN_listenChildSpeechResponse(self):
			'''
			callback function. called when the FSM enters "listenChildSpeechResponse" state
			'''
			self._ros_publish_data()
			self.listen_child_speech()

		def on_enter_childTURN_robotHelp(self):
			self._ros_publish_data()

		def on_enter_robotTURN_childHelp(self):
			self.ros_node_mgr.send_ispy_cmd(iSpyCommand.ROBOT_VIRTUAL_ACTIONS,{"robot_action":"ROBOT_ASK_HELP","clicked_object":""}) # enable the child to interact with the tablet
			self.ros_node_mgr.send_robot_cmd(RobotBehaviors.ROBOT_CUSTOM_SPEECH, ROOT_TEGA_SPEECH_FOLDER + "general/others/child_help.wav") # "now you can span the screen around"
			self._ros_publish_data()

		def listen_child_speech(self):
			'''
			listen to child's speech
			called by listenChildSpeechResponse in either child's turn or robot's turn
			'''
			print("====================== listen child speech ===================")


			def timeout_alert():
				print("====== time out alert==")
				# 6 seconds
				if ris.LISTEN_CHILD_SPEECH_RESPONSE in self.state: 
					self.on_tega_new_asr_result("")

			# start ASR listening mode
			print("\nENTER STATE: listen child speech response")
			print("===============before time sleep======")
			time.sleep(0.4)
			print("============wait until =======")
			self._wait_until_all_audios_done()
			print("INFO: ASR start listening")
			self.ros_node_mgr.start_asr_listening()
			threading.Timer(6.0, timeout_alert).start() # checking for timeout 
			

			
			if not self.asr_result_topic: # if the asr result topic publsiher doesn't exist
				# manually call the asr result callback function
				print("no asr result topic!!!")
				print("INFO: manually call tega new asr result")
				self.on_tega_new_asr_result("")


		def start_tracking_child_interaction(self):
			'''
			callback function on tracking child's engagment with the tablet. 
			called by ChildRobotInteraction
			'''
			#print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!start tracking child interaction")
			#print("on start tracking child interaction")
			# start a new thread to keep track of elapsed time
			def elapsed_time_alert(stop):
			
				start_time = datetime.now()
				alerted_once = False
				while True:
					delta_time = datetime.now() - start_time
					if stop():
						break
					if delta_time.total_seconds() > 40:
						self.on_no_ispy_action_alert(2)
						break
					elif delta_time.total_seconds() > 5 and alerted_once == False :
						# 10 secs have passed without receiving any tablet interaction input from the cihld
						self.on_no_ispy_action_alert(1)
						alerted_once = True		
				
			self.stop_thread_flag = True
			time.sleep(0.5)
			self.stop_thread_flag = False
			t = threading.Thread(target=elapsed_time_alert, args=(lambda: self.stop_thread_flag,)).start()
		
				
		def stop_tracking_child_interaction(self):
			self.stop_thread_flag = True



		def on_no_ispy_action_alert(self,attempt):
			'''
			callback function when no ispy action within a time period is detected
			when the child is not interacting with the tablet, the robot will try to encourage the child
			called by iSpyDataTracking 
			attempt: first alert, second alert
			'''
			pass 
			# check whether the current state is child_turn_no_interaction_1
			# if self.state != ris.CHILD_TURN:
			# 	return

			# else:

			# 	#getattr(self, ris.Triggers.NO_INTERACTION_ALERT)() # trigger the FSM transition 

			# 	print("INFO: No iSpy Tablet Touches")
				
			# 	self.ros_node_mgr.send_robot_cmd(RobotBehaviors.NO_ISPY_ACTION_ALERT)

			# 	self.child_states.numTouchAbsenceAlertPerTask += 1 # update the number of touch absence alert

			# 	self.stop_tracking_child_interaction()
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
			# it will be called manually if the asr has not been initialized

			print("CALLBACK: new asr results received")

			if self.state == ris.ROBOT_TURN or self.state == ris.CHILD_TURN: # check whether the child pronounces a given word after clicking an object for retrieval
				if data:
					self.child_states.objectWordPronounced = True
				else:
					self.child_states.objectWordPronounced = False

			if not ris.LISTEN_CHILD_SPEECH_RESPONSE in self.state:
				# FSM is not in the question's answer listening mode
				print("FSM not in the answer listening mode. RETURN")
				return 

			getattr(self, ris.Triggers.SPEECH_RECEIVED)()

			self.ros_node_mgr.stop_asr_listening()

			self._ros_publish_data()

			if data:

				self.asr_input = data.transcription.lower().strip()
				self.asr_input_confidence = data.confidence

				print("\n#####Tega Heard: {0}\n".format(self.asr_input))

				if self.asr_input: # if the child gives a response
					self.child_states.num_robot_questions_answered += 1 

					self.child_answer_content = self.asr_input

				#if "END_REMINDER" in self.role_behavior_mapping.current_question_query_path and self.role_behavior_mapping.get_child_answer_type(self.asr_input) == "positive":
				#	self.start_task_end_assessment(self.task_number)
				#	self.start_task_end_celebration(self.task_number)

					#assessment_file = str(self.task_controller.get_vocab_word()) + '_' + "assessment_result.txt"
					#print ("SAVE FILE!!!!!!!!")
					#with open(assessment_file, 'a') as file:
					#	file.write(self.asr_input + '\n')
					#file.close()

				self.attempt = 0

			else:
				print("WARNING: No response from child\n")
				try:
					self.attempt += 1
				except:
					self.attempt = 1

				self.asr_input = "no_response_" + str(self.attempt) # set asr_input to get correct robot's action to "no response"


			# get robot's contigent response based on child's speech content
			self.child_states.update_qa_result(self.role_behavior_mapping.get_child_answer_type(self.asr_input),self.attempt) # update QA results to child states
			action = self.role_behavior_mapping.get_robot_response_to_answer(self.asr_input) # action is based on child's answer
			help_response = self.role_behavior_mapping.get_robot_response_to_help(self.asr_input) # check whether the child gives a positive answer

			if "ROBOT" in action:
				self.ros_node_mgr.send_robot_cmd(action)
			else:
				path = ROOT_TEGA_SPEECH_FOLDER + "questions/"
				self.ros_node_mgr.send_robot_cmd(RobotBehaviors.ROBOT_CUSTOM_SPEECH, path+ action +".wav")

			time.sleep(0.5)
			self._wait_until_all_audios_done()


			if self.attempt == 0: # get child's response
				
				if "HELP" in self.role_behavior_mapping.current_question_query_path and help_response: # robot asks the child to help find an object
					# send a ros command to enable child's interaction with the tablet
					print("INFO: helping action starts\n")
					self.ros_node_mgr.send_ispy_cmd(iSpyCommand.ROBOT_VIRTUAL_ACTIONS,{"robot_action":self.role_behavior_mapping.current_action_name,"clicked_object":""})
					getattr(self, ris.Triggers.HELP_TRIGGER)()
				elif "END_REMINDER" in self.role_behavior_mapping.current_question_query_path:
					# test for task end response 
					if self.role_behavior_mapping.get_child_answer_type(self.asr_input) == "negative" or self.role_behavior_mapping.get_child_answer_type(self.asr_input) == "others":
						print ("====== INCORRECT RESPONSE TO TASK END QUESTION ====== ")
						time.sleep(2)
						self.ros_node_mgr.send_robot_cmd(RobotBehaviors.ROBOT_TASK_END_RESPONSE, self.task_controller.get_vocab_word())
						self._wait_until_all_audios_done()
						time.sleep(2)
					else:
						print ("====== RESPOND [KEYWORD] TO TASK END QUESTION =======")
					self.start_task_end_assessment(self.task_number)
					self.start_task_end_celebration(self.task_number)
					print("INFO: QA finished\n")
					getattr(self, ris.Triggers.QA_FINISHED)()

				else:
					print("INFO: QA finished\n")
					getattr(self, ris.Triggers.QA_FINISHED)() # q & a activiity is done

			elif self.attempt == 2: # child gives a response or the child reaches the max attempt
				print("INFO: QA finished\n")
				getattr(self, ris.Triggers.QA_FINISHED)() # q & a activiity is done

				# if the question is test end response
				if "END_REMINDER" in self.role_behavior_mapping.current_question_query_path:
					print ("===== NO RESPONSE TO TASK END QUESTION ======")
					time.sleep(2)
					self.ros_node_mgr.send_robot_cmd(RobotBehaviors.ROBOT_TASK_END_RESPONSE, self.task_controller.get_vocab_word())
					self.start_task_end_assessment(self.task_number)
					self.start_task_end_celebration(self.task_number)

				self.attempt = 0
				
			elif self.attempt == 1:
				print("INFO: RETRY QA\n")
				getattr(self, ris.Triggers.RETRY_QA)()
	
			self._ros_publish_data()	


		def reset_turn_taking(self):
			self.current_task_turn_index = 0
			self.state = ris.CHILD_TURN
			self.child_states.on_new_task_received() # reset some task-based variables in child's states
			
		def turn_taking(self):

			def _get_turn_duration():
				self.turn_end_time = datetime.now()
				self.turn_duration = str(self.turn_end_time - self.turn_start_time)
				self._ros_publish_data()

			_get_turn_duration()

			self.child_click_cancel_num =0  # reset child's number of clicks and cancels each turn

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

				
				print("\n==========TURN TAKING===============: "+self.state+'\n')
				# send the turn info (child/robot) to tablet via ROS

				self.ros_node_mgr.send_ispy_cmd(iSpyCommand.WHOSE_TURN, {"whose_turn":self.state})

				# update the number of available objects for child's learning states
				self.child_states.set_num_available_objs(self.task_controller.get_num_available_target_objs())
				
				self._wait_until_all_audios_done()

				# robot's response 
				self.get_turn_taking_actions()


		def react(self,gameStateTrigger,  clicked_obj_name = ""):
			'''
			react to ispy game state change
			'''

			# variables created for sending interaction data to ROS
			self.clicked_obj_name = clicked_obj_name
			self.gameStateTrigger = gameStateTrigger
			self.clicked_right_obj = str(self.task_controller.isTarget(clicked_obj_name)) if clicked_obj_name else ""

			if clicked_obj_name: # if clicked_obj_name exists, then publish data via ROS
				self._ros_publish_data() # publish data of child-robot interaction via ROS
				self.clicked_obj_name = ""


			if gameStateTrigger == gs.Triggers.TARGET_OBJECT_COLLECTED:
				self._perform_robot_physical_actions(ras.PRONOUNCE_CORRECT)
				self._perform_robot_physical_actions(ras.TURN_SWITCHING)
				self.child_states.update_turn_result(self.state,True) # the child finds the correct object

				if self.state == ris.CHILD_TURN or self.state == ris.ROBOT_TURN+'_'+ris.CHILD_HELP:
					pass

			elif gameStateTrigger  == gs.Triggers.NONTARGET_OBJECT_COLLECTED:
				
				self._perform_robot_physical_actions(ras.WRONG_OBJECT_FAIL)
				self._perform_robot_physical_actions(ras.TURN_SWITCHING)
				self.child_states.update_turn_result(self.state,False) # the child finds the correct object

				if self.state == ris.CHILD_TURN or self.state == ris.ROBOT_TURN+'_'+ris.CHILD_HELP:
					pass

			elif gameStateTrigger  == gs.Triggers.OBJECT_CLICKED:
				if self.state == ris.ROBOT_TURN or self.state == ris.CHILD_TURN+'_'+ris.ROBOT_HELP:
					self._perform_robot_physical_actions(ras.WRONG_OBJECT_CLICKED)

					self._wait_until() # wait until robot is done speaking. then click and pronounce the word
					self._perform_robot_virtual_action(RobotBehaviors.VIRTUALLY_CLICK_SAY_BUTTON)

				if (self.state == ris.CHILD_TURN or self.state == ris.ROBOT_TURN+'_'+ris.CHILD_HELP) and self.role == RobotRoles.EXPERT:
					if self.clicked_right_obj == "True":  # If correct, given assertion 
						self._perform_robot_physical_actions(ras.RIGHT_OBJECT_FOUND)
					else: # If incorrect, show doubt 
						self._perform_robot_physical_actions(ras.WRONG_OBJECT_CLICKED)

			elif gameStateTrigger  == gs.Triggers.SAY_BUTTON_PRESSED:
				if self.state == ris.CHILD_TURN or ris.CHILD_HELP in self.state:
					self.ros_node_mgr.start_asr_listening() # start asr listening to check whether the child pronoucnes the child or not
					threading.Timer(3.5, self.ros_node_mgr.stop_asr_listening()).start() # checking for timeout 
				self._perform_robot_physical_actions(ras.OBJECT_PRONOUNCED)

			elif gameStateTrigger == gs.Triggers.SCREEN_MOVED:
				self._perform_robot_physical_actions(ras.SCREEN_MOVING)

				if ris.ROBOT_TURN in self.state and not ris.CHILD_HELP in self.state:
					#print("START FREEZING ROBOT VIRTUAL ACTION. current state: "+self.state)
					#pass		
					threading.Thread(target=self._robot_virutal_action_wait).start()

			elif gameStateTrigger == gs.Triggers.PRONUNCIATION_PANEL_CLOSED:
				if self.state == ris.CHILD_TURN or ris.CHILD_HELP in self.state:
					self.child_click_cancel_num += 1 
					
			self._ros_publish_data()

		def _robot_virutal_action_wait(self):
			'''
			create a thread. wait for all child-robot interaction is over before letting the robot click an obj
			'''
			while self.state != ris.ROBOT_TURN:
				if self.state == ris.ROBOT_TURN:
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
					print("self.tega is playing sound==false")
					time.sleep(0.1) #0.5
					if self.tega_is_playing_sound==False:
						print("teag sound false 2222")
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
			def get_next_robot_role():
				next_action = random.choice([0,1])
				return RobotRoles(next_action)
			
			if self.state == ris.ROBOT_TURN: self.role = get_next_robot_role()
				
			physical_actions = self.role_behavior_mapping.get_actions(self.role,self.state,'physical')
			self.virtual_action = self.role_behavior_mapping.get_actions(self.role,self.state,'virtual')
			if self.virtual_action:
				self.virtual_action = self.virtual_action[0]

			if physical_actions:
				self.physical_actions = physical_actions
				self._perform_robot_physical_actions(ras.TURN_STARTING)
				# wait until robot's actions for TURN_STARTED to complete. the robot first explores the scene
				
				if self.state == ris.ROBOT_TURN: 
					time.sleep(3)
					self._perform_robot_virtual_action(RobotBehaviors.VIRTUALLY_EXPLORE)


		def get_robot_general_response(self):
			
			print("---get robot general response")
			physical_actions = self.role_behavior_mapping.get_actions("BACKUP",self.state,'physical')
			self.virtual_action = self.role_behavior_mapping.get_actions("BACKUP",self.state,'virtual')

			if self.virtual_action:
				self.virtual_action = self.virtual_action[0]

			if physical_actions:
				self.physical_actions = physical_actions
				self._perform_robot_physical_actions(ras.TURN_STARTING)

				# if self.state == ris.ROBOT_TURN: 
				# 	time.sleep(3)
				# 	self._perform_robot_virtual_action(RobotBehaviors.VIRTUALLY_EXPLORE)

		def start_task_end_celebration(self, action_number):

			time.sleep(7)

			action = RobotBehaviors.ROBOT_TASK_END_BEHAVIOR

			# send command for between mission recordings 
			recording_number = action_number % 4 + 1
			self.ros_node_mgr.send_robot_cmd(action, recording_number)

			# send robot action depending on between mission 

			time.sleep(5.5)
			behavior_number = action_number % 2 
			end_task_behavior_dict = {0: RobotBehaviors.ROBOT_DANCE, 1: RobotBehaviors.ROBOT_PLAY_MUSIC}
			self.ros_node_mgr.send_robot_cmd(end_task_behavior_dict[int(behavior_number)])

		def start_task_end_assessment(self, action_number):

			time.sleep(1.5)

			assessment = RobotBehaviors.Q_ROBOT_TASK_END_ASSESSMENT
			self.ros_node_mgr.send_robot_cmd (assessment, self.task_controller.get_vocab_word())
			
		
		def start_task_end_behavior(self, action_number):
			'''
			send between mission celebration behaviors 
			'''

			print ("End of Task")

			# task reminder at the end of mission 
			self.task_number = action_number

			reminder = RobotBehaviors.Q_ROBOT_TASK_END_REMINDER
			self._robot_question_asking(reminder)


		def _perform_robot_physical_actions(self,action_type):
			'''
			send the physical action message via ROS to the robot
			'''
			def _get_tega_speech(action_type,speech_type = 'general'):
				'''
				get tega speech audios based on current action type, robot's role
				'''
				role_name = self.role if isinstance(self.role,str) else self.role.name.lower()
				cur_state = self.state.replace('TURN','') if str(self.state.replace('TURN','')) != "child_noInteraction1" else "child"
				
				speech_audio_path = '/'.join([speech_type,role_name, cur_state])
				
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
		
			def _execute_actions():
				print("perform robot physical action: "+action_type)
				general_actions = self.physical_actions[action_type]['general']
			
				try:
					optional_actions = self.physical_actions[action_type]['optional']
				except:
					optional_actions = {}

			
				actions = { i:1.0 for i in general_actions}
				actions.update(optional_actions)

				#speech_actions = self._get_tega_speech(action_type)

				# send physical moition commands
				print("get actions for action type: "+action_type)
				print(actions.items())
				for action, prob in actions.items():
				
					print("action: "+action+" | prob:" + str(prob))
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
						print(" robot question asking is done... break for action loop")
						break

					else: 
						# ensure the current FSM state is not in Q & A states
						if not any([ i in self.state for i in [ris.QUESTION_ASKING,ris.LISTEN_CHILD_SPEECH_RESPONSE, ris.PARSE_CHILD_SPEECH_RESPONSE]]):
							input_data = ""
				
							if action == RobotBehaviors.ROBOT_SAY_WORD:
								input_data = self.robot_clickedObj
							elif action == RobotBehaviors.BASED_ON_PROMPTS_SPEECH or action == RobotBehaviors.HINT_SPEECH or action == RobotBehaviors.KEYWORD_DEFINITION_SPEECH or action == RobotBehaviors.REMINDER_SPEECH:
								input_data = self.task_controller.get_vocab_word()
							elif action == RobotBehaviors.VOCAB_EXPLANATION_SPEECH:
								input_data = [self.task_controller.get_vocab_word(), self.robot_clickedObj]
							elif action == RobotBehaviors.ROBOT_CUSTOM_SPEECH:
								input_data = _get_tega_speech(action_type) # speech file name


							self.ros_node_mgr.send_robot_cmd(action,input_data)

					time.sleep(0.5)

					# publish data of child-robot interaction via ROS
					self._ros_publish_data(action)
			while True:
				if self.state == ris.CHILD_TURN or self.state == ris.ROBOT_TURN or 'help' in self.state.lower():
					print("execute actions...")
					_execute_actions()	
					break
				else:
					continue

		def _ros_publish_data(self,action="NA", ispy_action=False):
			'''
			public ros data on child-robot interaction
			'''

			self.curr_robot_action = action
	
			msg = iSpyChildRobotInteraction()

			# current game task index: 
			msg.gameTask = self.task_controller.current_task_index 

			# vocab word in the current game task
			msg.taskVocab = self.task_controller.get_vocab_word()

			msg.taskStartTime = self.task_controller.get_task_time()['start']

			msg.taskEndTime =  self.task_controller.get_task_time()['end']

			msg.taskDuration = self.task_controller.get_task_time()['duration']

			msg.taskTurnIndex = self.current_task_turn_index

			# current turn: child or robot?
			msg.whoseTurn = self.state

			# robot's current role: expert or novice?
			msg.robotRole = self.role.name if not isinstance(self.role,str) else self.role

			msg.turnStartTime = str(self.turn_start_time) 

			msg.turnEndTime = str(self.turn_end_time) 

			msg.turnDuration = self.turn_duration

			###############
			
			msg.numFinishedObjectsForTask = [self.task_controller.num_finished_words,self.child_states.numChildCorrectAttemptsCurrTask ]

			#msg.numFinishedObjectsForTask[1] = 

			msg.numTotalAttemptsForTask = [self.child_states.total_num_trials,self.child_states.numChildAttemptsCurrTask]

 

			msg.numChildClickCancelForTurn = self.child_click_cancel_num 

			msg.numQAForTurn = [self.child_states.num_robot_questions_asked, self.child_states.pos_answers, 
									self.child_states.neg_answers, self.child_states.other_answers,
									self.child_states.no_answers_attempt1,self.child_states.no_answers_attempt2]
  

			msg.gameStateTrigger = self.gameStateTrigger

			msg.currentInteractionState = self.state

			msg.currentGameState = self.game_controller.FSM.state

			msg.robotBehavior = action

			msg.robotVirtualBehavior = str(self.virtual_action) if self.virtual_action else ""

			msg.robotClickedObj = self.robot_clickedObj

			msg.clickedRightObject = self.clicked_right_obj

			msg.clickedObjName = self.clicked_obj_name 

			msg.numTouchAbsenceAlertPerTask = self.child_states.numTouchAbsenceAlertPerTask #######

			msg.objectWordPronounced = self.child_states.objectWordPronounced

			msg.ispyAction = ["", "", "", "", ""]
			if ispy_action == True:
				msg.ispyAction  = [str(self.game_controller.isDragging), str(self.game_controller.pointerClick), str(self.game_controller.onPinch), str(self.game_controller.isScalingUp), str(self.game_controller.isScalingDown)]



			##############

			self.ros_node_mgr.pub_child_robot_interaction.publish(msg)


		def _robot_question_asking(self,question_cmd):
			'''
			robot asks a question and waits for the child to answer
			'''
			print("asr topic...")
			print(self.asr_result_topic)
			self.check_existence_of_asr_rostopic()
			if self.asr_result_topic == False: # if google asr shuts down, reopen it
				print("\nINFO: Google ASR is restarting!!\n")
				os.system("xterm -bg brown -geometry 45x20+300+550 -T \"Speech Recognition\" -e \"python3 /home/huilichen/catkin_ws/src/asr_google_cloud/src/ros_asr.py\" &")
				time.sleep(2)
				while self.asr_result_topic == False:
					self.check_existence_of_asr_rostopic()
					if self.asr_result_topic == True:
						print("===============yeah....asr result exists!!!")
						break



			print("\nINFO: robot question asking")
	
			# FSM transitions to QA activity
			getattr(self, ris.Triggers.ROBOT_QUESTION)() # trigger the FSM transition
			

			if question_cmd == RobotBehaviors.Q_END_OF_TURN:
				self.ros_node_mgr.send_robot_cmd(question_cmd, self.task_controller.get_vocab_word())
				self.child_states.update_qa_info("open_ended",question_cmd)
			else:
				question_speech_file = self.role_behavior_mapping.get_robot_question(question_cmd)
				self.child_states.update_qa_info(self.role_behavior_mapping.get_question_type(),question_cmd)
				if question_speech_file == "":
					return
				self.ros_node_mgr.send_robot_cmd(question_cmd, question_speech_file)

			# when tega is done speaking, enter asr listening mode
			self._wait_until()

		
			getattr(self, ris.Triggers.LISTEN_RESPONSE)()

		


		def _perform_robot_virtual_action(self,action):
			'''
			send the virtual action message via ROS to the tablet 
			'''

			action = self.role_behavior_mapping.get_action_name(action) # get the correct name

			if action == RobotBehaviors.VIRTUALLY_CLICK_CORRECT_OBJ:
				print("----virtually click correct obj")
				self.robot_clickedObj = self.task_controller.get_obj_for_robot(True)
				
			elif action == RobotBehaviors.VIRTUALLY_CLICK_WRONG_OBJ:
				print("---virtually click wrong obj")
				self.robot_clickedObj = self.task_controller.get_obj_for_robot(False)

			elif action == RobotBehaviors.VIRTUALLY_EXPLORE:
				self.robot_clickedObj = ""
			
			print("-----get virtual action----: "+action+"---clicked obj: "+self.robot_clickedObj)
			self.ros_node_mgr.send_ispy_cmd(iSpyCommand.ROBOT_VIRTUAL_ACTIONS,{"robot_action":action,"clicked_object":self.robot_clickedObj})
			

		
