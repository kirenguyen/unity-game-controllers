import time
import json

#from transitions import Machine
from transitions.extensions import HierarchicalMachine as Machine

from ..BaseClassFSM import BaseClassFSM
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
from datetime import datetime, timedelta

import os
import timestring
import rospy


ROOT_TEGA_SPEECH_FOLDER = 'roleswitching18/'


class ChildRobotInteractionFSM(BaseClassFSM):
		'''
		child robot interaction FSM for robot's role switching project
		It communicates with iSpyGameFSM, reinforcemnet learning agent model of the robot, robot's behaviors
		and ChildStates
		'''

		def __init__(self,ros_node_mgr,task_controller,game_controller,participant_id,game_round):
			# use hierachical FSM here. The python package can be found here: https://github.com/pytransitions/transitions
			self.states = [ {'name': ris.ROBOT_TURN, 'children':[ris.QUESTION_ASKING,ris.LISTEN_CHILD_SPEECH_RESPONSE, ris.PARSE_CHILD_SPEECH_RESPONSE, ris.CHILD_HELP]}, {'name':ris.CHILD_TURN,'children':[ ris.NO_INTERACTION_1, ris.QUESTION_ASKING ,ris.LISTEN_CHILD_SPEECH_RESPONSE, ris.PARSE_CHILD_SPEECH_RESPONSE, ris.ROBOT_HELP]} ]
			self.transitions = [
				{'trigger': ris.Triggers.CHILD_TURN_DONE, 'source': ris.CHILD_TURN, 'dest': ris.ROBOT_TURN },
				{'trigger': ris.Triggers.ROBOT_TURN_DONE, 'source': ris.ROBOT_TURN, 'dest': ris.CHILD_TURN},
				{'trigger': ris.Triggers.MAX_TIME, 'source': '*', 'dest': ris.ROBOT_TURN },

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
				{'trigger': ris.Triggers.RETRY_QA, 'source':ris.ROBOT_TURN+'_'+ris.PARSE_CHILD_SPEECH_RESPONSE, 'dest': ris.ROBOT_TURN+'_'+ris.LISTEN_CHILD_SPEECH_RESPONSE }]

			super().__init__(ros_node_mgr, task_controller, game_controller, participant_id, game_round)

			tega_speech_file = open("iSpyGameController/res/tega_speech.json")
			self.tega_speech_dict = json.loads(tega_speech_file.read())

			self.check_existence_of_asr_rostopic()

			
			self.ros_node_mgr.start_tega_state_listener(self.on_tega_state_received)

			self.ros_node_mgr.start_tega_asr(self.on_tega_new_asr_result,self.on_jibo_new_asr_result_callback)

		def check_existence_of_asr_rostopic(self):

			'''
			check whether google asr rostopic exists
			'''
			import rospy
			topics = rospy.get_published_topics()
			self.asr_result_topic = False

			if '/asr_result' in [i[0] for i in topics] and '/jibo_asr_result' not in [i[0] for i in topics]:
				print("=========TEGA/Google asr result publisher exists=========")
				self.asr_result_topic = True
			elif '/jibo_asr_result' in [i[0] for i in topics]:
				print("=========JIBO asr result publisher exists======")
				self.asr_result_topic = True
			else:
				print(topics)
				print("WARNING!!! ASR RESULT PUBLISHER DOES NOT EXIST. This may be a false negative. Run the game once, then check again.")

		def on_child_max_elapsed_time(self):
			''' max elapsed time for a child's turn'''
			# print("!!!!on child max elapsed time....")
			# print(ris.CHILD_TURN in self.state)
			# print(self.reset_elapsed_time)
			# if ris.CHILD_TURN in self.state and self.reset_elapsed_time == False:
			# 	self.elapsed = "True"
			# 	self._ros_publish_data()
			# 	self.elapsed = ""
			# 	self.reset_elapsed_time = True
			# 	print("!!!!")
			# 	self.turn_taking(max_time=True)
			pass

		def on_enter_childTURN(self):
			super().on_enter_childTURN()

		def on_enter_robotTURN(self):
			
			self.turn_start_time = datetime.now()
			self.turn_end_time = None
			self.turn_duration = ""
			self.current_task_turn_index += 1
		
			self.robot_clickedObj = ""
			self.explore_action = ""
			try:
				if self.help_response and not GlobalSettings.USE_TEGA:
					self.virtual_action = ""
				elif GlobalSettings.USE_TEGA:
					self.virtual_action = ""
				else:
					print("Child responded NO to help")
			except:
				print("WHAT IS GOING ONNNN")
			print("")
			print("3333333333333333333333333")
			print("right after on_enter_robotTurn : {}".format(self.virtual_action))
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
			print ("on_enter_childTURN_robotHelp")
			if self.continue_robot_help:
				self.ros_node_mgr.send_ispy_cmd(iSpyCommand.ROBOT_VIRTUAL_ACTIONS,{"robot_action":"ROBOT_OFFER_HELP","clicked_object":""})
				# self.ros_node_mgr.send_robot_cmd(RobotBehaviors.ROBOT_CUSTOM_SPEECH, ROOT_TEGA_SPEECH_FOLDER + "general/others/robot_help.wav") # "i'll help you then!"
				self.robot_clickedObj = self.task_controller.get_obj_for_robot(True)
				self.ros_node_mgr.send_ispy_cmd(iSpyCommand.ROBOT_VIRTUAL_ACTIONS,{"robot_action":RobotBehaviors.VIRTUALLY_CLICK_CORRECT_OBJ,"clicked_object":self.robot_clickedObj})
				self.continue_robot_help = True
			self._ros_publish_data()

		def on_enter_robotTURN_childHelp(self):
			print ("on_enter_robotTURN_childHelp")
			self.child_states.child_help = True
			self.ros_node_mgr.send_ispy_cmd(iSpyCommand.ROBOT_VIRTUAL_ACTIONS,{"robot_action":"ROBOT_ASK_HELP","clicked_object":""}) # enable the child to interact with the tablet
			if GlobalSettings.USE_TEGA:
				self.ros_node_mgr.send_robot_cmd(RobotBehaviors.ROBOT_CUSTOM_SPEECH, ROOT_TEGA_SPEECH_FOLDER + "general/others/child_help.wav") # "now you can span the screen around"
			else:
				print("NOW YOU CAN PAN THE SCREEN AROUND")
				self.ros_node_mgr.send_robot_cmd(RobotBehaviors.ROBOT_CUSTOM_SPEECH, ("others","child_help")) # "now you can span the screen around"
			self.virtual_action = ""
			self._ros_publish_data()

		def listen_child_speech(self):
			'''
			listen to child's speech
			called by listenChildSpeechResponse in either child's turn or robot's turn
			'''
			def timeout_alert():
				print("######========timeout_alert just called========#######")
				self.ros_node_mgr.stop_asr_listening()
				time.sleep(2.0)
				# 6 seconds
				if ris.LISTEN_CHILD_SPEECH_RESPONSE in self.state: 
					self.on_tega_new_asr_result("")
					print("~~~~~~inside of of timeout_alert~~~~~")

			# start ASR listening mode
			print("\n~~~~~~~~~~----ENTER STATE: listen child speech response----~~~~~~~~~~")
			time.sleep(0.4)
			self._wait_until_all_audios_done()
			print("INFO: ASR start listening")
			self.ros_node_mgr.start_asr_listening()
			print("!!!!!!!!!!!JIBO IS LISTENING!!!!!!!!!!!")
			threading.Timer(4.5, timeout_alert).start() # checking for timeout 
			

			#TODO: Does this have anything to do with it?
			if not self.asr_result_topic and GlobalSettings.USE_TEGA: # if the asr result topic publsiher doesn't exist
				# manually call the asr result callback function
				print("ASR result topic is not working")
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

		def on_jibo_new_asr_result_callback(self,data):
			print("====testing====== Jibo ASR Result callback function")

		def on_tega_new_asr_result(self,data):
			# callback function when new asr results are received from Tega
			# it will be called manually if the asr has not been initialized

			print("\nCALLBACK: new asr results received")

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

			print("ASR STOP LISTENING RESULTING FROM: on_tega_new_asr_result")

			#TODO: Consider how well this actually works
			if data:
				self.ros_node_mgr.stop_asr_listening()

			self._ros_publish_data()

			if data:

				self.asr_input = data.transcription.lower().strip()
				self.asr_input_confidence = data.confidence

				print("\n#####Tega Heard: {0}\n".format(self.asr_input))

				if self.asr_input: # if the child gives a response
					self.child_states.num_robot_questions_answered += 1 

					self.child_answer_content = self.asr_input

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
			self.help_response = help_response
			help_no_repeat = True if ("OFFER_HELP" in self.role_behavior_mapping.current_question_query_path) else False

			if "ROBOT" in action:
				self.ros_node_mgr.send_robot_cmd(action)
			else:
				if not ("INDUCE" in self.role_behavior_mapping.current_question_query_path or help_no_repeat) and (self.attempt == 1):
					if GlobalSettings.USE_TEGA:
						path = ROOT_TEGA_SPEECH_FOLDER + "questions/"
						self.ros_node_mgr.send_robot_cmd(RobotBehaviors.ROBOT_CUSTOM_SPEECH, path+ action +".wav")
					else:
						self.ros_node_mgr.send_robot_cmd(RobotBehaviors.ROBOT_CUSTOM_SPEECH, ("questions", action))
			time.sleep(0.5)
			self._wait_until_all_audios_done()


			if self.attempt == 0: # get child's response
				print("GETTING CHILD'S RESPONSE. THIS IS THE ANSWER OF HELP_RESPONSE: ", help_response )
				if "HELP" in self.role_behavior_mapping.current_question_query_path and help_response: # child agrees to help find an object
					# send a ros command to enable child's interaction with the tablet
					getattr(self, ris.Triggers.HELP_TRIGGER)() #converts string to fnc
					if ris.ROBOT_HELP in self.state: 
						print ("INFO: robot helping action starts\n")
					elif ris.CHILD_HELP in self.state: 
						print("INFO: child helping action starts\n")
				elif "HELP" in self.role_behavior_mapping.current_question_query_path and not help_response: # child does not want to help the robot
					try:
						print("Child does not want to help or get help")
						try:
							self.ros_node_mgr.send_robot_cmd(RobotBehaviors.ROBOT_CUSTOM_SPEECH, ("questions", action))	#Can you help me next time/I will make a guess for now
							getattr(self, ris.Triggers.QA_FINISHED)()
						except:
							print("JIBO WANTS TO DO: ", action)
					except:
						print("SOMETHING WENT WRONG. The likely issue is that there is no action that suits the child's response.")


				elif "END_REMINDER" in self.role_behavior_mapping.current_question_query_path:
					# test for task end response 

					print("INFO: QA finished\n")
					getattr(self, ris.Triggers.QA_FINISHED)()
					print("IN ELIF END_REMINDER1!", "CONDITIONAL: ", self.role_behavior_mapping.get_child_answer_type(self.asr_input))
					if self.role_behavior_mapping.get_child_answer_type(self.asr_input) == "negative" or self.role_behavior_mapping.get_child_answer_type(self.asr_input) == "others":
						time.sleep(2)
						self.ros_node_mgr.send_robot_cmd(RobotBehaviors.ROBOT_TASK_END_RESPONSE, self.task_controller.get_vocab_word())
						self._wait_until_all_audios_done()
						time.sleep(3)
					else:
						action = self.role_behavior_mapping.get_robot_response_to_answer(self.asr_input)
						self.ros_node_mgr.send_robot_cmd(RobotBehaviors.ROBOT_TASK_SPEECH_RESPONSE, action)
						self._wait_until_all_audios_done()
						print("")
						print("THIS IS WHAT JIBO IS DOING: ", action)
						print("")
					
					# if not self.task_controller.task_in_progress:
					# 	self.start_task_end_assessment(self.task_number)
					if not self.task_controller.task_in_progress:
						self.start_task_end_celebration(self.task_number)

				elif "INDUCE" in self.role_behavior_mapping.current_question_query_path:

					print("INFO: QA finished\n")
					getattr(self, ris.Triggers.QA_FINISHED)()
					print("IN ELIF END_REMINDER2!", "CONDITIONAL: ", self.role_behavior_mapping.get_child_answer_type(self.asr_input))

					if self.role_behavior_mapping.get_child_answer_type(self.asr_input) == "negative" or self.role_behavior_mapping.get_child_answer_type(self.asr_input) == "others":
						time.sleep(1)
						self.ros_node_mgr.send_robot_cmd(RobotBehaviors.ROBOT_INDUCE_SPEECH_RESPONSE, self.task_controller.get_vocab_word())
						self._wait_until_all_audios_done()
					else:
						action = self.role_behavior_mapping.get_robot_response_to_answer(self.asr_input)
						self.ros_node_mgr.send_robot_cmd(RobotBehaviors.ROBOT_TASK_SPEECH_RESPONSE, action)
						self._wait_until_all_audios_done()
						print("")
						print("THIS IS WHAT JIBO IS DOING: ", action)
						print("")
				else:
					print("INFO: QA finished\n")
					getattr(self, ris.Triggers.QA_FINISHED)() # q & a activiity is done

			elif self.attempt == 2: # child gives a response or the child reaches the max attempt
				print("INFO: QA finished\n")
				getattr(self, ris.Triggers.QA_FINISHED)() # q & a activiity is done

				# if the question is test end response
				if "END_REMINDER" in self.role_behavior_mapping.current_question_query_path:
					# time.sleep(1)
					self.ros_node_mgr.send_robot_cmd(RobotBehaviors.ROBOT_TASK_END_RESPONSE, self.task_controller.get_vocab_word())
					self._wait_until_all_audios_done()
					# time.sleep(2)

					if not self.task_controller.task_in_progress:
						self.start_task_end_assessment(self.task_number)
					if not self.task_controller.task_in_progress:
						self.start_task_end_celebration(self.task_number)

				self.attempt = 0
				
			elif self.attempt == 1:
				if "INDUCE" in self.role_behavior_mapping.current_question_query_path:
					print("INFO: QA finished\n")
					getattr(self, ris.Triggers.QA_FINISHED)()
					
					if self.role_behavior_mapping.get_child_answer_type(self.asr_input) == "absence": 
						print ("===== NO RESPONSE TO INDUCE SPEECH QUESTION ======")
						# time.sleep(1)
						self.ros_node_mgr.send_robot_cmd(RobotBehaviors.ROBOT_INDUCE_SPEECH_RESPONSE, self.task_controller.get_vocab_word())
					
					self.attempt = 0

				elif "HELP" in self.role_behavior_mapping.current_question_query_path:
					print("INFO: QA finished\n")
					getattr(self, ris.Triggers.QA_FINISHED)()

					self.attempt = 0

				else:
					print("INFO: RETRY QA\n")
					getattr(self, ris.Triggers.RETRY_QA)()
	
			self._ros_publish_data()

		def turn_taking(self,max_time=False):
			def _get_turn_duration():
				self.turn_end_time = datetime.now()
				self.turn_duration = str(self.turn_end_time - self.turn_start_time)
				self._ros_publish_data()

			_get_turn_duration()

			if ris.CHILD_TURN in self.state: 
				self.child_states.evaluate_rl_action() # evaluate the rl action

			self.child_click_cancel_num =0  # reset child's number of clicks and cancels each turn

			if self.task_controller.task_in_progress:
				if not max_time:
					# check whether it is robot's turn or child's turn in the game play
					if ris.ROBOT_TURN in self.state:
						# stop tracking the previous turn's rewards
						rewards = self.child_states.stop_tracking_rewards(self.state)

						# then, next turn is child's 
						getattr(self, ris.Triggers.ROBOT_TURN_DONE)() # convert the variabel to string, which is the name of the called function
						self.child_states.start_tracking_rewards(self.state)

					elif ris.CHILD_TURN in self.state:

						rewards = self.child_states.stop_tracking_rewards(self.state)
						#self.agent_model.onRewardsReceived(rewards) # update the RL model 
						# then, next turn is robot's
						getattr(self, ris.Triggers.CHILD_TURN_DONE)()
						# start tracking rewards (engagement) for the robot's role during child's turn
						self.child_states.start_tracking_rewards(self.state)
				else:
					getattr(self, ris.Triggers.MAX_TIME)()

				print("\n============================================")
				print("\n=================TURN TAKING===============: "+self.state+'\n')
				# send the turn info (child/robot) to tablet via ROS

				super().turn_taking()

				self._wait_until_all_audios_done()

				# robot's response 
				self.get_turn_taking_actions()
		


		def react(self,gameStateTrigger,  clicked_obj_name = ""):
			'''
			react to ispy game state change
			'''

			print("react func: virt act {}".format(self.virtual_action))
					

			# variables created for sending interaction data to ROS
			self.clicked_obj_name = clicked_obj_name
			self.gameStateTrigger = gameStateTrigger
			self.clicked_right_obj = str(self.task_controller.isTarget(clicked_obj_name)) if clicked_obj_name else ""

			if clicked_obj_name: # if clicked_obj_name exists, then publish data via ROS
				self._ros_publish_data() # publish data of child-robot interaction via ROS
				self.clicked_obj_name = ""


			if gameStateTrigger == gs.Triggers.TARGET_OBJECT_COLLECTED:
				print ("TARGET_OBJECT_COLLECTED")
				self._perform_robot_physical_actions(ras.PRONOUNCE_CORRECT)
				if self.task_controller.task_in_progress:
					self._perform_robot_physical_actions(ras.TURN_SWITCHING)
				else:
					print ("=== END OF MISSION NO NEED FOR TURN SWITCHING ===")
				self._wait_until_all_audios_done()
				self.child_states.update_turn_result(self.state,True) # the child finds the correct object

				if self.state == ris.CHILD_TURN or self.state == ris.ROBOT_TURN+'_'+ris.CHILD_HELP:
					pass

			elif gameStateTrigger  == gs.Triggers.NONTARGET_OBJECT_COLLECTED:
				if not ris.CHILD_HELP in self.state:
					self._perform_robot_physical_actions(ras.WRONG_OBJECT_FAIL)
				self._perform_robot_physical_actions(ras.TURN_SWITCHING)
				self.child_states.update_turn_result(self.state,False) # the child finds the incorrect object

			elif gameStateTrigger  == gs.Triggers.OBJECT_CLICKED:
				if self.state == ris.ROBOT_TURN:
					if self.role == RobotRoles.NOVICE:
						self._perform_robot_physical_actions(ras.WRONG_OBJECT_CLICKED)
					elif self.role == RobotRoles.EXPERT:
						self._perform_robot_physical_actions(ras.RIGHT_OBJECT_FOUND)

					#self._wait_until() # wait until robot is done speaking. then click and pronounce the word
					self._perform_robot_virtual_action(RobotBehaviors.VIRTUALLY_CLICK_SAY_BUTTON)

				if self.state == ris.CHILD_TURN+'_'+ris.ROBOT_HELP:
					self._perform_robot_virtual_action(RobotBehaviors.VIRTUALLY_CLICK_SAY_BUTTON)

				if (self.state == ris.CHILD_TURN or self.state == ris.ROBOT_TURN+'_'+ris.CHILD_HELP) and self.role == RobotRoles.EXPERT:
					if self.clicked_right_obj == "True":  # If correct, given assertion 
						self._perform_robot_physical_actions(ras.RIGHT_OBJECT_FOUND)
					else: # If incorrect, show doubt 
						self._perform_robot_physical_actions(ras.WRONG_OBJECT_CLICKED)

			elif gameStateTrigger  == gs.Triggers.SAY_BUTTON_PRESSED:
				if self.state == ris.CHILD_TURN or ris.CHILD_HELP in self.state:
					self.ros_node_mgr.start_asr_listening() # start asr listening to check whether the child pronoucnes the child or not
					print("ASR STOP LISTENING RESULTING FROM: gameStateTrigger...")
					threading.Timer(3.5, self.ros_node_mgr.stop_asr_listening()).start() # checking for timeout 
				self.continue_robot_help = False
				self._perform_robot_physical_actions(ras.OBJECT_PRONOUNCED)

			elif gameStateTrigger == gs.Triggers.SCREEN_MOVED:
				self._perform_robot_physical_actions(ras.SCREEN_MOVING)

				if ris.ROBOT_TURN in self.state and not ris.CHILD_HELP in self.state:
					print("START FREEZING ROBOT VIRTUAL ACTION. current state: "+self.state)
					#pass	#TODO: This was here???	
					print("right before threading robot virutal action wait: virt act {}".format(self.virtual_action))
					threading.Thread(target=self._robot_virtual_action_wait).start()

			elif gameStateTrigger == gs.Triggers.PRONUNCIATION_PANEL_CLOSED:
				if self.state == ris.CHILD_TURN or ris.CHILD_HELP in self.state:
					self.child_click_cancel_num += 1 
					
			#self._ros_publish_data()

		def _robot_virtual_action_wait(self):
			'''
			create a thread. wait for all child-robot interaction is over letting the robot click an obj
			'''
			while self.state != ris.ROBOT_TURN:
				if self.state == ris.ROBOT_TURN:
					break
			self._wait_until()
			if self.state == ris.CHILD_TURN+'_'+ris.ROBOT_HELP or self.state == ris.ROBOT_TURN:
				print("*************************************")
				print(self.state)
				print("THIS IS WHAT WE'RE DOING IN _ROBOT_VIRTUAL_ACTION_WAIT: ", self.virtual_action)
				self._perform_robot_virtual_action(self.virtual_action)

				#TODO: DELETE THIS AFTER TESTING IT
				self.virtual_action = ""
				print("*************************************")
				print("WE ARE INSIDE _ROBOT_VIRTUAL_ACTION_WAIT")
				print(self.state)
				print(self.virtual_action)
				print("*************************************")


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
					time.sleep(0.1) #0.5
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
			def get_next_robot_role():
				''' 3 conditions: fixed novice, fixed expert and adaptive role switching'''
				cond_switcher = {
					"expert":  RobotRoles(0),
					"novice":	RobotRoles(1),
					"adaptive": RobotRoles(self.child_states.get_next_robot_role()), 
					"random":	RobotRoles(random.choice([0,1]))
				}
				role = cond_switcher[self.subj_cond]
				if role == RobotRoles.EXPERT:
					self.child_states.s2_prev_expert_roles +=1
				return role
			
			if self.state == ris.ROBOT_TURN: 
				#self.child_states.evaluate_rl_action() # evaluate the rl action
				self.role = get_next_robot_role()
				
			physical_actions = self.role_behavior_mapping.get_actions(self.role,self.state,'physical')

			if physical_actions:
				self.physical_actions = physical_actions
				self._perform_robot_physical_actions(ras.TURN_STARTING)
				# wait until robot's actions for TURN_STARTED to complete. the robot first explores the scene
				
				if self.state == ris.ROBOT_TURN: 
					self.explore_action = "VIRTUALLY_EXPLORE"
					time.sleep(3)
					self._perform_robot_virtual_action(RobotBehaviors.VIRTUALLY_EXPLORE)
		
			virtual_action_dict = self.role_behavior_mapping.get_actions(self.role,self.state,'virtual')
			print("==========virtual action dict=================")
			print(virtual_action_dict)

			if virtual_action_dict:
				ran = random.uniform(0,1)
				for key,val in virtual_action_dict.items():
					if ran <= val:
						self.virtual_action = key
						break
				self.virtual_action = virtual_action_dict.keys()[0] if not self.virtual_action else self.virtual_action
				print("virtual action here in side if virtual action dict: {}".format(self.virtual_action))
		def get_robot_general_response(self):
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

			time.sleep(5)

			if self.task_controller.task_in_progress:
				return

			if action_number %2 ==0:
				return 


			action = RobotBehaviors.ROBOT_TASK_END_BEHAVIOR

			# send command for between mission recordings 
			recording_number = action_number % 4 + 1
			self.ros_node_mgr.send_robot_cmd(action, recording_number)

			# send robot action depending on between mission 
			if self.task_controller.task_in_progress:
				return

			time.sleep(5.5)
			behavior_number = random.randint(0,1)
			end_task_behavior_dict = {0: RobotBehaviors.ROBOT_DANCE, 1: RobotBehaviors.ROBOT_PLAY_MUSIC}
			self.ros_node_mgr.send_robot_cmd(end_task_behavior_dict[behavior_number])

		def start_task_end_assessment(self, action_number):

			time.sleep(1.5)

			if self.task_controller.task_in_progress:
				return

			assessment = RobotBehaviors.Q_ROBOT_TASK_END_ASSESSMENT
			self.ros_node_mgr.send_robot_cmd (assessment, self.task_controller.get_vocab_word())
			
		
		def start_task_end_behavior(self, action_number):
			'''
			send between mission celebration behaviors 
			'''

			super().start_task_end_behavior(action_number)

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

				speech_audio_path = '/'.join([speech_type, role_name, cur_state])

				try:
					all_audio_arrs= self.tega_speech_dict[speech_audio_path]

					if GlobalSettings.USE_TEGA:
						speech_audios = [ ROOT_TEGA_SPEECH_FOLDER+speech_audio_path+"/"+i+'.wav' for i in all_audio_arrs if action_type in i]
					else:
						speech_audios = [(i, role_name, cur_state) for i in all_audio_arrs if action_type in i]

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

				try:
					conditional_actions = self.physical_actions[action_type]['conditional']
				except:
					conditional_actions = {}

				def convert_exclusive(actions):
					if actions == {}:
						return {}

					default_action = "LOOK_CENTER"
					exclusive_dict = {}
					len_actions = len(actions)

					test_prob = random.random()
					low = min(actions.values())
					high = max(actions.values())

					for action, prob in actions.items():
						if prob == low:
							temp_action = action

					if test_prob <= low:
						final_action = temp_action
					elif test_prob > high:
						final_action = default_action
					else:
						for action, prob in actions.items():
							if prob >= test_prob and prob <= high:
								high = prob
								final_action = action
							elif prob <  test_prob and prob >= low:
								low = prob

					return {final_action: 1.0}


				# dict {[action, action]: prob} exclusive one happen or the other, with first having prob [0, 1] of happening
				try:
					exclusive_actions = self.physical_actions[action_type]['exclusive']
				except:
					exclusive_actions = {}

				exclusive_actions_dict = convert_exclusive(exclusive_actions)

			
				actions = { i:1.0 for i in general_actions}
				actions.update(optional_actions)
				actions.update(exclusive_actions_dict)

				if ris.ROBOT_HELP in self.state:
					actions.update(conditional_actions)


				#speech_actions = self._get_tega_speech(action_type)

				# send physical moition commands
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
							elif action == RobotBehaviors.NOVICE_ROLE_KEYWORD:
								input_data = self.task_controller.get_vocab_word()


							self.ros_node_mgr.send_robot_cmd(action,input_data)

					time.sleep(0.5)

					# publish data of child-robot interaction via ROS
					self._ros_publish_data(action)
			while True:
				if self.state == ris.CHILD_TURN or self.state == ris.ROBOT_TURN or 'help' in self.state.lower():
					_execute_actions()	
					break
				else:
					continue


		

		def _ros_publish_data(self,action="", v_action = "", ispy_action=False):
			'''
			public ros data on child-robot interaction
			'''

			self.curr_robot_action = action
	
			msg = iSpyChildRobotInteraction()
			
			# add header
			msg.header = Header()
			msg.header.stamp = rospy.Time.now()

			# current game task index: 
			msg.gameTask = self.task_controller.current_task_index 

			# vocab word in the current game task
			msg.taskVocab = self.task_controller.get_vocab_word()

			msg.taskStartTime = str(self.task_controller.get_task_time()['start'])

			msg.taskEndTime =  str(self.task_controller.get_task_time()['end'])

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

			msg.numTotalAttemptsForTask = [self.child_states.total_num_trials,self.child_states.numChildAttemptsCurrTask]

			msg.numChildClickCancelForTurn = self.child_click_cancel_num 

			#msg.numHintButtonPressedForTask = self.numHintButtonPressedForTask

			msg.numQAForTurn = [self.child_states.num_robot_questions_asked, self.child_states.pos_answers, 
									self.child_states.neg_answers, self.child_states.other_answers,
									self.child_states.no_answers_attempt1,self.child_states.no_answers_attempt2]
  

			msg.gameStateTrigger = self.gameStateTrigger

			msg.currentInteractionState = self.state

			msg.currentGameState = self.game_controller.FSM.state

			msg.robotBehavior = action

			if self.virtual_action:
				msg.robotVirtualBehavior =  str(self.virtual_action)
			elif self.explore_action:
				msg.robotVirtualBehavior = str(self.explore_action)
			else:
				msg.robotVirtualBehavior = ""

			msg.robotClickedObj = self.robot_clickedObj

			msg.clickedRightObject = self.clicked_right_obj

			msg.clickedObjName = self.clicked_obj_name 

			msg.numTouchAbsenceAlertPerTask = self.child_states.numTouchAbsenceAlertPerTask #######

			msg.objectWordPronounced = self.child_states.objectWordPronounced



			msg.ispyAction = ["", "", "", "", ""]
			if ispy_action == True:
				msg.ispyAction  = [str(self.game_controller.isDragging), str(self.game_controller.pointerClick), str(self.game_controller.onPinch), str(self.game_controller.isScalingUp), str(self.game_controller.isScalingDown)]


			msg.maxElapsedTime = self.elapsed
			##############

			self.ros_node_mgr.pub_child_robot_interaction.publish(msg)


		def _robot_question_asking(self,question_cmd):
			'''
			robot asks a question and waits for the child to answer
			'''
			self._ros_publish_data(question_cmd)

			self.check_existence_of_asr_rostopic()
			self.continue_robot_help = True
			if self.asr_result_topic == False and GlobalSettings.USE_TEGA: # if google asr shuts down, reopen it
				print("\nINFO: Google ASR is restarting!!\n")
				os.system("xterm -bg brown -geometry 45x20+300+550 -T \"Speech Recognition\" -e \"python3 /home/huilichen/catkin_ws/src/asr_google_cloud/src/ros_asr.py\" &")
				time.sleep(2)
				while self.asr_result_topic == False:
					self.check_existence_of_asr_rostopic()
					if self.asr_result_topic == True:
						print("===============yeah....asr result exists!!!")
						break
			else:
				if self.asr_result_topic == True:
					print("ASR RESULT EXISTS")
				else:
					print("ASR RESULT NOT FOUND")


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
			self._ros_publish_data("", action)

			print("\n++++++++++++++++++++virtual action: "+action)

			action = self.role_behavior_mapping.get_action_name(action) # get the correct name

			if action == RobotBehaviors.VIRTUALLY_CLICK_CORRECT_OBJ:
				self.robot_clickedObj = self.task_controller.get_obj_for_robot(True)
				
			elif action == RobotBehaviors.VIRTUALLY_CLICK_WRONG_OBJ:
				self.robot_clickedObj = self.task_controller.get_obj_for_robot(False)

			elif action == RobotBehaviors.VIRTUALLY_EXPLORE:
				self.robot_clickedObj = ""
			
			print("-----get virtual action----: "+action+"---clicked obj: "+self.robot_clickedObj)
			self.ros_node_mgr.send_ispy_cmd(iSpyCommand.ROBOT_VIRTUAL_ACTIONS,{"robot_action":action,"clicked_object":self.robot_clickedObj})
