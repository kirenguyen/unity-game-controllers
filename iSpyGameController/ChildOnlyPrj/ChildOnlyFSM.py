from ..BaseClassFSM import BaseClassFSM
from GameUtils.GlobalSettings import iSpyRobotInteractionStates as ris
import random
from unity_game_msgs.msg import iSpyCommand
from unity_game_msgs.msg import iSpyChildRobotInteraction
from ..iSpyTaskController import iSpyTaskController

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

import os
import timestring
import rospy

import json
jibo_tts_data = "iSpyGameController/res/jibo_speech.json"
jibo_tts_file = open(jibo_tts_data)
jibo_tts_dict = json.loads(jibo_tts_file.read())
hints_dict = jibo_tts_dict['hint']


class ChildOnlyFSM(BaseClassFSM):
		'''
		child only interaction FSM
		It communicates with iSpyGameFSM and ChildStates
		'''
		
		def __init__(self,ros_node_mgr,task_controller,game_controller,participant_id,game_round):
			
			# use hierachical FSM here. The python package can be found here: https://github.com/pytransitions/transitions
			
			self.numHintAppear = 0

			self.states = [{'name': ris.CHILD_TURN, 
                                    'children':[ris.CHILD_TURN]}]
			self.transitions = [
				{'trigger': ris.Triggers.CHILD_TURN_DONE, 'source': ris.CHILD_TURN, 'dest': ris.CHILD_TURN },
				# when the child does not interact with the tablet
				{'trigger': ris.Triggers.NO_INTERACTION_ALERT, 'source': ris.CHILD_TURN, 'dest':ris.CHILD_TURN+"_"+ris.NO_INTERACTION_1 }
                            ]
			
			self.state = ris.CHILD_TURN     

			super().__init__(ros_node_mgr,task_controller,game_controller,participant_id,game_round)

		def on_enter_childTURN(self):

			r = random.choice([-1, 1])
			if r > 0: 

				self.ros_node_mgr.send_ispy_cmd(iSpyCommand.BUTTON_DISABLED, {"buttonName": "helpingHintActivate"})
				current_word = self.task_controller.vocab_word
				desired_speech = hints_dict[current_word]
				self.numHintAppear += 1
				print(desired_speech, "desiredSpeech")
				print('~')
				print('~')

				self.ros_node_mgr.send_ispy_cmd(iSpyCommand.SPEAK, {'desiredSpeech': desired_speech})
			else: 
				self.ros_node_mgr.send_ispy_cmd(iSpyCommand.BUTTON_DISABLED, {"buttonName": "helpingHintDeactivate"})
			
			super().on_enter_childTURN()

		def turn_taking(self,max_time=False):
			if self.task_controller.task_in_progress:
				self.on_enter_childTURN()
				super().turn_taking()


		"""
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

			self._ros_publish_data()
		"""

		
		def _ros_publish_data(self,action="", v_action = "", ispy_action=False):
			'''
			public ros data on child-robot interaction
			'''

			self.curr_robot_action = action
	
			msg = iSpyChildRobotInteraction()
			
			# add header
			msg.header = Header()
			msg.header.stamp = rospy.Time.now()


			# Number of times hint button appears
			# msg.numHintAppear = self.numHintAppear

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

			#msg.numFinishedObjectsForTask[1] = 

			msg.numTotalAttemptsForTask = [self.child_states.total_num_trials,self.child_states.numChildAttemptsCurrTask]

 

			msg.numChildClickCancelForTurn = self.child_click_cancel_num 

			#msg.numHintButtonPressedForTask = self.numHintButtonPressedForTask

			msg.numQAForTurn = [self.child_states.num_robot_questions_asked, self.child_states.pos_answers, 
									self.child_states.neg_answers, self.child_states.other_answers,
									self.child_states.no_answers_attempt1,self.child_states.no_answers_attempt2]
  

			msg.gameStateTrigger = None #self.gameStateTrigger

			msg.currentInteractionState = self.state

			msg.currentGameState = self.game_controller.FSM.state

			msg.robotBehavior = action

			'''
			if self.virtual_action:
				msg.robotVirtualBehavior =  str(self.virtual_action)
			elif self.explore_action:
				msg.robotVirtualBehavior = str(self.explore_action)
			else:
				msg.robotVirtualBehavior = ""
			'''
			msg.robotVirtualBehavior = None

			msg.robotClickedObj = None#self.robot_clickedObj

			msg.clickedRightObject = None#self.clicked_right_obj

			msg.clickedObjName = None#self.clicked_obj_name 

			msg.numTouchAbsenceAlertPerTask = self.child_states.numTouchAbsenceAlertPerTask #######

			msg.objectWordPronounced = self.child_states.objectWordPronounced



			msg.ispyAction = ["", "", "", "", ""]
			if ispy_action == True:
				msg.ispyAction  = [str(self.game_controller.isDragging), str(self.game_controller.pointerClick), str(self.game_controller.onPinch), str(self.game_controller.isScalingUp), str(self.game_controller.isScalingDown)]


			msg.maxElapsedTime = self.elapsed
			##############

			self.ros_node_mgr.pub_child_only_interaction.publish(msg)
		