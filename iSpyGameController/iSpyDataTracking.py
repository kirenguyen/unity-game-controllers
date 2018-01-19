import pandas as pd
from datetime import datetime
import threading
import time

from GameUtils.GlobalSettings import iSpyRobotInteractionStates as ris


#from unity_game_msgs.msg import iSpyAction
CSV_PATH = "ispy_data_files/"



class iSpyDataTracking:
	def __init__(self,childRobotFSM,ros_node_mgr,participant_id, experimenter):

		self.ros_node_mgr = ros_node_mgr
		# create a pandas dataframe to store all interaction data based on timestamps
		self.game_start_time = None
		self.child_robot_FSM = childRobotFSM

		self._initialize_csv(participant_id, experimenter)

		self.ros_node_mgr.start_child_robot_interaction_pub_sub(self.on_child_robot_interaction_data_received)
		# create ros subscribers to published data
		#self.sub_child_robot_interaction = rospy.Subscriber()
		
	def _initialize_csv(self,participant_id, experimenter):

		import datetime

		now = datetime.datetime.now()
		date = now.strftime("%Y-%m-%d")

		self.ispy_action_log_csv = open(CSV_PATH + "ispy_action_log.csv","a") 
		self.ispy_action_log_csv.write(','.join(['elapsedTime','localTime', 'isScalingUpDown',
			'pointerClick','isDragging','onPinch','clickedObjectName']))

		self.child_robot_interaction_csv = open(CSV_PATH+"interaction_log_"+participant_id+"_"+experimenter+"_"+date+".csv","a") 
		self.child_robot_interaction_csv.write(','.join(['elapsedTime','localTime', 'whoseTurn',
			'robotRole','robotBehavior','robotClickedObj','clickedRightObject','clickedObjName',
			'gameTask','vocab','numFinishedObjects', 'numQsAsked','numQsAnswered','numChildAttemptsPerGame'
			'numChildCorrectAttemptsPerGame','gameStateTrigger', 
			'numRobotYNQuestion', 'numRobotYNQuestionAnswered',
			'numRobotOpenQuestion','numRobotOpenQuestionAnswered','numTouchAbsenceAlertPerTask',
			'current_turn_length','currentInteractionState',
			'childCurrAttemptCorrectness', 'childPrevAttemptCorrectness',
			'objectWordPronounced' , 'numChildAttemptsCurrTask',
			'numChildCorrectAttemptsCurrTask', 'numRobotOfferHelp' ,
			'numChildAcceptHelp', 'numRobotAskHelp',
			'numChildOfferHelp'
			])+'\n')

	def start_stopwatch(self): #
		self.game_start_time = datetime.now()


	def on_child_robot_interaction_data_received(self,msg):
		'''
		callback function. called by ros node manager when child-robot interaction data are received
		write the data to csv file
		'''
		
		# update the ispy action data frame
		elapsedTime = str(datetime.now() - self.game_start_time)
		content = ','.join(map(str,[elapsedTime,str(datetime.now()),msg.whoseTurn, 
			msg.robotRole, msg.robotBehavior, msg.robotClickedObj, msg.clickedRightObject, msg.clickedRightObject, 
			msg.clickedObjName, msg.gameTask, msg.taskVocab, msg.numFinishedObjects, 
			msg.numRobotQuestionsAsked, msg.numRobotQuestionsAnswered, msg.numChildAttemptsPerGame, 
			msg.numChildCorrectAttemptsPerGame, 
			msg.gameStateTrigger, 
			msg.numRobotYNQuestion, msg.numRobotYNQuestionAnswered,
			msg.numRobotOpenQuestion, msg.numRobotOpenQuestionAnswered, msg.numTouchAbsenceAlertPerTask,
			msg.current_turn_length, msg.currentInteractionState, 
			msg.childCurrAttemptCorrectness, msg.childPrevAttemptCorrectness,
			msg.objectWordPronounced , msg.numChildAttemptsCurrTask,
			msg.numChildCorrectAttemptsCurrTask, msg.numRobotOfferHelp ,
			msg.numChildAcceptHelp, msg.numRobotAskHelp,
			msg.numChildOfferHelp
			]))

		self.child_robot_interaction_csv.write(content+'\n')


	def on_ispy_action_received(self,data):
		'''
		receive iSpy interaction log data from iSpyGameFSM. 
		called everytime when an interaction action from ispy game is received
		'''
		
		self.stop_thread_flag  = True # set stop thread to be true to stop elapsed time counting thread 

		#Removes object position from ispy_action_msg
		object_name = ""
		for letter in data.clickedObjectName:
			if letter == "-":
				break
			object_name += letter

		# update the ispy action data frame
		elapsedTime = str(datetime.now() - self.game_start_time)
		isScalingUpDown = any(n == True for n in [data.isScalingUp,data.isScalingDown])

		self.ispy_action_log_csv.write(','.join(map(str,[elapsedTime,str(datetime.now()), isScalingUpDown, data.pointerClick,data.isDragging,data.onPinch,object_name,data.speakingStage ])))
	
	
		

	

	def on_ispy_child_learning_received(self,data):
		pass
