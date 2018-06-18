import pandas as pd
from datetime import datetime
import threading
import time
import os

from GameUtils.GlobalSettings import iSpyRobotInteractionStates as ris


#from unity_game_msgs.msg import iSpyAction
CSV_PATH = "ispy_data_files/"



class iSpyDataTracking:
	def __init__(self,childRobotFSM,ros_node_mgr,participant_id, experimenter, session_number, is_child_robot):

		if is_child_robot == "True": is_child_robot = True
		if is_child_robot == "False": is_child_robot = False

		self.ros_node_mgr = ros_node_mgr
		# create a pandas dataframe to store all interaction data based on timestamps
		self.game_start_time = None
		self.child_robot_FSM = childRobotFSM
		self.session_number = session_number
		self.is_child_robot = is_child_robot

		if not os.path.isdir("ispy_data_files/"): # check exitence of folders
			os.makedirs("ispy_data_files/")

		if self.session_number != "practice" and is_child_robot:
			self._initialize_csvs_CR(participant_id, experimenter, session_number)
		if self.session_number != "practice" and not is_child_robot:
			self._initialize_csvs_CO(participant_id, experimenter, session_number)

		if is_child_robot:
			self.ros_node_mgr.start_child_robot_interaction_pub_sub(self.on_child_robot_interaction_data_received)
		else: 
			self.ros_node_mgr.start_child_only_interaction_pub_sub(self.on_child_only_interaction_data_recieved)
		
	def _initialize_csvs_CR(self,participant_id, experimenter, session_number):

		print('55555555555555555555555555555555555555555555')
		import datetime

		now = datetime.datetime.now()
		date = now.isoformat()

		self.child_robot_interaction_csv = open(CSV_PATH+"interaction_log_"+participant_id+"_"+experimenter+"_"+session_number+"_"+date+"_childrobot_"+".csv","a") 
		
		self.child_robot_interaction_csv.write("TESTING TESTING")

		self.child_robot_interaction_csv.write(','.join([
			'elapsedTimeFromGameStart','currentLocalTime',

			'gameTask','vocab', 'taskStartTime','taskEndTime', 'taskDuration',

			'taskTurnIndex', 'whoseTurn', 'robotRole', 

			'turnStartTime','turnEndTime','turnDuration',

			'numCollectedObjectsForTask', 'numChildCollectedObjectsForTask', # task related
			
			'numTotalAttemptsForTask','numChildTotalAttemptsForTask', # task related 

			'numChildClickCancelForTurn', 'numHintButtonPressedForTask', # turn related

			'numQsAskeddForTask','numPositiveAnswerForTask','numNegativeAnswerForTask','numOtherAnswerForTask','numNoAnswerAttempt1ForTask',# turn related

			'gameStateTrigger','currentInteractionState','currentGameState',

			'robotPhysicalBehavior', 'robotVirtualBehavior',

			'robotClickedObj','clickedRightObject','clickedObjName',

			'numTouchAbsenceAlertPerTask','objectWordPronounced' ,

			'isDraggin', 'pointerClick','onPinch','isScalingUp','isScalingDown',

			'maxElapsedTimeReached'
			
			
			])+'\n')

	def _initialize_csvs_CO(self,participant_id, experimenter, session_number):

		import datetime

		now = datetime.datetime.now()
		date = now.isoformat()

		self.child_only_interaction_csv = open(CSV_PATH+"interaction_log_"+participant_id+"_"+experimenter+"_"+session_number+"_"+date+"_childonly_"+".csv","a") 
		self.child_only_interaction_csv.write("please write something")

		self.child_only_interaction_csv.write(','.join([
			'elapsedTimeFromGameStart','currentLocalTime',
			'gameTask','vocab', 'taskStartTime','taskEndTime', 'taskDuration',
			'taskTurnIndex', 
			'turnStartTime','turnEndTime','turnDuration',
			'numCollectedObjectsForTask', 'numChildCollectedObjectsForTask', # task related
			'numTotalAttemptsForTask','numChildTotalAttemptsForTask', # task related 
			'numChildClickCancelForTurn',  # turn related
			'gameStateTrigger','currentInteractionState','currentGameState',
			'clickedRightObject','clickedObjName',
			'numTouchAbsenceAlertPerTask','objectWordPronounced' ,
			'isDraggin', 'pointerClick','onPinch','isScalingUp','isScalingDown',
			'maxElapsedTimeReached', 'numHintButtonPressedForTask'
			])+'\n')

	def start_stopwatch(self): #
		self.game_start_time = datetime.now()

	
	def on_child_only_interaction_data_recieved(self,msg):

		print("+++++++++ WRITE TO CSV CO +++++++++++++++++++")
		print("444444444444444444444444444444444444444444444")


		if self.session_number == 'practice': return

		elapsedTime = str(datetime.now() - self.game_start_time) if self.game_start_time else ""
		content = ','.join(map(str,[
			elapsedTime,str(datetime.now()),
			msg.gameTask, msg.taskVocab,  msg.taskStartTime, msg.taskEndTime, msg.taskDuration,  # task related 
			msg.taskTurnIndex,
			msg.turnStartTime, msg.turnEndTime, msg.turnDuration,
			msg.numFinishedObjectsForTask[0], msg.numFinishedObjectsForTask[1], # turn related 
			msg.numTotalAttemptsForTask[0],msg.numTotalAttemptsForTask[1],
			msg.numChildClickCancelForTurn, 
			msg.gameStateTrigger, msg.currentInteractionState, msg.currentGameState,
			msg.clickedRightObject, msg.clickedObjName, 
			msg.numTouchAbsenceAlertPerTask, msg.objectWordPronounced,
			msg.ispyAction[0], msg.ispyAction[1], msg.ispyAction[2], msg.ispyAction[3], msg.ispyAction[4],
			msg.maxElapsedTime, msg.numHintButtonPressedForTask
			]))

		self.child_only_interaction_csv.write(content+'\n')



	def on_child_robot_interaction_data_received(self,msg):
		'''
		callback function. called by ros node manager when child-robot interaction data are received
		write the data to csv file
		'''
		
		print("+++++++++ WRITE TO CSV CR +++++++++++++++++++")

		# update the ispy action data frame
		
		if self.session_number == "practice": return


		elapsedTime = str(datetime.now() - self.game_start_time) if self.game_start_time else ""
		content = ','.join(map(str,[
			elapsedTime,str(datetime.now()),

			msg.gameTask, msg.taskVocab,  msg.taskStartTime, msg.taskEndTime, msg.taskDuration,  # task related 

			msg.taskTurnIndex, msg.whoseTurn, msg.robotRole, 

			msg.turnStartTime, msg.turnEndTime, msg.turnDuration,

			msg.numFinishedObjectsForTask[0], msg.numFinishedObjectsForTask[1], # turn related 

			msg.numTotalAttemptsForTask[0],msg.numTotalAttemptsForTask[1],

			msg.numChildClickCancelForTurn, #msg.numHintButtonPressedForTask, 

			msg.numQAForTurn[0], msg.numQAForTurn[1], msg.numQAForTurn[2], 

			msg.numQAForTurn[3], msg.numQAForTurn[4],  msg.numQAForTurn[5],

			msg.gameStateTrigger, msg.currentInteractionState, msg.currentGameState,

			msg.robotBehavior, msg.robotVirtualBehavior,

			msg.robotClickedObj, msg.clickedRightObject, msg.clickedObjName, 

			msg.numTouchAbsenceAlertPerTask, msg.objectWordPronounced,

			msg.ispyAction[0], msg.ispyAction[1], msg.ispyAction[2], msg.ispyAction[3], msg.ispyAction[4],

			msg.maxElapsedTime 
			
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
		elapsedTime = str(datetime.now() - self.game_start_time) if self.game_start_time else ""
		isScalingUpDown = any(n == True for n in [data.isScalingUp,data.isScalingDown])	

	def on_ispy_child_learning_received(self,data):
		pass
