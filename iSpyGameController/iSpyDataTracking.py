import pandas as pd
from datetime import datetime
import threading
import time
import os

from GameUtils.GlobalSettings import iSpyRobotInteractionStates as ris


#from unity_game_msgs.msg import iSpyAction
CSV_PATH = "ispy_data_files/"



class iSpyDataTracking:
	def __init__(self,childRobotFSM,ros_node_mgr,participant_id, experimenter, session_number):

		self.ros_node_mgr = ros_node_mgr
		# create a pandas dataframe to store all interaction data based on timestamps
		self.game_start_time = None
		self.child_robot_FSM = childRobotFSM

		if not os.path.isdir("ispy_data_files/"): # check exitence of folders
			os.makedirs("ispy_data_files/")

		self._initialize_csvs(participant_id, experimenter, session_number)

		self.ros_node_mgr.start_child_robot_interaction_pub_sub(self.on_child_robot_interaction_data_received)
		# create ros subscribers to published data
		#self.sub_child_robot_interaction = rospy.Subscriber()

		
		
	def _initialize_csvs(self,participant_id, experimenter, session_number):

		import datetime

		now = datetime.datetime.now()
		date = now.isoformat()

		# self.ispy_action_log_csv = open(CSV_PATH + "ispy_action_log.csv","a") 
		# self.ispy_action_log_csv.write(','.join(['elapsedTime','localTime', 'isScalingUpDown',
		# 	'pointerClick','isDragging','onPinch','clickedObjectName']))

		self.child_robot_interaction_csv = open(CSV_PATH+"interaction_log_"+participant_id+"_"+experimenter+"_"+session_number+"_"+date+".csv","a") 
		

		self.child_robot_interaction_csv.write(','.join([
			'elapsedTimeFromGameStart','currentLocalTime',

			'gameTask','vocab', 'taskStartTime','taskEndTime', 'taskDuration',

			'taskTurnIndex', 'whoseTurn', 'robotRole', 

			'turnStartTime','turnEndTime','turnDuration',

			'numCollectedObjectsForTask', 'numChildCollectedObjectsForTask', # task related
			
			'numTotalAttemptsForTask','numChildTotalAttemptsForTask', # task related 

			'numChildClickCancelForTurn', # turn related

			'numQsAskeddForTask','numPositiveAnswerForTask','numNegativeAnswerForTask','numOtherAnswerForTask','numNoAnswerAttempt1ForTask', 'numNoAnswerAttempt2ForTask',# turn related

			'gameStateTrigger','currentInteractionState','currentGameState',

			'robotPhysicalBehavior', 'robotVirtualBehavior',

			'robotClickedObj','clickedRightObject','clickedObjName',

			'numTouchAbsenceAlertPerTask','objectWordPronounced' ,

			'isDraggin', 'pointerClick','onPinch','ScalingUp','isScalingDown'
			
			
			])+'\n')

		# self.interaction_turn_csv = open(CSV_PATH+"interaction_turn_summary_"+participant_id+"_"+experimenter+"_"+session_number_"+date+".csv","a") 
		# elf.interaction_turn_csv.write(','.join(['turn_index','elapsedTimeFromGameStart','localTime',
		# 	'turnStartTime','turnEndTime', 
		# 	'gameTask','vocab', 'whoseTurn', 
		# 	'turn_length', 'robotRole', 'clickedRightObject','clickedObjName',
		# 	'numFinishedObjects', 
		# 	'totalNumQsAsked','totalNumQsAnswered',
		# 	'numQsAskedArr','numQsAnsweredArr',

		# 	'numRobotOfferHelp' ,'numChildAcceptHelp', 
		# 	'numRobotAskHelp', 'numChildOfferHelp',
		# 	'timeOnScreenDragging'
		# 	])+'\n')

	def start_stopwatch(self): #
		self.game_start_time = datetime.now()

	

	def on_child_robot_interaction_data_received(self,msg):
		'''
		callback function. called by ros node manager when child-robot interaction data are received
		write the data to csv file
		'''
		
		# update the ispy action data frame


		elapsedTime = str(datetime.now() - self.game_start_time) if self.game_start_time else ""
		content = ','.join(map(str,[
			elapsedTime,str(datetime.now()),

			msg.gameTask, msg.taskVocab,  msg.taskStartTime, msg.taskEndTime, msg.taskDuration,  # task related 

			msg.taskTurnIndex, msg.whoseTurn, msg.robotRole, 

			msg.turnStartTime, msg.turnEndTime, msg.turnDuration,

			msg.numFinishedObjectsForTask[0], msg.numFinishedObjectsForTask[1], # turn related 

			msg.numTotalAttemptsForTask[0],msg.numTotalAttemptsForTask[1],

			msg.numChildClickCancelForTurn,

			msg.numQAForTurn[0], msg.numQAForTurn[1], msg.numQAForTurn[2], 

			msg.numQAForTurn[3], msg.numQAForTurn[4],  msg.numQAForTurn[5],

			msg.gameStateTrigger, msg.currentInteractionState, msg.currentGameState,

			msg.robotBehavior, msg.robotVirtualBehavior,

			msg.robotClickedObj, msg.clickedRightObject, msg.clickedObjName, 

			msg.numTouchAbsenceAlertPerTask, msg.objectWordPronounced,

			msg.ispyAction[0], msg.ispyAction[1], msg.ispyAction[2], msg.ispyAction[3], msg.ispyAction[4]
			
			]))

		self.child_robot_interaction_csv.write(content+'\n')


	# def on_interaction_turn_summary_data_received(self,msg):
	# 	'''
	# 	callback function. updated after each turn (either child or robot)
	# 	'''
		
	# 	elapsedTime = str(datetime.now() - self.game_start_time)
	# 	content = ','.join(map(str,[ msg.turnIndex, elapsedTime,str(datetime.now()),
	# 		msg.turnStartTime, msg.turnEndTime,
	# 		msg.gameTask, msg.taskVocab, msg.whoseTurn, 
	# 		msg.turnLength, msg.robotRole, msg.clickedRightObject, msg.clickedObjName,
	# 		msg.numFinishedObjects,
	# 		msg.totalNumQsAsked,msg.totalNumQsAnswered,
	# 		msg.numQsAskedArr,msg.numQsAnsweredArr,

	# 		msg.numRobotOfferHelp,msg.numChildAcceptHelp, 
	# 		msg.numRobotAskHelp, msg.numChildOfferHelp,

	# 		msg.timeOnScreenDragging


	# 		]))


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

		#self.ispy_action_log_csv.write(','.join(map(str,[elapsedTime,str(datetime.now()), isScalingUpDown, data.pointerClick,data.isDragging,data.onPinch,object_name,data.speakingStage ])))
	
	
		

	

	def on_ispy_child_learning_received(self,data):
		pass
