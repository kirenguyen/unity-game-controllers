import pandas as pd
from datetime import datetime
import threading
import time

from GameUtils.GlobalSettings import iSpyRobotInteractionStates as ris


#from unity_game_msgs.msg import iSpyAction
CSV_PATH = "ispy_data_files/"



class iSpyDataTracking:
	def __init__(self,childRobotFSM,ros_node_mgr):

		self.ros_node_mgr = ros_node_mgr
		# create a pandas dataframe to store all interaction data based on timestamps
		self.game_start_time = datetime.now()
		self.child_robot_FSM = childRobotFSM

		self._initialize_csv()

		self.ros_node_mgr.start_child_robot_interaction_pub_sub(self.on_child_robot_interaction_data_received)
		# create ros subscribers to published data
		#self.sub_child_robot_interaction = rospy.Subscriber()
		
	def _initialize_csv(self):
		self.ispy_action_log_csv = open(CSV_PATH + "ispy_action_log.csv","a") 
		self.ispy_action_log_csv.write(','.join(['elapsedTime','localTime', 'isScalingUpDown',
			'pointerClick','isDragging','onPinch','clickedObjectName']))

		self.child_robot_interaction_csv = open(CSV_PATH+"ispy_interaction_log.csv","a") 
		self.child_robot_interaction_csv.write(','.join(['elapsedTime','localTime', 'whoseTurn',
			'robotRole','robotBehavior','robotClickedObj','clickedRightObject','clickedObjName',
			'gameTask','vocab','numFinishedObjects', 'numQsAsked','numQsAnswered','numChildAttempts'
			'numChildCorrectAttempts','gameStateTrigger']))


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
			msg.numQsAsked, msg.numQsAnswered, msg.numChildAttempts, msg.numChildCorrectAttempts,
			msg.gameStateTrigger]))

		self.child_robot_interaction_csv.write(content)


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
	
	def on_start_tracking_child_interaction(self):
		'''
		callback function on tracking child's engagment with the tablet. 
		called by ChildRobotInteraction
		'''
		

		# start a new thread to keep track of elapsed time
		def elapsed_time_alert(stop):
			#print("!!!!!======....start thread.....")
			start_time = datetime.now()
			alerted_once = False
			while True:
				delta_time = datetime.now() - start_time
				if stop():
					break
				if delta_time.total_seconds() > 20:
					self.child_robot_FSM.on_no_ispy_action_alert(2)
					break
				elif delta_time.total_seconds() > 10 and alerted_once == False :
					# 10 secs have passed without receiving any tablet interaction input from the cihld
					self.child_robot_FSM.on_no_ispy_action_alert(1)
					alerted_once = True		
				
		if self.child_robot_FSM.state == ris.CHILD_TURN: 
			time.sleep(1)
			self.stop_thread_flag = False
			t = threading.Thread(target=elapsed_time_alert, args=(lambda: self.stop_thread_flag,))
			t.start()
		else:
			self.stop_thread_flag = True
		
	def on_stop_tracking_child_interaction(self):
		self.stop_thread_flag = True

	def on_ispy_child_learning_received(self,data):
		pass
