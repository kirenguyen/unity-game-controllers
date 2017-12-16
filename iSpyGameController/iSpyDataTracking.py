import pandas as pd
from datetime import datetime
import threading
import time

from GameUtils.GlobalSettings import iSpyRobotInteractionStates as ris
#from unity_game_msgs.msg import iSpyAction

class iSpyDataTracking:
	def __init__(self,childRobotFSM):
		# create a pandas dataframe to store all interaction data based on timestamps
		self.game_start_time = datetime.now()
		self.ispy_action_log_csv = open("ispy_action_log.csv","a") 
		self.ispy_action_log_csv.write(','.join(['elapsedTime','localTime', 'isScalingUpDown','pointerClick','isDragging','onPinch','clickedObjectName']))
		self.child_robot_FSM = childRobotFSM
		
	def save_data_to_csv(self):
		pass

	def on_ispy_action_received(self,data):
		'''
		receive iSpy interaction log data from iSpyGameFSM
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
	
	def on_start_tracking_child_interact(self):
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
					#print("************threading..  Exiting loop.")
					break
				if delta_time.total_seconds() > 20:
					self.child_robot_FSM.on_no_ispy_action_alert(2)
					break
				elif delta_time.total_seconds() > 8 and alerted_once == False :
					# 10 secs have passed without receiving any tablet interaction input from the cihld
					self.child_robot_FSM.on_no_ispy_action_alert(1)
					alerted_once = True
					
				
			#print("=====Thread signing off")
		if self.child_robot_FSM.state == ris.CHILD_TURN: 
			time.sleep(1)
			self.stop_thread_flag = False
			t = threading.Thread(target=elapsed_time_alert, args=(lambda: self.stop_thread_flag,))
			t.start()
		else:
			self.stop_thread_flag = True
		


	def on_ispy_child_learning_received(self,data):
		pass
