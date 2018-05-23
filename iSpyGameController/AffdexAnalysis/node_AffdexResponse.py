#!/usr/bin/env python
import time
from random import randint



from std_msgs.msg import Bool # for child_attention topic
from std_msgs.msg import Header # standard ROS msg header
from std_msgs.msg import String
from std_msgs.msg import Int32
import os
import datetime



import shelve # for database mangagement

last_motion = 0
time_since_last_motion = 0
DELAY = 25

# def send_motion_message(motion):
#     """ Publish TegaAction do motion message """
#     msg = TegaAction()
#     msg.do_motion = True
#     msg.motion = motion
#     pub_response.publish(msg)

class AffdexAnalysis:

	def __init__(self,gameController,ros_node_mgr,p_id, experimenter, session_number):
		self.gameController = gameController
		self.ros_node_mgr = ros_node_mgr
		self.ros_node_mgr.start_affdex_listener(self.on_affdex_data_received)
		if not os.path.isdir("ispy_data_files/"): # check exitence of folders
			os.makedirs("ispy_data_files/")

		if not os.path.isdir("ispy_data_files/affdex_data/"): # check exitence of folders
			os.makedirs("ispy_data_files/affdex_data/") 

		now = datetime.datetime.now()
		date = now.isoformat()

		self.csv_file_prefix = "ispy_data_files/affdex_data/affdex_log_"+p_id+"_"+experimenter+"_"+session_number+"_"+date+"_"
		self.start_time = '-'.join([str(i) for i in time.localtime()])
		self.csv_file_name = self.csv_file_prefix+self.start_time+".csv"
		self.file = open(self.csv_file_name, 'w')

		self.first_line = True
		self.start_time = ""
		self.frame_number = 0




	def add_file_heading(self,data):
		print("affdex recording starts")
		heading = "frame_number, elapsed_time (sec), localtime, gameTask, taskTurnIndex, currentRobotAction"
		emotion_heading = ','.join([ "emotion-"+str(i) for i in range(1,len(data.emotions)+1,1)])
		expression_heading = ','.join([ "expression-"+str(i) for i in range(1,len(data.expressions)+1,1)])
		measurement_heading = ','.join([ "measurement-"+str(i) for i in range(1,len(data.measurements)+1,1)])

		self.file.write(','.join([heading,emotion_heading,expression_heading,measurement_heading])+'\n')
	

	def on_affdex_data_received(self,data):
		''''
		callback function when affdex data are received
		'''

		self.write_data_to_file(data)
		

	def write_data_to_file(self,data):
		'''
		write affdex data to file
		'''
		self.frame_number = self.frame_number + 1

		if self.first_line == True:
			self.add_file_heading(data)
			self.start_time = time.time()
			self.first_line = False

		timestamp = '-'.join([str(i) for i in time.localtime()])
		elapsed_time = time.time() - self.start_time
		gameTask = self.gameController.task_controller.current_task_index
		taskTurnIndex = self.gameController.interaction.current_task_turn_index

		curr_robot_action = self.gameController.interaction.curr_robot_action
		
			
		emotions_str = ','.join([str(i) for i in data.emotions])	
		expressions_str = ','.join([str(i) for i in data.expressions])
		measurements_str = ','.join([str(i) for i in data.measurements])
		
		line = ','.join([str(self.frame_number), str(elapsed_time), timestamp, str(gameTask),str(taskTurnIndex), curr_robot_action, emotions_str,expressions_str,measurements_str]) 

		
		self.file.write(line+'\n')
		
