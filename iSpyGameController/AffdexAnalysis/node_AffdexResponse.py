#!/usr/bin/env python
import time
from random import randint



from std_msgs.msg import Bool # for child_attention topic
from std_msgs.msg import Header # standard ROS msg header
from std_msgs.msg import String
from std_msgs.msg import Int32



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

	def __init__(self,ros_node_mgr):
		self.ros_node_mgr = ros_node_mgr
		self.ros_node_mgr.start_affdex_listener(self.on_affdex_data_received)
		self.csv_file_prefix = "scripts/affdex_data/affdex-csv-file"
		self.start_time = '-'.join([str(i) for i in time.localtime()])
		self.csv_file_name = self.csv_file_prefix+"-"+self.start_time+".csv"
		self.file = open(self.csv_file_name, 'w')

		self.first_line = True
		self.start_time = ""
		self.frame_number = 0


	def add_file_heading(self,data):
		heading = "frame_number, elapsed_time (sec), timestamp"
		emotion_heading = ','.join([ "emotion-"+str(i) for i in range(1,len(data.emotions)+1,1)])
		expression_heading = ','.join([ "expression-"+str(i) for i in range(1,len(data.expressions)+1,1)])
		measurement_heading = ','.join([ "measurement-"+str(i) for i in range(1,len(data.measurements)+1,1)])

		self.file.write(','.join([heading,emotion_heading,expression_heading,measurement_heading]))
		print(len(','.join([heading,emotion_heading,expression_heading,measurement_heading]).split(",")))

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



		print("elapsed time")
		print(elapsed_time)
			
		emotions_str = ','.join([str(i) for i in data.emotions])	
		expressions_str = ','.join([str(i) for i in data.expressions])
		measurements_str = ','.join([str(i) for i in data.measurements])
		
		line = ','.join([str(self.frame_number), str(elapsed_time), timestamp,emotions_str,expressions_str,measurements_str]) 

		print("line...")
		print(len(line.split(',')))

		self.file.write(line+'\n')
		
