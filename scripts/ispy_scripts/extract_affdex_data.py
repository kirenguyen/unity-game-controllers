# this script extracts affdex data from rostopic /usb_cam

import rospy
from std_msgs.msg import String
from affdex_msgs.msg import AffdexFrameInfo
import time
import datetime
from unity_game_msgs.msg import iSpyCommand
from unity_game_msgs.msg import iSpyChildRobotInteraction

import subprocess
import sys
import datetime

class AffdexAnalysis:

	def __init__(self,rosbag_name):

		self.inter_data_frame = 0
		rospy.init_node('node_name')
		rospy.Subscriber("affdex_data", AffdexFrameInfo, self.on_affdex_data_received)
		#self.csv_file_prefix = "affdex_log_"+p_id+"_"+experimenter+"_"+game_round+"_"
		self.csv_file_name = "affdex-outputs/"+rosbag_name.replace('bag','csv')
		rospy.Subscriber("/ispy_cmd_topic",iSpyCommand,self.on_game_start)
		rospy.Subscriber("/data_child_robot_interaction", iSpyChildRobotInteraction, self.on_inter_data)
		
		rospy.Subscriber("/ispy_transition_state_topic", String, self.on_transition_state)   
    
		self.start_time = time.localtime(time.time())

		print(self.start_time)

		self.file = open(self.csv_file_name, 'w')

		self.first_line = True
		
		self.frame_number = 0
		self.game_start_time = None
		self.time_diff = 0

		self.transition_state_start_time = None

		subprocess.call("./tega_cam_affect_analysis", shell=True)

		rospy.spin()



	def on_transition_state(self,data):
		if data.data == "TOPLEFT_BUTTON_PRESSED" and not self.transition_state_start_time:
			print("transition state start  time is set....")
			self.transition_state_start_time = time.time()
			self.game_start_time = time.time()
			print(self.transition_state_start_time)
		


	def on_inter_data(self,data):
		self.inter_data_frame += 1

	def on_game_start(self,data):
		if self.time_diff == 0:
			nanosec_offset = float(data.header.stamp.nsecs) /(10**9)

			self.time_diff = time.time() - (data.header.stamp.secs + nanosec_offset )
		
		

	def add_file_heading(self,data):
		print("affdex recording starts")
		heading = "frame_number, orig_local_time, orig_local_time2, local_time, game_elapsed_time, time_diff, inter_data_frame_number"
		emotion_heading = ','.join([ "emotion-"+str(i) for i in range(1,len(data.emotions)+1,1)])
		expression_heading = ','.join([ "expression-"+str(i) for i in range(1,len(data.expressions)+1,1)])
		measurement_heading = ','.join([ "measurement-"+str(i) for i in range(1,len(data.measurements)+1,1)])

		self.file.write(','.join([heading,emotion_heading,expression_heading,measurement_heading])+'\n')
	

	def reformat_data(self,data):
		self.frame_number = self.frame_number + 1

		if self.first_line == True:
			self.add_file_heading(data)
			
			self.first_line = False

		nanosec_offset = float(data.header.stamp.nsecs) /(10**9)
		
		
		elapsed_time =  data.header.stamp.secs + nanosec_offset - self.game_start_time if self.game_start_time else ""
		#print("elapse..{}".format(elapsed_time))
		#print(time.localtime(time.time() - self.time_diff))
		
		orig_local_time = time.strftime("%Y-%m-%d-%H:%M:%S",time.localtime(time.time() - self.time_diff)) if self.time_diff != 0 else ""
		orig_time2 = str(datetime.datetime.now() - datetime.timedelta(seconds=self.time_diff)) if self.time_diff !=0 else ""
		emotions_str = ','.join([str(i) for i in data.emotions])	
		expressions_str = ','.join([str(i) for i in data.expressions])
		measurements_str = ','.join([str(i) for i in data.measurements])
		
		return [str(self.frame_number), orig_local_time, orig_time2, time.strftime("%Y-%m-%d-%H:%M:%S",time.localtime()), str(elapsed_time), str(self.time_diff), str(self.inter_data_frame), emotions_str,expressions_str,measurements_str] 


	def on_affdex_data_received(self,data):
		''''
		callback function when affdex data are received
		'''
		
		self.write_data_to_file(data)
		

	#def write_data_to_df(self,data):


	def write_data_to_file(self,data):
		'''
		write affdex data to file
		'''
		outs = self.reformat_data(data)
		
		line = ','.join(outs)

		self.file.write(line+'\n')

if __name__ == "__main__":
	rosbag_name = sys.argv[1]
	print("rosbag name: {}".format(rosbag_name))
	aff = AffdexAnalysis(rosbag_name)