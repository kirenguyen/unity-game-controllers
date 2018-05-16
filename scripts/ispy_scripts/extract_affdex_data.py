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

	def __init__(self,rosbag_name,session_number,interaction_log):

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
		self.interaction_log = True if "true" in interaction_log else False

		self.transition_state_start_time = None

		participant_id = rosbag_name.split('_')[0]
		experimenter= rosbag_name.split('_')[1]
		date_info = rosbag_name.split('__')[1].split('.')[0].replace("-",":")
		if self.interaction_log:
			print("interaction log is true")
			self._initialize_csvs(participant_id, experimenter, session_number,date_info)
		else:
			print("interaction log is false")

		subprocess.call("./tega_cam_affect_analysis", shell=True)

		rospy.spin()



	def on_transition_state(self,data):
		if data.data == "TOPLEFT_BUTTON_PRESSED" and not self.transition_state_start_time:
			print("transition state start  time is set....")
			self.transition_state_start_time = time.time()
			self.game_start_time = time.time()
			print(self.transition_state_start_time)
		

	def _initialize_csvs(self,participant_id, experimenter, session_number,date_info):
		import datetime

		now = datetime.datetime.now()
		date = now.isoformat()

		# self.ispy_action_log_csv = open(CSV_PATH + "ispy_action_log.csv","a") 
		# self.ispy_action_log_csv.write(','.join(['elapsedTime','localTime', 'isScalingUpDown',
		# 	'pointerClick','isDragging','onPinch','clickedObjectName']))
		INTERACTION_OUTPUTS ="interaction_outputs/"
		self.child_robot_interaction_csv = open(INTERACTION_OUTPUTS+"interaction_log_"+participant_id+"_"+experimenter+ ":"+date_info+"_"+session_number+"_"+date+".csv","a") 
		

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

	def on_child_robot_interaction_data_received(self,msg):
		'''
		callback function. called by ros node manager when child-robot interaction data are received
		write the data to csv file
		'''
		print("msg!!!")
		print(msg)

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


	def on_inter_data(self,data):
		self.inter_data_frame += 1
		print("on inter data")
		if self.interaction_log:
			self.on_child_robot_interaction_data_received(data)

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
	print(sys.argv)
	rosbag_name = sys.argv[1]
	session_number = sys.argv[2]
	interaction_log = sys.argv[3]
	print(interaction_log)
	print("rosbag name: {}".format(rosbag_name))
	aff = AffdexAnalysis(rosbag_name,session_number,interaction_log)