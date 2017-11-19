#!/usr/bin/env python
import time
from random import randint


from affdex_msgs.msg import AffdexFrameInfo # please download affdex ros msg from PRG's git
from affdex_msgs.msg import Vec2 # please download affdex ros msg from PRG's git

from std_msgs.msg import Bool # for child_attention topic
from std_msgs.msg import Header # standard ROS msg header
from std_msgs.msg import String
from std_msgs.msg import Int32

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
    
	def onStateReceived(self,data):
		global last_motion
		global time_since_last_motion
	
		if data.doing_motion == True:
			last_motion = data.header.seq
			time_since_last_motion = 0
		else:
			time_since_last_motion = data.header.seq - last_motion

	def onAffdexDataReceived(data):	
		if time_since_last_motion > DELAY:
			if data.emotions[0] > 50: # Happy
				pass
			elif data.emotions[2] > 50: # Disgust
				pass
			elif data.emotions[6] > 10: # Sad
				pass
			elif data.emotions[7] > 50: # Surprised
				pass
			elif data.expressions[4] > 50: # Eye closure
				pass
			elif abs(data.measurements[1]) > 20 or abs(data.measurements[2]) > 12: # Looking away
				spass

	def write_to_csv(self):
		pass
#node = rospy.init_node('node_AffdexAnalysis', anonymous=True)
#sub_affdex = rospy.Subscriber('affdex_data', AffdexFrameInfo, onAffdexDataReceived) # affdex data

#rospy.spin()
