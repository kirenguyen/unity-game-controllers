import time
import json

#from ..BaseClassFSM import BaseClassFSM


#from transitions import Machine
from transitions.extensions import HierarchicalMachine as Machine

from ..RoleSwitchingPrj.ChildStates import ChildStates

# from GameUtils import Curriculum
from GameUtils import GlobalSettings
from GameUtils.GlobalSettings import iSpyGameStates as gs
from GameUtils.GlobalSettings import iSpyRobotInteractionStates as ris


if GlobalSettings.USE_ROS:
	from std_msgs.msg import Header  # standard ROS msg header
	from std_msgs.msg import String
	from unity_game_msgs.msg import iSpyCommand
	from unity_game_msgs.msg import iSpyAction

import random

from unity_game_msgs.msg import iSpyChildRobotInteraction
import threading
from datetime import datetime, timedelta

import os
import timestring
import rospy


ROOT_TEGA_SPEECH_FOLDER = 'roleswitching18/'



#class ChildOnlyFSM(BaseClassFSM):
class ChildOnlyFSM:
		'''
		child only interaction FSM
		It communicates with iSpyGameFSM and ChildStates
		'''

		def __init__(self,ros_node_mgr,task_controller,game_controller,participant_id,game_round):
			# use hierachical FSM here. The python package can be found here: https://github.com/pytransitions/transitions
			self.states = [{'name': ris.CHILD_TURN, 
                                    'children':[ris.CHILD_TURN]}]
			self.transitions = [
				{'trigger': ris.Triggers.CHILD_TURN_DONE, 'source': ris.CHILD_TURN, 'dest': ris.CHILD_TURN },
				# when the child does not interact with the tablet
				{'trigger': ris.Triggers.NO_INTERACTION_ALERT, 'source': ris.CHILD_TURN, 'dest':ris.CHILD_TURN+"_"+ris.NO_INTERACTION_1 }
                            ]
			
			self.state = ris.CHILD_TURN     

			self.state_machine = Machine(self, states=self.states, transitions=self.transitions,
									 initial=ris.CHILD_TURN)

			self.ros_node_mgr = ros_node_mgr

			self.task_controller = task_controller

			self.game_controller = game_controller

			# load assigned condition the participant is in
			subj_assign_dict = json.loads(open("iSpyGameController/res/participant_assignment.json").read())
			self.subj_cond = subj_assign_dict[participant_id]

			self.child_states = ChildStates(participant_id,self.subj_cond,task_controller)

			self.explore_action = ""

			self.role = "novice" #default to novice at the beginning (backup)
			
			self.current_task_turn_index = 0 # for the current task, current turn index

			self.turn_start_time = None

			self.turn_end_time = None

			self.turn_duration = ""

			self.child_click_cancel_num = 0
                     
			self.numHintButtonPressedForTask = 0

			self.reset_elapsed_time = False;

			self.elapsed = ""
                     
		#def on_enter_childTURN(self):
			#super().on_enter_childTURN()
		def on_enter_childTURN(self):
			self.turn_start_time = datetime.now()
			self.turn_end_time = None
			self.turn_duration = ""
			self.current_task_turn_index += 1
			
			self.explore_action = ""
			self.virtual_action = ""
			threading.Timer(10.0, self.on_child_max_elapsed_time).start()
			self.reset_elapsed_time = False

		def start_task_end_behavior(self, action_number):
			pass              
              
		def react(self,gameStateTrigger,  clicked_obj_name = ""):
			pass
            
		#def turn_taking(self,max_time=False):
			#super().turn_taking()
		def turn_taking(self,max_time=False):
			self.ros_node_mgr.send_ispy_cmd(iSpyCommand.WHOSE_TURN, {"whose_turn":self.state})
			# update the number of available objects for child's learning states
			self.child_states.set_num_available_objs(self.task_controller.get_num_available_target_objs())
				
		def start_tracking_child_interaction(self):
			pass
			
		def stop_tracking_child_interaction(self):
			pass
	
		def _ros_publish_data(self,action="", v_action = "", ispy_action=False):
			pass

		def reset_turn_taking(self):
			pass

		def get_robot_general_response(self):
			pass
                    
		def on_child_max_elapsed_time(self):
			pass
                
