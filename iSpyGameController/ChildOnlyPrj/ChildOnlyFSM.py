from ..BaseClassFSM import BaseClassFSM
from GameUtils.GlobalSettings import iSpyRobotInteractionStates as ris
import random

class ChildOnlyFSM(BaseClassFSM):
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

			super().__init__(ros_node_mgr,task_controller,game_controller,participant_id,game_round)

		def on_enter_childTURN(self):

			r = random.choice([-1, 1])
			if r > 0: self.ros_node_mgr.send_ispy_cmd(iSpyCommand.BUTTON_DISABLED, {"buttonName": "helpingHintActivate"})
			else: self.ros_node_mgr.send_ispy_cmd(iSpyCommand.BUTTON_DISABLED, {"buttonName": "helpingHintDeactivate"})
			super().on_enter_childTURN()

		def turn_taking(self,max_time=False):
			super().turn_taking()