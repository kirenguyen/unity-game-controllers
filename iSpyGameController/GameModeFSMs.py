from transitions import Machine
from GameUtils.GlobalSettings import iSpyGameStates as gs
from GameUtils.GlobalSettings import iSpyRobotInteractionStates as ris
from .ROSNodeMgr import ROSNodeMgr

# contain FSMs for different modes
# iSpy role switching project uses "alwaysMissionModeFSM"
# curiosity assessment project uses "alwaysExploreModeFSM"
# "CompleteModeFSM" allows players to switch between explore and mission mode easily

BUTTON_DISABLED=31

class BaseGameFSM:
	def start_trigger(self,trigger_string):
		getattr(self, trigger_string)()

	def get_state(self):
		return self.state

class CompleteModeFSM(BaseGameFSM):
	'''
	switch between explore and mission modes
	'''
	def __init__(self):
		self.states = [gs.GAME_START, gs.EXPLORATION_MODE,gs.MISSION_MODE,gs.PRONUNCIATION_PANEL,gs.PRONUNCIATION_RESULT,gs.WORD_DISPLAY]
		self.transitions = [
				{'trigger': gs.Triggers.START_BUTTON_PRESSED, 'source': gs.GAME_START, 'dest': gs.EXPLORATION_MODE},
				{'trigger': gs.Triggers.TOPLEFT_BUTTON_PRESSED, 'source': gs.EXPLORATION_MODE, 'dest': gs.MISSION_MODE},
				{'trigger': gs.Triggers.TOPLEFT_BUTTON_PRESSED, 'source': gs.MISSION_MODE, 'dest': gs.EXPLORATION_MODE},
				{'trigger': gs.Triggers.OBJECT_CLICKED, 'source': gs.MISSION_MODE, 'dest': gs.PRONUNCIATION_PANEL},
				{'trigger': gs.Triggers.TARGET_OBJECT_COLLECTED , 'source': gs.PRONUNCIATION_RESULT, 'dest': gs.MISSION_MODE},
				{'trigger': gs.Triggers.SAY_BUTTON_PRESSED, 'source': gs.PRONUNCIATION_PANEL, 'dest': gs.PRONUNCIATION_RESULT},
				{'trigger': gs.Triggers.PRACTICE_FAILED, 'source': gs.PRONUNCIATION_RESULT, 'dest': gs.PRONUNCIATION_PANEL},
				{'trigger': gs.Triggers.PRONUNCIATION_PANEL_CLOSED, 'source':gs.PRONUNCIATION_RESULT , 'dest': gs.MISSION_MODE},
				{'trigger': gs.Triggers.PRONUNCIATION_PANEL_CLOSED, 'source':gs.PRONUNCIATION_PANEL , 'dest': gs.MISSION_MODE},
				{'trigger': gs.Triggers.PRONUNCIATION_PANEL_CLOSED, 'source':gs.MISSION_MODE , 'dest': gs.MISSION_MODE},
				{'trigger': gs.Triggers.PRACTICE_FINISHED, 'source':gs.PRONUNCIATION_RESULT , 'dest': gs.MISSION_MODE},
				{'trigger': gs.Triggers.OBJECT_CLICKED, 'source': gs.EXPLORATION_MODE, 'dest':gs.WORD_DISPLAY },
				{'trigger': gs.Triggers.N_SECONDS_LATER, 'source': gs.WORD_DISPLAY, 'dest': gs.EXPLORATION_MODE}
				
		]
		
		self.state_machine = Machine(self, states=self.states, transitions=self.transitions,
									 initial=gs.GAME_START)


	

class AlwaysMissionModeFSM(BaseGameFSM):
	'''
	the game only has mission mode
	'''
	def __init__(self,ros_node_mgr):
		self.states = [gs.GAME_START, gs.EXPLORATION_MODE,gs.MISSION_MODE,gs.PRONUNCIATION_PANEL,gs.PRONUNCIATION_RESULT,gs.WORD_DISPLAY]
		self.transitions = [
				{'trigger': gs.Triggers.START_BUTTON_PRESSED, 'source': gs.GAME_START, 'dest': gs.EXPLORATION_MODE},
				{'trigger': gs.Triggers.TOPLEFT_BUTTON_PRESSED, 'source': gs.EXPLORATION_MODE, 'dest': gs.MISSION_MODE},
				{'trigger': gs.Triggers.TOPLEFT_BUTTON_PRESSED, 'source': gs.MISSION_MODE, 'dest': gs.MISSION_MODE},
				#{'trigger': gs.Triggers.TOPLEFT_BUTTON_PRESSED, 'source': gs.MISSION_MODE, 'dest': gs.EXPLORATION_MODE},
				{'trigger': gs.Triggers.OBJECT_CLICKED, 'source': gs.MISSION_MODE, 'dest': gs.PRONUNCIATION_PANEL},
				{'trigger': gs.Triggers.TARGET_OBJECT_COLLECTED , 'source': gs.PRONUNCIATION_RESULT, 'dest': gs.MISSION_MODE},
				{'trigger': gs.Triggers.SAY_BUTTON_PRESSED, 'source': gs.PRONUNCIATION_PANEL, 'dest': gs.PRONUNCIATION_RESULT},
				{'trigger': gs.Triggers.PRACTICE_FAILED, 'source': gs.PRONUNCIATION_RESULT, 'dest': gs.PRONUNCIATION_PANEL},
				{'trigger': gs.Triggers.PRONUNCIATION_PANEL_CLOSED, 'source':gs.PRONUNCIATION_RESULT , 'dest': gs.MISSION_MODE},
				{'trigger': gs.Triggers.PRONUNCIATION_PANEL_CLOSED, 'source':gs.PRONUNCIATION_PANEL , 'dest': gs.MISSION_MODE},
				{'trigger': gs.Triggers.PRONUNCIATION_PANEL_CLOSED, 'source':gs.MISSION_MODE , 'dest': gs.MISSION_MODE},
				{'trigger': gs.Triggers.PRACTICE_FINISHED, 'source':gs.PRONUNCIATION_RESULT , 'dest': gs.MISSION_MODE},
				{'trigger': gs.Triggers.OBJECT_CLICKED, 'source': gs.EXPLORATION_MODE, 'dest':gs.WORD_DISPLAY },
				{'trigger': gs.Triggers.N_SECONDS_LATER, 'source': gs.WORD_DISPLAY, 'dest': gs.EXPLORATION_MODE}
		
		]
		
		self.state_machine = Machine(self, states=self.states, transitions=self.transitions,
									 initial=gs.GAME_START)
		self.ros_node_mgr = ros_node_mgr

	def start_trigger(self,trigger_string):
		if trigger_string == gs.Triggers.TOPLEFT_BUTTON_PRESSED:
			if self.state == gs.EXPLORATION_MODE:
				print("send ROS commands to disable button...")
				# if it is the first time entering the mission mode, 
				# disable the 'explore' button so that the player cannot go back to the explore mode
				results_params = {"buttonName":"ExploreButton"}
				self.ros_node_mgr.send_ispy_cmd(BUTTON_DISABLED,results_params)
		getattr(self, trigger_string)()

	
class AlwaysExploreModeFSM(BaseGameFSM):
	'''
	the game only has explore modes
	'''
	def __init__(self):
		self.states = [gs.GAME_START, gs.EXPLORATION_MODE,gs.MISSION_MODE,gs.PRONUNCIATION_PANEL,gs.PRONUNCIATION_RESULT,gs.WORD_DISPLAY]
		self.transitions = [
				
		]
		
		self.state_machine = Machine(self, states=self.states, transitions=self.transitions,
									 initial=gs.GAME_START)