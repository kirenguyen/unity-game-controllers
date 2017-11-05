from transitions import Machine
from GameUtils.GlobalSettings import iSpyGameStates as gs
from GameUtils.GlobalSettings import iSpyRobotInteractionStates as ris

# contain FSMs for different modes
# iSpy role switching project uses "alwaysMissionModeFSM"
# curiosity assessment project uses "alwaysExploreModeFSM"
# "CompleteModeFSM" allows players to switch between explore and mission mode easily

class BaseGameFSM:
	def start_trigger(self,trigger_string):
		getattr(self, trigger_string)()

	def get_state(self):
		return self.state

class CompleteModeFSM(BaseGameFSM):
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
	def __init__(self):
		self.states = [gs.GAME_START, gs.EXPLORATION_MODE,gs.MISSION_MODE,gs.PRONUNCIATION_PANEL,gs.PRONUNCIATION_RESULT,gs.WORD_DISPLAY]
		self.transitions = [
				
		]
		
		self.state_machine = Machine(self, states=self.states, transitions=self.transitions,
									 initial=gs.GAME_START)
	
class AlwaysExploreModeFSM(BaseGameFSM):
	def __init__(self):
		self.states = [gs.GAME_START, gs.EXPLORATION_MODE,gs.MISSION_MODE,gs.PRONUNCIATION_PANEL,gs.PRONUNCIATION_RESULT,gs.WORD_DISPLAY]
		self.transitions = [
				
		]
		
		self.state_machine = Machine(self, states=self.states, transitions=self.transitions,
									 initial=gs.GAME_START)