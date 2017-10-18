import time

from transitions import Machine
from .AgentModel import AgentModel
from .StudentModel import StudentModel
from .RobotBehaviorList import RobotBehaviors
from .RobotBehaviorList import RobotRoles
from .RobotBehaviorList import RobotRolesBehaviorsMap

# from GameUtils import Curriculum
from GameUtils import GlobalSettings
from GameUtils.GlobalSettings import iSpyGameStates as gs
from GameUtils.GlobalSettings import iSpyRobotInteractionStates as ris
from GameUtils.PronunciationUtils.PronunciationUtils import PronunciationUtils

if GlobalSettings.USE_ROS:
	from std_msgs.msg import Header  # standard ROS msg header
	from std_msgs.msg import String
	from unity_game_msgs.msg import iSpyCommand
	from unity_game_msgs.msg import iSpyAction

class ChildRobotInteractionFSM:
		'''
		child robot interaction FSM for robot's role switching project
		'''
		def __init__(self,ros_node_mgr):
			
			self.states = [ ris.ROBOT_TURN, ris.CHILD_TURN ]
			self.transitions = [
				{'trigger': ris.Triggers.CHILD_TURN_DONE, 'source': ris.CHILD_TURN, 'dest': ris.ROBOT_TURN },
				{'trigger': ris.Triggers.ROBOT_TURN_DONE, 'source': ris.ROBOT_TURN, 'dest': ris.CHILD_TURN},
			]
			self.state_machine = Machine(self, states=self.states, transitions=self.transitions,
									 initial=ris.CHILD_TURN)

			self.ros_node_mgr = ros_node_mgr

			self.agent_model = AgentModel()

			self.role_behavior_mapping = RobotRolesBehaviorsMap()

		def turn_taking(self):
			# check whether it is robot's turn or child's turn in the game play
			if self.state == ris.ROBOT_TURN:
				# then, next turn is child's 
				getattr(self, ris.Triggers.ROBOT_TURN_DONE)() # convert the variabel to string, which is the name of the called function
		
			elif self.state == ris.CHILD_TURN:
				# then, next turn is robot's
				getattr(self, ris.Triggers.CHILD_TURN_DONE)()
	
			print("==========TURN TAKING===============: Current Turn = "+self.state)

			self.physical_actions =[]
			# robot's response 
			self.get_turn_taking_action()

		
		def react(self,gameStateTrigger):
			'''
			react to ispy game state change
			'''
			## TEMPORARY: some code is badly written.needs modifications later
			if gameStateTrigger == gs.Triggers.TARGET_OBJECT_COLLECTED:
				if self.state == ris.ROBOT_TURN:
					# robot just collected an object. celebrate
					self._perform_robot_physical_action(self.physical_actions)
					time.sleep(1)
					self.ros_node_mgr.send_robot_cmd(RobotBehaviors.WIN_MOTION)
				elif self.state == ris.CHILD_TURN:
					self.ros_node_mgr.send_robot_cmd(RobotBehaviors.REACT_CHILD_ANSWER_CORRECT)
					time.sleep(0.4)
					self.ros_node_mgr.send_robot_cmd(RobotBehaviors.RING_ANSWER_CORRECT)
			
			elif gameStateTrigger  == gs.Triggers.OBJECT_CLICKED:
				if self.state == ris.ROBOT_TURN:
					#robot's turn, pronounce the word
					self.ros_node_mgr.send_robot_cmd(RobotBehaviors.EYE_FIDGET)
					self.ros_node_mgr.send_robot_cmd(RobotBehaviors.RING_ANSWER_CORRECT)
				elif self.state == ris.CHILD_TURN:
					self.ros_node_mgr.send_robot_cmd(RobotBehaviors.EYE_FIDGET)
			
			
			elif gameStateTrigger  == gs.Triggers.SAY_BUTTON_PRESSED:
				if self.state == ris.ROBOT_TURN:
					self._perform_robot_physical_action(self.physical_actions)

			
			elif gameStateTrigger  == gs.Triggers.PRONUNCIATION_PANEL_CLOSED:
				if self.state == ris.CHILD_TURN:
					self.ros_node_mgr.send_robot_cmd(RobotBehaviors.REACT_CHILD_ANSWER_WRONG)
					self.ros_node_mgr.send_robot_cmd(RobotBehaviors.LOOK_CENTER)

		def send_robot_action(self,action_name):
			'''
			get general robot's responsee (not related to robot's role swtiching)
			examples: encouragement, celerbration, empathy, happiness
			'''
			self.ros_node_mgr.send_robot_cmd(action_name)

		def get_turn_taking_action(self):
			'''
			check the current interaction FSM to decide whether the robot should respond
			then, use agent model to decide how the robot should respond if it needs to respond
			'''
			
			physical_actions = ""
			virtual_action = ""

			if self.state == ris.ROBOT_TURN:
				# choose an action for robot
				print("Turn Taking Action...ROBOTS TURN")
				robot_role = self._get_role()
				actions = self._get_behaviors(robot_role)
				physical_actions = actions['physical'] 
				virtual_action = actions['virtual']

			elif self.state == ris.CHILD_TURN:
				# no need to respond at this point 
				print("Turn Taking Action...CHILD TURN")

			if physical_actions:
				self._perform_robot_physical_action(physical_actions)
				time.sleep(5)
				self._perform_robot_physical_action(self.physical_actions)
			if virtual_action:
				time.sleep(5) 
				self._perform_robot_virtual_action(virtual_action)

		def _get_behaviors(self,role):
			'''
			Get corresponding virtual and physical actions for a given input robot's role
			'''
			return self.role_behavior_mapping.get_actions(role)
			

		def _perform_robot_physical_action(self,actions):
			'''
			send the physical action message via ROS to the robot
			'''
			print("perform robot physical action runs..")
			print("remaining actions")
			print(actions)
			if len(actions) > 0:
				action = actions[0]
				self.physical_actions = actions[1:]
				print("physical action is: "+action)

				## TEMPERAROY: CHEATING HERE
				#'physical':[RobotBehaviors.LOOK_AT_TABLET,RobotBehaviors.ROBOT_TURN_SPEECH,
				#RobotBehaviors.PRONOUNCE_CORRECT, RobotBehaviors.WIN_SPEECH], 
				if action == RobotBehaviors.PRONOUNCE_CORRECT:
					self.ros_node_mgr.send_robot_cmd(action,"cat")
				else:
					self.ros_node_mgr.send_robot_cmd(action)

		def _perform_robot_virtual_action(self,action):
			'''
			send the virtual action message via ROS to the tablet 
			'''
			print("perform robot virtual action")
			print(action)
			#get_game_object_for_clicking()
			self.ros_node_mgr.send_ispy_cmd(iSpyCommand.ROBOT_VIRTUAL_ACTIONS,action)
			
		def get_game_object_for_clicking():
			'''
			select a game object in the ispy game for jibo to click and pronounce
			'''
			pass

		def _get_role(self):
			'''
			get the most approriate role from agent model
			'''
			role = self.agent_model.get_next_robot_role()
			return role