from abc import ABC, abstractmethod

class BaseClassFSM:

		@abstractmethod
		def __init__(self,ros_node_mgr,task_controller,game_controller,participant_id,game_round):
			pass

		@abstractmethod
		def check_existence_of_asr_rostopic(self):
			print("Some implementation!")

		@abstractmethod
		def on_child_max_elapsed_time(self):
			print("Some implementation!")

		@abstractmethod
		def on_enter_childTURN(self):
			self.turn_start_time = datetime.now()
			self.turn_end_time = None
			self.turn_duration = ""
			self.current_task_turn_index += 1
			
			self.robot_clickedObj = "" # reset robot's clicked obj
			self.explore_action = ""
			self.virtual_action = ""
			self._ros_publish_data()
			threading.Timer(3.0, self.start_tracking_child_interaction).start()
			threading.Timer(10.0, self.on_child_max_elapsed_time).start()
			self.reset_elapsed_time = False

		@abstractmethod
		def on_enter_robotTURN(self):
			print("Some implementation!")

		@abstractmethod
		def on_enter_childTURN_listenChildSpeechResponse(self):
			print("Some implementation!")

		@abstractmethod
		def on_enter_robotTURN_listenChildSpeechResponse(self):
			print("Some implementation!")
		
		@abstractmethod
		def on_enter_childTURN_robotHelp(self):
			print("Some implementation!")

		@abstractmethod
		def on_enter_robotTURN_childHelp(self):
			print("Some implementation!")

		@abstractmethod
		def listen_child_speech(self):
			print("Some implementation!")

		@abstractmethod
		def start_tracking_child_interaction(self):
			print("Some implementation!")
		
		@abstractmethod
		def stop_tracking_child_interaction(self):
			print("Some implementation!")

		@abstractmethod
		def on_no_ispy_action_alert(self,attempt):
			print("Some implementation!")

		@abstractmethod
		def on_tega_state_received(self,data):
			print("Some implementation!")

		@abstractmethod
		def on_tega_new_asr_result(self,data):
			print("Some implementation!")

		@abstractmethod
		def reset_turn_taking(self):
			print("Some implementation!")

		@abstractmethod
		def turn_taking(self,max_time=False):
			self.ros_node_mgr.send_ispy_cmd(iSpyCommand.WHOSE_TURN, {"whose_turn":self.state})
			# update the number of available objects for child's learning states
			self.child_states.set_num_available_objs(self.task_controller.get_num_available_target_objs())

		@abstractmethod
		def react(self,gameStateTrigger,  clicked_obj_name = ""):
			print("Some implementation!")

		@abstractmethod
		def _robot_virutal_action_wait(self):
			print("Some implementation!")

		@abstractmethod
		def _wait_until(self):
			print("Some implementation!")

		@abstractmethod
		def _wait_until_all_audios_done(self):
			print("Some implementation!")

		@abstractmethod
		def get_turn_taking_actions(self):
			print("Some implementation!")

		@abstractmethod
		def get_robot_general_response(self):
			print("Some implementation!")

		@abstractmethod
		def start_task_end_celebration(self, action_number):
			print("Some implementation!")

		@abstractmethod
		def start_task_end_assessment(self, action_number):
			print("Some implementation!")
		
		@abstractmethod
		def start_task_end_behavior(self, action_number):
			print("Some implementation!")

		@abstractmethod
		def _perform_robot_physical_actions(self,action_type):
			print("Some implementation!")

		@abstractmethod
		def _ros_publish_data(self,action="", v_action = "", ispy_action=False):
			print("Some implementation!")

		@abstractmethod
		def _robot_question_asking(self,question_cmd):
			print("Some implementation!")

		@abstractmethod
		def _perform_robot_virtual_action(self,action):
			print("Some implementation!")

		
