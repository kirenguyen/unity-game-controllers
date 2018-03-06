from storybook_controller.storybook_fsm import StorybookFSM
from storybook_controller.ros_node_manager import ROSNodeManager
from storybook_controller.student_model import StudentModel

from unity_game_msgs.msg import StorybookCommand

import argparse
import signal
import sys
import time
import _thread as thread

def signal_handler(signal, frame):
  print("Received signal, closing!")
  sys.exit()

def main(argv):
  # TODO: Parse arguments using argparse.

  ros_node_manager = ROSNodeManager("Storybook_FSM_Controller")
  ros_node_manager.init_ros_node()
  student_model = StudentModel()

  fsm = StorybookFSM(ros_node_manager, student_model)
  
  # Run the ROS isteners in separate threads.
  # We force serialization by having all handlers add events to the
  # task queue, and then executing events on the queue in order
  # in a separate dedicated thread.
  ros_node_manager.start_storybook_publisher()
  ros_node_manager.start_jibo_publisher()
  ros_node_manager.start_jibo_asr_publisher()

  thread.start_new_thread(ros_node_manager.start_storybook_event_listener,
                          (fsm.storybook_event_ros_message_handler,))

  thread.start_new_thread(ros_node_manager.start_storybook_page_info_listener,
                          (fsm.storybook_page_info_ros_message_handler,))

  thread.start_new_thread(ros_node_manager.start_storybook_state_listener,
                          (fsm.storybook_state_ros_message_handler,))   

  thread.start_new_thread(ros_node_manager.start_jibo_state_listener,
                          (fsm.jibo_state_ros_message_handler,))
  
  thread.start_new_thread(ros_node_manager.start_jibo_asr_listener,
                          (fsm.jibo_asr_ros_message_handler,))
  
  # Main event queue.
  thread.start_new_thread(fsm.process_main_event_queue, ())
  signal.signal(signal.SIGINT, signal_handler)

  # Spin and periodically check the state of the student model.
  # Don't plot too often, maybe like once every few seconds.
  while True:
    try:
      # student_model.plot_distribution()
      time.sleep(1)
    except KeyboardInterrupt:
      print("Received signal to abort!")
      sys.exit()

if __name__ == "__main__":
  main(sys.argv)


