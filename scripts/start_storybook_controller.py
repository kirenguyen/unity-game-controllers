from storybook_controller.storybook_fsm import StorybookFSM
from storybook_controller.ros_node_manager import ROSNodeManager
from storybook_controller.student_model import StudentModel
from storybook_controller.jibo_commands_builder import JiboStorybookBehaviors
from storybook_controller.storybook_constants import *

from unity_game_msgs.msg import StorybookCommand
from unity_game_msgs.msg import StorybookEvent
from unity_game_msgs.msg import StorybookPageInfo
from unity_game_msgs.msg import StorybookState
from jibo_msgs.msg import JiboAction # Commands to Jibo
from jibo_msgs.msg import JiboState # State from Jibo
from jibo_msgs.msg import JiboAsrCommand # ASR commands to Jibo
from jibo_msgs.msg import JiboAsrResult # ASR results from Jibo

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

  ros_node_manager.start_publisher(STORYBOOK_COMMAND_TOPIC, StorybookCommand)
  ros_node_manager.start_publisher(JIBO_ACTION_TOPIC, JiboAction)
  ros_node_manager.start_publisher(JIBO_ASR_COMMAND_TOPIC, JiboAsrCommand)

  thread.start_new_thread(ros_node_manager.start_listener, (
    STORYBOOK_EVENT_TOPIC, StorybookEvent, fsm.storybook_event_ros_message_handler,))

  thread.start_new_thread(ros_node_manager.start_listener, (
    STORYBOOK_PAGE_INFO_TOPIC, StorybookPageInfo, fsm.storybook_page_info_ros_message_handler,))

  thread.start_new_thread(ros_node_manager.start_listener, (
    STORYBOOK_STATE_TOPIC, StorybookState, fsm.storybook_state_ros_message_handler,))

  thread.start_new_thread(ros_node_manager.start_listener, (
    JIBO_STATE_TOPIC, JiboState, fsm.jibo_state_ros_message_handler,))

  thread.start_new_thread(ros_node_manager.start_listener, (
    JIBO_ASR_RESULT_TOPIC, JiboAsrResult, fsm.jibo_asr_ros_message_handler,))

  # Main event queue.
  thread.start_new_thread(fsm.process_main_event_queue, ())

  # TODO: should send this in response to something else happening, not right
  # at the start? Or maybe it's ok, as long as we first start Jibo then
  # start the controller. Hard to work out this protocol.
  # ros_node_manager.send_jibo_asr_command(JiboAsrCommand.START)
  time.sleep(10)
  # ros_node_manager.send_jibo_command(JiboStorybookBehaviors.EXPLAIN_WORD, "hello", 1.0)

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


