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
  # TODO: really need to think hard about how concurrency is going
  # to mess with this...
  ros_node_manager.start_storybook_publisher()
  thread.start_new_thread(ros_node_manager.start_storybook_listener,
                          (fsm.ros_message_handler,))
  # thread.start_new_thread(ros_node_manager.start_robot_listener,
  #                         (fsm.ros_message_handler,))
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


