"""
This file implements the finite state machine (FSM) controller for the
interactive storybook app.

The FSM is responsible for state management, publication and subscription to
ROS topics, updating the student model, and sending commands to the robot agent
(Jibo or Tega).
"""

import transitions
import json
import queue
import time

from unity_game_msgs.msg import StorybookCommand
from unity_game_msgs.msg import StorybookGameInfo

from storybook_controller.storybook_constants import *

class StorybookFSM(object):
  def __init__(self, ros_node_manager, student_model):
    # The FSM maintains a queue that ros messages are added to, so that
    # different topics can send their events and they will be processed
    # one at a time, in order.
    self.event_queue = queue.Queue()

    self.ros = ros_node_manager
    self.student_model = student_model

    self.ALLOWED_LOG_MESSAGES = [
      StorybookGameInfo.HELLO_WORLD,
      StorybookGameInfo.SPEECH_ACE_RESULT,
      StorybookGameInfo.REQUEST_ROBOT_FEEDBACK,
      StorybookGameInfo.WORD_TAPPED,
    ]

    self.states = []
    self.initial_state = None
    self.transitions = [
      {
        "trigger": "",
        "source": "",
        "dest": "",
        "after": ""
      },
      {
        "trigger": "",
        "source": "",
        "dest": "",
        "after": ""
      }
    ]

    # self.state_machine = transitions.Machine(
    #   self, states=self.states, transitions=self.transitions,
    #   initial=self.initial_state)

  def process_main_event_queue(self):
    while True:
      # This call to get() will block the thread until something is there.
      data = self.event_queue.get()
      # Handle data.
      if data.message_type == StorybookGameInfo.HELLO_WORLD:
        print("Received hello world!")
        # Sending back a dummy response.
        # Wait for a few seconds to ensure message not dropped.
        time.sleep(3)
        params = {
          "obj1": 1,
          "obj2": 2
        }
        command = StorybookCommand.PING_TEST
        self.ros.send_storybook_command(command, "")
      elif data.message_type == StorybookGameInfo.SPEECH_ACE_RESULT:
        speechace_result = json.loads(data.message)
        # print(speechace_result)
        print(speechace_result["text_score"]["quality_score"])
        params = {
          "obj1": 1,
          "obj2": "hi"
        }
        command = StorybookCommand.PING_TEST
        self.ros.send_storybook_command(command, json.dumps(params))
      elif data.message_type == StorybokGameInfo.REQUEST_ROBOT_FEEDBACK:
        pass
      elif data.message_type == StorybookGameInfo.WORD_TAPPED:
        text = data.message
        # TODO: do something with this data
      # TODO: call queue.task_done() differenlty in each above case,
      # because we might want to use a join in the future and block
      # on all tasks being completed, for example waiting for a message
      # to be sent and acknowledged before continuing on with execution.
      self.event_queue.task_done()

  def ros_message_handler(self, data):
    """
    Define the callback function to pass to ROS node manager. This callback is
    called when the ROS node manager receives new data.
    """
    print("Received ROS message with data:\n", data)
    
    if data.message_type in self.ALLOWED_LOG_MESSAGES:
      self.event_queue.put(data)
    else:
      # Fail fast.
      print("Unknown message type!")
      sys.exit(1)
  

    # CONCURRENCY WILL BE A HUGE PROBLEM FOR THIS
    #
    # Need to define all of the callback functions for conditions and after
    # functions.
    #
    # Triggers need to be called by name by other functions.
    #
    # Conditions with arguments are passed those arguments by the call to the
    # trigger.
