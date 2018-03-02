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
from unity_game_msgs.msg import StorybookInfo

from storybook_controller.storybook_constants import *
from storybook_controller.jibo_commands import JiboStorybookBehaviors

class StorybookFSM(object):
  def __init__(self, ros_node_manager, student_model):
    # The FSM maintains a queue that ros messages are added to, so that
    # different topics can send their events and they will be processed
    # one at a time, in order.
    self.event_queue = queue.Queue()

    self.ros = ros_node_manager
    self.student_model = student_model

    self.ALLOWED_LOG_MESSAGES = [
      StorybookInfo.HELLO_WORLD,
      StorybookInfo.SPEECH_ACE_RESULT,
      StorybookInfo.REQUEST_ROBOT_FEEDBACK,
      StorybookInfo.WORD_TAPPED,
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
      if data.message_type == StorybookInfo.HELLO_WORLD:
        print("Received hello world!")
        # Sending back a dummy response.
        params = {
          "obj1": 1,
          "obj2": 2
        }
        command = StorybookCommand.PING_TEST
        # Wait for a little bit to ensure message not dropped.
        time.sleep(1)
        print("Sending ack")
        self.ros.send_storybook_command(command, "")
        print("Sending message to Jibo")
        self.ros.send_jibo_command(JiboStorybookBehaviors.EXPLAIN_WORD, "Hi Hanna, it worked!")
      
      elif data.message_type == StorybookInfo.SPEECH_ACE_RESULT:
        speechace_result = json.loads(data.message)
        # print(speechace_result)
        print(speechace_result["text_score"]["quality_score"])
        params = {
          "obj1": 1,
          "obj2": "hi"
        }
        command = StorybookCommand.PING_TEST
        self.ros.send_storybook_command(command, json.dumps(params))
      
      elif data.message_type == StorybookInfo.REQUEST_ROBOT_FEEDBACK:
        pass
      
      elif data.message_type == StorybookInfo.WORD_TAPPED:
        text = data.message
        # TODO: do something with this data
      
      # TODO: Maybe call queue.task_done() differently in each above case,
      # because we might want to use a join in the future and block
      # on all tasks being completed, for example waiting for a message
      # to be sent and acknowledged before continuing on with execution.
      self.event_queue.task_done()

  def storybook_ros_message_handler(self, data):
    """
    Define the callback function to pass to ROS node manager.
    This callback is called when the ROS node manager receives a new
    StorybookInfo message, which will be triggered by events in the storybook.
    """
    print("Received StorybookInfo message with data:\n", data)
    
    if data.message_type in self.ALLOWED_LOG_MESSAGES:
      self.event_queue.put(data)
    else:
      # Fail fast.
      print("Unknown message type!")
      sys.exit(1)

  def jibo_state_ros_message_handler(self, data):
    """
    Define callback function for when ROS node manager receives a new
    JiboState message, which will be at 10Hz.
    """
    if data.is_playing_sound:
      print("playing sound:", data.tts_msg)


  def jibo_asr_ros_message_handler(self, data):
    """
    Define callback function for when ROS node manager receives a new
    JiboAsr message, which should be whenever someone says "Hey, Jibo! ..."
    """
    print("Received JiboAsr message with data:\n", data)
