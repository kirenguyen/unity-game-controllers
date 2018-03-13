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
import sys

from unity_game_msgs.msg import StorybookCommand
from unity_game_msgs.msg import StorybookEvent
from jibo_msgs.msg import JiboAsrCommand

from storybook_controller.storybook_constants import *
from storybook_controller.jibo_commands_builder import JiboStorybookBehaviors

class StorybookFSM(object):
  def __init__(self, ros_node_manager, student_model):
    # The FSM maintains a queue that ros messages are added to, so that
    # different topics can send their events and they will be processed
    # one at a time, in order.
    self.event_queue = queue.Queue()

    self.ros = ros_node_manager
    self.student_model = student_model

    # State information about what's going on in the story right now.
    self.current_storybook_mode = None
    self.current_story = None
    self.current_page_number = None
    self.current_stanza_index = None
    self.current_tinker_texts = None
    self.current_scene_objects = None
    self.current_sentences = None

    self.STORYBOOK_EVENT_MESSAGES = [
      StorybookEvent.HELLO_WORLD,
      StorybookEvent.SPEECH_ACE_RESULT,
      StorybookEvent.WORD_TAPPED,
      StorybookEvent.SCENE_OBJECT_TAPPED,
      StorybookEvent.SENTENCE_SWIPED,
    ]

    self.started = False;

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

  # TODO: consider getting rid of this and just handling everything via
  # the FSM python logic stuff.
  def process_main_event_queue(self):
    while True:
      # This call to get() will block the thread until something is there.
      data = self.event_queue.get()
      # Handle data.
      if data.event_type == StorybookEvent.HELLO_WORLD:
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
        self.ros.send_storybook_command(command, json.dumps(params))
        # TODO: decide best place to send the ASR command.
        # self.ros.send_jibo_command(JiboStorybookBehaviors.EXPLAIN_WORD, "Sent ASR command")

      elif data.event_type == StorybookEvent.SPEECH_ACE_RESULT:
        speechace_result = json.loads(data.message)
        self.student_model.update_with_speechace_result(speechace_result)
      
      elif data.event_type == StorybookEvent.WORD_TAPPED:
        message = json.loads(data.message)
        word = message["word"]
        self.student_model.update_with_word_tapped(word)
        # TODO: if we asked a question, see if this tap was the correct answer.

        # Tell Jibo to say this word.
        self.ros.send_jibo_command(JiboStorybookBehaviors.EXPLAIN_WORD, word, .5, .45);
      
      elif data.event_type == StorybookEvent.SCENE_OBJECT_TAPPED:
        message = json.loads(data.message)
        print("Scene object tapped:", message["label"])

      elif data.event_type == StorybookEvent.SENTENCE_SWIPED:
        message = json.loads(data.message)
        print("Sentence swiped:", message["index"], message["text"])

      # TODO: Maybe call queue.task_done() differently in each above case,
      # because we might want to use a join in the future and block
      # on all tasks being completed, for example waiting for a message
      # to be sent and acknowledged before continuing on with execution.
      self.event_queue.task_done()

  def storybook_event_ros_message_handler(self, data):
    """
    Define the callback function to pass to ROS node manager.
    This callback is called when the ROS node manager receives a new
    StorybookEvent message, which will be triggered by events in the storybook.
    """

    if data.event_type in self.STORYBOOK_EVENT_MESSAGES:
      self.event_queue.put(data)
    else:
      # Fail fast.
      print("Unknown message type!")
      sys.exit(1)


  def storybook_page_info_ros_message_handler(self, data):
    """
    Define the callback function for StorybookPageInfo messages.
    """
    # TODO: update our knowledge of which page we're on, etc.
    # Also might need to update student model with what current words are?
    pass

  def storybook_state_ros_message_handler(self, data):
    """
       Define the callback function for StorybookState messages.
    """
    
    # TODO: Remove this after done testing stuff.
    if data.audio_playing:
      print("Audio is playing in storybook:", data.audio_file)

    # TODO: update fsm state.

  def jibo_state_ros_message_handler(self, data):
    """
    Define callback function for when ROS node manager receives a new
    JiboState message, which will be at 10Hz.
    """
    # TODO: this is just testing code, replace later.
    if data.is_playing_sound:
      print("Jibo tts ongoing:", data.tts_msg)


  def jibo_asr_ros_message_handler(self, data):
    """
    Define callback function for when ROS node manager receives a new
    JiboAsr message, which should be whenever someone says "Hey, Jibo! ..."
    """
    if data.transcription == "" or data.transcription == ASR_NOSPEECH:
      return

    print("Got Jibo ASR transcription:", data.transcription)

