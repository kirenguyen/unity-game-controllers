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
import threading

from unity_game_msgs.msg import StorybookCommand
from unity_game_msgs.msg import StorybookEvent
from unity_game_msgs.msg import StorybookPageInfo
from unity_game_msgs.msg import StorybookState

from jibo_msgs.msg import JiboAction # Commands to Jibo
from jibo_msgs.msg import JiboState # State from Jibo
from jibo_msgs.msg import JiboAsrCommand # ASR commands to Jibo
from jibo_msgs.msg import JiboAsrResult # ASR results from Jibo

from storybook_controller.storybook_constants import *
from storybook_controller.robot_feedback import EndPageQuestionType
from storybook_controller.jibo_commands_builder import JiboStorybookBehaviors

class StorybookFSM(object):
  def __init__(self, ros_node_manager, student_model):
    # The FSM maintains a queue that ros messages are added to, so that
    # different topics can send their events and they will be processed
    # one at a time, in order.
    self.event_queue = queue.Queue()

    self.ros = ros_node_manager
    self.student_model = student_model

    # State information about Jibo
    self.jibo_tts_on = False
    self.jibo_asr_empty = True

    # State information about what's going on in the story right now.
    # State that changes once per story.
    self.current_storybook_mode = None
    self.current_story = None
    self.num_story_pages = None
    # State that changes once per page.
    self.current_page_number = None
    self.current_tinkertexts = None
    self.current_scene_objects = None
    self.current_sentences = None
    # State that changes more frequently than once per page.
    self.reported_evaluating_sentence_index = None
    self.storybook_audio_playing = None
    self.storybook_audio_file = None

    # Timer for when we're waiting for the child to read a sentence in
    # evaluate mode.
    self.child_audio_evaluate_timer = None
    self.CHILD_AUDIO_SILENCE_TIMEOUT_SECONDS = 8 # Amount of time after detecting silence before reprompting a read.

    self.child_end_page_question_timer = None
    self.CHILD_END_PAGE_QUESTION_TIMEOUT_SECONDS = 8

    # For when Jibo prompts the child at the end of a page.
    self.end_page_questions = []
    self.end_page_question_idx = None

    # Useful to have a list of allowed messages, make sure no unknown messages.
    self.STORYBOOK_EVENT_MESSAGES = [
      StorybookEvent.HELLO_WORLD,
      StorybookEvent.SPEECH_ACE_RESULT,
      StorybookEvent.WORD_TAPPED,
      StorybookEvent.SCENE_OBJECT_TAPPED,
      StorybookEvent.SENTENCE_SWIPED,
      StorybookEvent.RECORD_AUDIO_COMPLETE,
      StorybookEvent.STORY_SELECTED,
      StorybookEvent.STORY_LOADED,
      StorybookEvent.CHANGE_MODE,
      StorybookEvent.REPEAT_END_PAGE_QUESTION,
    ]

    not_reading_states = ["APP_START"]
    explore_states = [
      "BEGIN_EXPLORE",
      "EXPLORING_PAGE",
      "END_EXPLORE"
    ]

    evaluate_states = [
      "BEGIN_EVALUATE", # After switching to evaluate mode.
      "WAITING_FOR_STORY_LOAD", # After a story selected and is loading.
      "WAITING_FOR_NEXT_PAGE", # After story loaded, waiting for page_info.
      "WAITING_FOR_CHILD_AUDIO", # After a sentence has been shown.
      "WAITING_FOR_END_PAGE_JIBO_QUESTION", # Wait for Jibo tts to finish.
      "WAITING_FOR_END_PAGE_CHILD_RESPONSE", # Could be speech or tablet event.
      "WAITING_FOR_END_PAGE_JIBO_RESPONSE", # Jibo gives corrections/comments.
      "WAITING_FOR_NEXT_PAGE_JIBO_INTERLUDE", # After end page child response.
      "END_STORY", # Moved to the "The End" page, tell Jibo to ask a question,
      "WAITING_FOR_END_STORY_CHILD_AUDIO", # Wait for child to respond.
      "END_EVALUATE" # End evaluate mode. Idle until another mode is entered.
    ]

    self.states = not_reading_states + explore_states + evaluate_states
    self.initial_state = "APP_START"

    self.not_reading_transitions = []
    self.explore_transitions = []
    self.evaluate_transitions = [
      {
        "trigger":"storybook_selected", # After assets have downloaded 
        "source": ["BEGIN_EVALUATE", "BEGIN_EXPLORE"],
        "dest": "WAITING_FOR_STORY_LOAD",
        # Optional spot to do stuff while page is loading, but ran
        # into problems if Jibo talks due to timing issues with jibo_finish_tts.
      },
      {
        "trigger":"storybook_loaded", # At this point, we're on the title screen.
        "source": "WAITING_FOR_STORY_LOAD",
        "dest": "WAITING_FOR_STORY_LOAD",
        "before": "jibo_start_story",
        "conditions": ["in_evaluate_mode"]
      },
      {
        "trigger":"jibo_finish_tts",
        "source": "WAITING_FOR_STORY_LOAD",
        "dest": "WAITING_FOR_NEXT_PAGE",
        "before":"tablet_next_page", # Go to the first page.
        "conditions": ["in_evaluate_mode"]
      },
      {
        "trigger": "jibo_finish_tts",
        "source": "WAITING_FOR_STORY_LOAD",
        "dest": "EXPLORING_PAGE",
        "conditions": ["in_explore_mode"]
      },
      {
        "trigger":"page_info_received",
        "source":"WAITING_FOR_NEXT_PAGE",
        "dest": "WAITING_FOR_CHILD_AUDIO",
        # This action has really become more like start page, because it tells the
        # storybook to show the first sentence, then the storybook handles showing
        # subsequent sentences on its own.
        "after": ["tablet_show_next_sentence", "start_child_audio_timer"],
        "conditions": ["in_evaluate_mode"]
      },
      {
        "trigger": "page_info_received",
        "source": "EXPLORING_PAGE",
        "dest": "EXPLORING_PAGE",
        "unless": ["is_last_page"]
      },
      {
        "trigger": "page_info_received",
        "source": "EXPLORING_PAGE",
        "dest": "END_STORY",
        "after": ["jibo_end_story"]
      },
      {
        "trigger":"child_audio_timeout",
        "source":"WAITING_FOR_CHILD_AUDIO",
        "dest": "WAITING_FOR_CHILD_AUDIO",
        "before":["jibo_reprompt_child_audio", "tablet_stop_and_discard_record"],
      },
      {
        "trigger":"jibo_finish_tts", # This trigger will be used a lot, but it will have different effects based on source state.
        "source":"WAITING_FOR_CHILD_AUDIO",
        "dest": "WAITING_FOR_CHILD_AUDIO",
        "after": ["tablet_begin_record"], # After Jibo finishes reprompting.
        # TODO: commented out start_child_audio_timer
      },
      # Only start timer after there has been silence. Either child didn't press button
      # or child didn't speak at all.
      {
        "trigger":"jibo_finish_child_asr",
        "source":"WAITING_FOR_CHILD_AUDIO",
        "dest":"WAITING_FOR_CHILD_AUDIO",
        "after": ["start_child_audio_timer"]
      },
      {
        "trigger":"jibo_got_new_asr", # When it goes from silence to not silence.
        "source":"WAITING_FOR_CHILD_AUDIO",
        "dest":"WAITING_FOR_CHILD_AUDIO",
        "after":["stop_child_audio_timer"]
      },
      {
        "trigger":"child_sentence_audio_complete",
        "source":"WAITING_FOR_CHILD_AUDIO",
        "dest": "WAITING_FOR_CHILD_AUDIO",
        "before":"stop_child_audio_timer",
        # "before":"", # TODO: allow Jibo to respond? "Good job!"
        # TODO commented out everything here, "after": ["start_child_audio_timer"], # Commented out tablet_show_next_sentence since tablet is responsible for that now.
        "conditions": ["more_sentences_available"]
      },
      {
        "trigger":"child_sentence_audio_complete",
        "source":"WAITING_FOR_CHILD_AUDIO",
        "dest": "WAITING_FOR_END_PAGE_JIBO_QUESTION",
        "before":"stop_child_audio_timer",
        "after": "send_end_page_prompt" # Could involve commands to Jibo and tablet.
      },
      # Either Jibo is asking a question for the first time or is repeating it,
      # regardless, we need to start the timer when Jibo's done talking.
      {
        "trigger":"jibo_finish_tts",
        "source": ["WAITING_FOR_END_PAGE_JIBO_QUESTION", "WAITING_FOR_END_PAGE_CHILD_RESPONSE"],
        "dest": "WAITING_FOR_END_PAGE_CHILD_RESPONSE",
        "after": "start_child_end_page_question_timer"
      },
      {
        "trigger": "child_end_page_question_timeout",
        "source": "WAITING_FOR_END_PAGE_CHILD_RESPONSE",
        "dest": "WAITING_FOR_END_PAGE_CHILD_RESPONSE",
        "after": "resend_end_page_prompt"
      },
      {
        "trigger": "child_request_repeat_end_page_question",
        "source": "WAITING_FOR_END_PAGE_CHILD_RESPONSE",
        "dest": "WAITING_FOR_END_PAGE_CHILD_RESPONSE",
        "before": "stop_child_end_page_question_timer",
        "after": "resend_end_page_prompt"
      },
      {
        "trigger":"child_end_page_got_answer",
        "source":"WAITING_FOR_END_PAGE_CHILD_RESPONSE",
        "dest":"WAITING_FOR_END_PAGE_JIBO_RESPONSE",
        "before":["stop_child_end_page_question_timer",
                  "jibo_end_page_response_to_child"]
      },
      {
        "trigger":"jibo_finish_tts",
        "source":"WAITING_FOR_END_PAGE_JIBO_RESPONSE",
        "dest":"WAITING_FOR_END_PAGE_JIBO_RESPONSE",
        "after":["delay_after_end_page_jibo_response", "end_page_jibo_response_complete"]
      },
      {
        "trigger":"end_page_jibo_response_complete",
        "source":"WAITING_FOR_END_PAGE_JIBO_RESPONSE",
        "dest": "WAITING_FOR_NEXT_PAGE_JIBO_INTERLUDE",
        "after": "jibo_next_page", # Jibo says something like 'Ok on to the next page!'
        "conditions": ["more_pages_available"]
      },
      {
        "trigger": "jibo_finish_tts",
        "source": "WAITING_FOR_NEXT_PAGE_JIBO_INTERLUDE",
        "dest": "WAITING_FOR_NEXT_PAGE",
        "after": "tablet_next_page" # Show the next page button, or navigate to the next page if automatic.
      },
      {
        "trigger":"end_page_jibo_response_complete",
        "source": "WAITING_FOR_END_PAGE_JIBO_RESPONSE",
        "dest": "END_STORY",
        "after": ["tablet_go_to_end_page", "jibo_end_story"]
      },
      {
        "trigger": "jibo_finish_tts",
        "source": "END_STORY",
        "dest": "WAITING_FOR_END_STORY_CHILD_AUDIO"
      },
      {
      # TODO: this will be a little tricky. Basically just listening for the child to say anything.
      # Don't want to cut off the child while she's speaking.
      # Can check for consecutive asr results from Jibo until one says NOSPEECH.
        "trigger": "jibo_finish_child_asr", 
        "source": "WAITING_FOR_END_STORY_CHILD_AUDIO",
        "dest": "END_EVALUATE",
        "after": "jibo_respond_to_end_story",
        "conditions": ["in_evaluate_mode"]
      },
      {
        "trigger": "jibo_finish_tts",
        "source": "END_EVALUATE",
        "dest": "END_EVALUATE",
        "after": ["begin_evaluate_mode", "tablet_show_library_panel"],
        "conditions": ["in_evaluate_mode"]
      },
      # Copy of above two triggers for explore mode.
      {
        "trigger": "jibo_finish_child_asr", 
        "source": "WAITING_FOR_END_STORY_CHILD_AUDIO",
        "dest": "END_EXPLORE",
        "after": "jibo_respond_to_end_story",
        "conditions": ["in_explore_mode"]
      },
      {
        "trigger": "jibo_finish_tts",
        "source": "END_EXPLORE",
        "dest": "END_EXPLORE",
        "after": ["begin_explore_mode", "tablet_show_library_panel"],
        "conditions": ["in_explore_mode"]
      },
      # Switching modes.
      {
        "trigger": "begin_evaluate_mode",
        "source": "*",
        "dest": "BEGIN_EVALUATE"
      },
      {
        "trigger": "begin_explore_mode",
        "source": "*",
        "dest": "BEGIN_EXPLORE"
      },
      # Catch all the triggers that might be called without regard
      # to the current state.
      {
        "trigger": "jibo_finish_tts",
        "source": "*",
        "dest": "="
      },
      {
        "trigger": "jibo_got_new_asr",
        "source": "*",
        "dest": "="
      },
      {
        "trigger": "jibo_finish_child_asr",
        "source": "*",
        "dest": "="
      },
      {
        "trigger": "page_info_received",
        "source": "*",
        "dest": "="
      },
      {
        "trigger": "child_request_repeat_end_page_question",
        "source": "*",
        "dest": "="
      }
    ]

    self.transitions = self.not_reading_transitions + \
      self.explore_transitions +self.evaluate_transitions

    # Use queued=True to ensure ensure that a transition finishes before the
    # next begins.
    self.state_machine = transitions.Machine(
      model=self, states=self.states, transitions=self.transitions,
      initial=self.initial_state, queued=True)

  # Runs in its own threads.
  # StorybookEvent messages are put onto this queue when received.
  # This forces serialization.
  # The messages are controlled in such a way that there should not be
  # out of order message problems.
  def process_main_event_queue(self):
    while True:
      try:
        data = self.event_queue.get_nowait()
        # Handle data.
        self.process_storybook_event(data)
        # TODO: Maybe call queue.task_done() differently in each above case,
        # because we might want to use a join in the future and block
        # on all tasks being completed, for example waiting for a message
        # to be sent and acknowledged before continuing on with execution.
        self.event_queue.task_done()
      except queue.Empty:
        time.sleep(.05)

  def process_storybook_event(self, data):
    if data.event_type == StorybookEvent.HELLO_WORLD:
      print("HELLO_WORLD message received")
      # Sending back a dummy response.
      params = {
        "obj1": 1,
        "obj2": 2
      }
      command = StorybookCommand.PING_TEST
      # Wait for a little bit to ensure message not dropped.
      time.sleep(1)
      print("Sending ack")
      self.ros.send_storybook_command(command, params)
      # Start Jibo ASR (stop first to prevent multiple active listeners).
      self.ros.send_jibo_asr_command(JiboAsrCommand.STOP)
      time.sleep(5)
      self.ros.send_jibo_asr_command(JiboAsrCommand.START)

    elif data.event_type == StorybookEvent.SPEECH_ACE_RESULT:
      print("SPEECH_ACE_RESULT message received")
      message = json.loads(data.message)
      sentence_index = message["index"]
      text = message["text"]
      duration = message["duration"]
      speechace_result = json.loads(message["speechace"])
      self.student_model.update_with_duration(duration, text)
      self.student_model.update_with_speechace_result(speechace_result)
    
    elif data.event_type == StorybookEvent.WORD_TAPPED:
      message = json.loads(data.message)
      word = message["word"].lower()
      phrase = message["phrase"].lower()
      print("WORD_TAPPED message received, word is", word, "phrase is", phrase, 
        "state is", self.state)
      if self.in_evaluate_mode():
        if self.state == "WAITING_FOR_END_PAGE_CHILD_RESPONSE":
          self.try_answer_question(EndPageQuestionType.WORD_TAP, word)
      elif self.in_explore_mode():
        self.student_model.update_with_explore_word_tapped(word)
        # Tell Jibo to say this phrase.
        text_to_say = None
        if len(phrase) > 0:
          text_to_say = "That says... " + phrase
        else:
          text_to_say = "That says... " + word
        self.ros.send_jibo_command(JiboStorybookBehaviors.SPEAK, text_to_say, .5, .45)
    
    elif data.event_type == StorybookEvent.SCENE_OBJECT_TAPPED:
      message = json.loads(data.message)
      tapped_id = message["id"]
      label = message["label"]
      print("SCENE_OBJECT_TAPPED message received:", label)

      if self.in_evaluate_mode():
        if self.state == "WAITING_FOR_END_PAGE_CHILD_RESPONSE":
          self.try_answer_question(EndPageQuestionType.SCENE_OBJECT_TAP, label)
      elif self.in_explore_mode():
        self.student_model.update_with_explore_scene_object_tapped(label)
        text_to_say = "That is... " + label
        self.ros.send_jibo_command(JiboStorybookBehaviors.SPEAK,
          text_to_say, .5, .45)

    elif data.event_type == StorybookEvent.SENTENCE_SWIPED:
      message = json.loads(data.message)
      print("SENTENCE_WIPED message for ", message["index"], message["text"])

    elif data.event_type == StorybookEvent.RECORD_AUDIO_COMPLETE:
      message = json.loads(data.message)
      print("RECORD_AUDIO_COMPLETE message for sentence", message["index"])
      # Trigger!
      self.reported_evaluating_sentence_index = message["index"]
      self.child_sentence_audio_complete()

    elif data.event_type == StorybookEvent.STORY_SELECTED:
      print("STORY_SELECTED message received")
      message = json.loads(data.message)
      # TODO: in the future can read message["needs_download"] to see if we
      # potentially need to stall. But we already have a loading page on the
      # tablet app and stalling could cause timing issues with jibo_finish_tts
      # so just don't do any stalling for now.
      
      # Trigger!
      self.storybook_selected()

    elif data.event_type == StorybookEvent.STORY_LOADED:
      print("STORY_LOADED message received")
      # Trigger!
      self.storybook_loaded()

    elif data.event_type == StorybookEvent.CHANGE_MODE:
      print("CHANGE_MODE message received")
      message = json.loads(data.message)
      if int(message["mode"]) == StorybookState.EVALUATE_MODE:
        # Trigger!
        print("New mode is EVALUATE")
        self.begin_evaluate_mode()
      elif int(message["mode"]) == StorybookState.EXPLORE_MODE:
        # Trigger!
        print("New mode is EXPLORE")
        self.begin_explore_mode()

    elif data.event_type == StorybookEvent.REPEAT_END_PAGE_QUESTION:
      print("REPEAT_END_PAGE_QUESTION message received")
      # Trigger
      self.child_request_repeat_end_page_question()


  """
  ROS Message Registered Handlers
  """

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
    print("STORYBOOK_PAGE_INFO message received for page:", data.page_number)
    # Update our knowledge of which page we're on, etc.
    self.current_page_number = data.page_number
    self.current_sentences = data.sentences
    self.current_scene_objects = data.scene_objects
    self.current_tinkertexts = data.tinkertexts

    # Tell student model what sentences are on the page now.
    self.student_model.update_sentences(data.page_number, data.sentences)
    self.student_model.update_scene_objects(data.scene_objects)

    # Trigger!
    self.page_info_received()

  def storybook_state_ros_message_handler(self, data):
    """
       Define the callback function for StorybookState messages.
    """
    self.current_storybook_mode = data.storybook_mode
    self.current_story = data.current_story
    self.num_story_pages = data.num_pages
    # Commented out because we just read it when the audio ends, since that's
    # the only time it changes.
    # self.reported_evaluating_sentence_index = data.evaluating_sentence_index
    self.storybook_audio_playing = data.audio_playing
    self.storybook_audio_file = data.audio_file

    # TODO: Remove this after done testing stuff.
    if data.audio_playing:
      print("Audio is playing in storybook:", data.audio_file)


  def jibo_state_ros_message_handler(self, data):
    """
    Define callback function for when ROS node manager receives a new
    JiboState message, which will be at 10Hz.
    """
    if self.jibo_tts_on and data.tts_msg == "":
      # This is when the sound has just stopped.
      # Trigger!
      self.jibo_finish_tts()

    # Update state.
    self.jibo_tts_on = (data.tts_msg != "")

  def jibo_asr_ros_message_handler(self, data):
    """
    Define callback function for when ROS node manager receives a new
    JiboAsr message, which should be whenever someone says "Hey, Jibo! ..."
    """
    if data.transcription == "" or data.transcription == ASR_NOSPEECH:
      # If previous state says that Jibo ASR was still ongoing, then this is
      # the point at which we can detect that the person is done speaking.
      if not self.jibo_asr_empty:
        # Trigger!
        self.jibo_finish_child_asr()
      # Update state.
      self.jibo_asr_empty = True
    else:
      print("Got Jibo ASR transcription:", data.transcription)
      if self.jibo_asr_empty:
        # If this is start of a new set of transcriptions (meaning that
        # previous state was silence).
        # Trigger!
        self.jibo_got_new_asr()
      # Update state.
      self.jibo_asr_empty = False

      # Handle the response received, depending on the state.
      if self.state == "WAITING_FOR_END_PAGE_CHILD_RESPONSE":
        self.try_answer_question(EndPageQuestionType.WORD_PRONOUNCE, data.transcription)
      
      # TODO: if we're in explore mode, this might be a question from the child,
      # and we'll need to respond to it accordingly.
      # Ideally will be able to just read the stop words and have simple cases.


  """
  Triggers
  """
  def storybook_selected(self):
    print("trigger: storybook_selected")

  def storybook_loaded(self):
    print("trigger: storybook_loaded")

  def page_info_received(self):
    print("trigger: page_info_received")

  def child_audio_timeout(self):
    print("trigger: child_audio_timeout")

  def child_sentence_audio_complete(self):
    print("trigger: child_sentence_audio_complete")

  def jibo_finish_tts(self):
    print("trigger: jibo_finish_tts")

  def child_end_page_question_timeout(self):
    print("trigger: child_end_page_question_timeout")

  def child_request_repeat_end_page_question(self):
    print("trigger: child_request_repeat_end_page_question")

  def child_end_page_got_answer(self):
    print("trigger: child_end_page_got_answer")

  def end_page_jibo_response_complete(self):
    print("trigger: end_page_jibo_response_complete")

  def jibo_got_new_asr(self):
    # When asr goes from silence to not silence.
    print("trigger: jibo_got_new_asr")

  def jibo_finish_child_asr(self):
    # When asr goes from not silence to silence.
    print("trigger: jibo_finish_child_asr")

  def begin_evaluate_mode(self):
    print("trigger: begin_evaluate_mode")

  def begin_explore_mode(self):
    print("trigger: begin_explore_mode")

  # Timer handlers.
  def child_audio_timer_expire_handler(self):
    print("audio timer expired!")
    # Trigger!
    self.child_audio_timeout()

  def child_end_page_question_timer_expire_handler(self):
    print("end page question timer expired!")
    # Shouldn't need this check, but there might be a chance that
    # the the timer gets started when jibo_finish_tts fires
    # after Jibo's done responding to the child but the state
    # hasn't changed yet.
    if self.state in ["WAITING_FOR_END_PAGE_JIBO_QUESTION","WAITING_FOR_END_PAGE_CHILD_RESPONSE"]:
      # Trigger!
      self.child_end_page_question_timeout()
    else:
      print("Does this happen? Not in correct state when timer expires.")


  """
  Actions
  """
  def tablet_show_library_panel(self):
    print("action: tablet_show_library_panel")
    self.ros.send_storybook_command(StorybookCommand.SHOW_LIBRARY_PANEL)

  def tablet_set_evaluate_mode(self):
    print("action: tablet_set_evaluate_mode")
    self.ros.send_storybook_command(StorybookCommand.SET_STORYBOOK_MODE, {"mode": StorybookState.EVALUATE_MODE})

  def jibo_start_story(self):
    print("action: jibo_start_story: ")
    self.ros.send_jibo_command(JiboStorybookBehaviors.HAPPY_ANIM)
    self.ros.send_jibo_command(JiboStorybookBehaviors.SPEAK,
      "Great, it's time to start, I'm so excited!") # I would love it if you would read to me! Every time a sentence appears, read it as best as you can, then click the blue button to see the next sentence. Ready? Let's go!")

  def tablet_next_page(self):
    print("action: tablet_next_page")
    self.reported_evaluating_sentence_index = -1
    self.ros.send_storybook_command(StorybookCommand.NEXT_PAGE)

  def jibo_next_page(self):
    print("action: jibo_next_page")
    self.ros.send_jibo_command(JiboStorybookBehaviors.HAPPY_ANIM)
    self.ros.send_jibo_command(JiboStorybookBehaviors.SPEAK,
      "Ok, moving on to the next page!")

  def tablet_show_next_sentence(self):
    print("action: tablet_show_next_sentence")
    if self.current_page_number <= 0:
      # Fail fast!
      print("Not on a page with words to show, cannot show next sentence")
      sys.exit(1)
    sentence_index = self.reported_evaluating_sentence_index + 1
    if sentence_index < 0 or sentence_index >= len(self.current_sentences):
      # This means that the latest storybook state hasn't updated
      # evaluating_stanza_index back to -1 on a new page.
      print("Sentence index out of range:", sentence_index, len(self.current_sentences))
      sys.exit(1)

    child_turn = self.student_model.is_child_turn(sentence_index)
    should_record = self.current_storybook_mode == StorybookState.EVALUATE_MODE
    params = {
      "index": sentence_index,
      "child_turn": child_turn,
      "record": should_record
    }
    self.ros.send_storybook_command(StorybookCommand.SHOW_NEXT_SENTENCE, params)

  def tablet_begin_record(self):
    print("action: tablet_begin_record")
    self.ros.send_storybook_command(StorybookCommand.START_RECORD)

  def tablet_stop_and_discard_record(self):
    print("action: tablet_stop_and_discard_record") # No speechace should be sent, not uploaded to S3 either.
    self.ros.send_storybook_command(StorybookCommand.CANCEL_RECORD)

  def start_child_audio_timer(self):
    print("action: start_child_audio_timer")
    self.child_audio_evaluate_timer = threading.Timer(
      self.CHILD_AUDIO_SILENCE_TIMEOUT_SECONDS, self.child_audio_timer_expire_handler)
    self.child_audio_evaluate_timer.start()

  def stop_child_audio_timer(self):
    print("action: stop_child_audio_timer")
    self.child_audio_evaluate_timer.cancel()

  def jibo_reprompt_child_audio(self):
    print("action: jibo_reprompt_child_audio")
    self.ros.send_jibo_command(JiboStorybookBehaviors.HAPPY_ANIM)
    self.ros.send_jibo_command(JiboStorybookBehaviors.SPEAK, "Come on, give it a try. It's tough, but you can do it! And don't forget to press the button to move on!")

  def send_end_page_prompt(self):
    print("action: send_end_page_prompt")
    self.end_page_questions = self.student_model.get_end_page_questions()
    self.end_page_question_idx = 0
    self.end_page_questions[self.end_page_question_idx].ask_question(self.ros)
   
  def start_child_end_page_question_timer(self):
    print("action: start_child_end_page_question_timer")
    self.child_end_page_question_timer = threading.Timer(
      self.CHILD_END_PAGE_QUESTION_TIMEOUT_SECONDS,
      self.child_end_page_question_timer_expire_handler)
    self.child_end_page_question_timer.start()

  def stop_child_end_page_question_timer(self):
    print("action: stop_child_end_page_question_timer")
    self.child_end_page_question_timer.cancel()

  def resend_end_page_prompt(self):
    print("action: resend_end_page_prompt")
    self.end_page_questions[self.end_page_question_idx].ask_question(self.ros)

  def jibo_end_page_response_to_child(self):
    print("action: jibo_end_page_response_to_child")
    # Delay before, otherwise seems too rushed.
    time.sleep(1.5)
    self.current_end_page_question().respond_to_child(self.ros)
    print("done with end page response")
    # Reset the current end page questions. TODO: maybe just want to increment the index in the future
    # when we want to ask multiple questions.
    self.end_page_questions = []
    self.end_page_question_idx = None

  def delay_after_end_page_jibo_response(self):
    print("delay_after_end_page_jibo_response")
    # Called after Jibo has finished saying its response.
    # Without this, child doesn't have time to process the correct answer.
    time.sleep(2)

  def tablet_go_to_end_page(self):
    print("action: tablet_go_to_end_page")
    self.ros.send_storybook_command(StorybookCommand.GO_TO_END_PAGE)

  def jibo_end_story(self):
    print("action: jibo_end_story")
    self.ros.send_jibo_command(JiboStorybookBehaviors.HAPPY_ANIM)
    self.ros.send_jibo_command(JiboStorybookBehaviors.SPEAK, "Wow, what a great story! I want to know, what was your favorite part?")
    self.ros.send_jibo_command(JiboStorybookBehaviors.QUESTION_ANIM)

  def jibo_respond_to_end_story(self):
    print("action: jibo_respond_to_end_story")
    self.ros.send_jibo_command(JiboStorybookBehaviors.HAPPY_ANIM)
    self.ros.send_jibo_command(JiboStorybookBehaviors.SPEAK, "Cool, yeah, that's an interesting point. That was fun really really fun!! I hope we can read again some time.")
    self.ros.send_jibo_command(JiboStorybookBehaviors.HAPPY_DANCE)

  """
  Conditions
  """

  def more_sentences_available(self):
    available = self.reported_evaluating_sentence_index + 1 \
      < len(self.current_sentences)
    print("condition: more_sentences_available:", available)
    return available

  def more_pages_available(self):
    available = self.current_page_number + 1 < self.num_story_pages
    print("condition: more_pages_available:", available)
    return available

  def is_last_page(self):
    last = self.current_page_number == self.num_story_pages
    print("condition: is_last_page:", last)
    return last

  def in_not_reading_mode(self):
    not_reading_mode = self.current_storybook_mode == StorybookState.NOT_READING
    print("condition: in_not_reading_mode:", not_reading_mode)
    return not_reading_mode

  def in_explore_mode(self):
    explore_mode = self.current_storybook_mode == StorybookState.EXPLORE_MODE
    print("condition: in_explore_mode:", explore_mode)
    return explore_mode

  def in_evaluate_mode(self):
    evaluate_mode = self.current_storybook_mode == StorybookState.EVALUATE_MODE
    print("condition: in_evaluate_mode:", evaluate_mode)
    return evaluate_mode


  """
  Helpers
  """

  def current_end_page_question(self):
    """
    Helper to return the current end_page_question.
    """
    if self.end_page_question_idx is None or self.end_page_question_idx < 0 or \
    self.end_page_question_idx >= len(self.end_page_questions):
      raise Exception("No end page question exists right now!")

    return self.end_page_questions[self.end_page_question_idx]

  def try_answer_question(self, question_type, query):
    if question_type != self.current_end_page_question().question_type:
      return

    self.current_end_page_question().try_answer(query, self.student_model)
    # Trigger!
    print("About to trigger")
    self.child_end_page_got_answer()
