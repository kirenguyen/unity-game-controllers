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

from storybook_controller.storybook_fsm_structure import StorybookFSMStructure
from storybook_controller.storybook_constants import *
from storybook_controller.robot_feedback import EndPageQuestionType
from storybook_controller.jibo_commands_builder import JiboStorybookBehaviors

from storybook_controller.jibo_statements import JiboStatements
from storybook_controller.jibo_statements import JiboStatementType

class StorybookFSM(object):
  def __init__(self, ros_node_manager, student_model, participant_id, prev_saved_state):
    # The FSM maintains a queue that ros messages are added to, so that
    # different topics can send their events and they will be processed
    # one at a time, in order.
    self.event_queue = queue.Queue()

    self.ros = ros_node_manager
    self.student_model = student_model

    self.participant_id = participant_id
    self.prev_saved_state = prev_saved_state
    self.should_continue_from_prev_state = False

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

    self.current_story_needs_download = None
    self.explore_pages_read = {}

    # Timer for when we're waiting for the child to read a sentence in
    # evaluate mode.
    self.child_audio_evaluate_timer = None
    self.CHILD_AUDIO_SILENCE_TIMEOUT_SECONDS = 30 # Amount of time after detecting silence before reprompting a read.

    self.child_end_page_question_timer = None
    self.CHILD_END_PAGE_QUESTION_TIMEOUT_SECONDS = 30

    self.child_explore_page_timer = None
    self.CHILD_EXPLORE_PAGE_TIMEOUT_SECONDS = 6

    self.eos_timer = None
    self.EOS_TIMEOUT_SECONDS = .5

    # We get ASR results incrementally, this should accumulate the results until
    # we detect end of speech.
    self.streaming_transcription = ""

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
      StorybookEvent.END_STORY, # Only received in explore mode.
      StorybookEvent.RETURN_TO_LIBRARY_EARLY, # Only in explore mode.
      StorybookEvent.CHILD_READ_STORY_START, # Only in explore mode.
      StorybookEvent.JIBO_READ_STORY_REQUEST, # Only in explore mode.
    ]

    self.states = StorybookFSMStructure.states
    self.initial_state = StorybookFSMStructure.initial_state
    self.transitions = StorybookFSMStructure.all_transitions

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
        self.event_queue.task_done()
      except queue.Empty:
        time.sleep(.05)

  def process_storybook_event(self, data):
    if data.event_type == StorybookEvent.HELLO_WORLD:
      print("HELLO_WORLD message received")

      # Wait for a little bit to ensure message not dropped.
      time.sleep(1)
      print("Sending ack")
      params = {
        "participant_id": self.participant_id,
        "story_name": ""
      }
      # If we were previously in evaluate mode, tell the app that so it
      # knows to display the option to pick up where we left off.
      if self.prev_saved_state is not None:
        if self.prev_saved_state["storybook_mode"] == StorybookState.EVALUATE_MODE:
          if self.prev_saved_state["page_number"] is not None and self.prev_saved_state["page_number"] > 0:
            params["story_name"] = self.prev_saved_state["story_name"]
      command = StorybookCommand.HELLO_WORLD_ACK
      self.ros.send_storybook_command(command, params)

    elif data.event_type == StorybookEvent.SPEECH_ACE_RESULT:
      print("SPEECH_ACE_RESULT message received")
      message = json.loads(data.message)
      page_num = message["page_num"]
      sentence_index = message["index"]
      text = message["text"]
      duration = message["duration"]
      speechace_result = json.loads(message["speechace"])
      self.student_model.update_with_duration(duration, text)
      self.student_model.update_with_speechace_result(page_num, sentence_index, speechace_result)
    
    elif data.event_type == StorybookEvent.WORD_TAPPED:
      message = json.loads(data.message)
      word = message["word"].lower()
      phrase = message["phrase"].lower()
      print("WORD_TAPPED message received, word is", word, "phrase is", phrase, 
        "state is", self.state)
      if self.in_evaluate_mode():
        if self.state == "WAITING_FOR_END_PAGE_CHILD_RESPONSE":
          self.try_answer_question(EndPageQuestionType.WORD_TAP, word)
      elif self.responding_to_tablet_interactions():
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
      elif self.responding_to_tablet_interactions():
        self.student_model.update_with_explore_scene_object_tapped(label)
        text_to_say = "That is... " + label
        self.ros.send_jibo_command(JiboStorybookBehaviors.SPEAK,
          text_to_say, .5, .45)

    elif data.event_type == StorybookEvent.SENTENCE_SWIPED:
      message = json.loads(data.message)
      print("SENTENCE_SWIPED message for ", message["index"], message["text"])

    elif data.event_type == StorybookEvent.RECORD_AUDIO_COMPLETE:
      message = json.loads(data.message)
      print("RECORD_AUDIO_COMPLETE message for sentence", message["index"])
      # Trigger!
      self.reported_evaluating_sentence_index = message["index"]
      self.child_read_audio_complete()

      if self.in_evaluate_mode():
        if self.state == "WAITING_FOR_END_PAGE_CHILD_RESPONSE":
          self.try_answer_question(EndPageQuestionType.REREAD_SENTENCE, self.streaming_transcription)

    elif data.event_type == StorybookEvent.STORY_SELECTED:
      print("STORY_SELECTED message received")
      message = json.loads(data.message)
      # TODO: in the future can read message["needs_download"] to see if we
      # potentially need to stall. But we already have a loading page on the
      # tablet app and stalling could cause timing issues with jibo_finish_tts
      # so just don't do any stalling for now.
      self.current_story_needs_download = message["needs_download"]
      
      # Get target words for this story. If there are none, student model
      # will use the hardcoded target words instead, but that is not good,
      # after updating the Unity side, every story should have target words.
      if "target_words" in message and len(message["target_words"]) > 0:
        self.student_model.update_target_words(message["target_words"])
      else:
        self.student_model.update_target_words(None)

      # Clear the dictionary.
      self.explore_pages_read = {}

      # Trigger!
      self.storybook_selected()

    elif data.event_type == StorybookEvent.STORY_LOADED:
      # Story loaded occurs after story selected, it means that any assets
      # have successfully been loaded and the story is waiting on the
      # title page ready to go.
      print("STORY_LOADED message received")
      message = json.loads(data.message)
      # Trigger!
      if message["continue_midway"]:
        self.should_continue_from_prev_state = True
        self.storybook_loaded_with_continue()
      else:
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

    elif data.event_type == StorybookEvent.END_STORY:
      print("END_STORY message received")
      # Trigger
      self.tablet_explore_end_story()

    elif data.event_type == StorybookEvent.RETURN_TO_LIBRARY_EARLY:
      print("RETURN_TO_LIBRARY_EARLY message received")
      # Trigger
      self.begin_explore_mode()

    elif data.event_type == StorybookEvent.EXPLORE_CHILD_READ_START:
      print("EXPLORE_CHILD_READ_START")
      self.explore_child_read_start()

    elif data.event_type == StorybookEvent.JIBO_READ_STORY_REQUEST:
      print("JIBO_READ_STORY_REQUEST message received")
      self.explore_jibo_read_requested()


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

    # Clear the end page questions, reset index to 0.
    self.end_page_questions = []
    self.end_page_question_idx = 0 

    print("got prompts: ", data.prompts)

    # Tell student model what sentences are on the page now.
    self.student_model.update_current_page(self.current_story, data.page_number, data.sentences, data.scene_objects, data.prompts)

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
    self.reported_evaluating_sentence_index = data.evaluating_sentence_index
    self.storybook_audio_playing = data.audio_playing
    self.storybook_audio_file = data.audio_file

  def jibo_state_ros_message_handler(self, data):
    """
    Define callback function for when ROS node manager receives a new
    JiboState message, which will be at 10Hz.
    """
    if self.jibo_tts_on and data.tts_msg == "":
      # This is when the sound has just stopped.
      # Trigger!
      self.jibo_finish_tts()

    # Check that the last tts command we sent actually occured.
    self.ros.report_jibo_tts_received(data.tts_msg)

    # Update state.
    self.jibo_tts_on = (data.tts_msg != "")

  def jibo_asr_ros_message_handler(self, data):
    """
    Define callback function for when ROS node manager receives a new
    JiboAsrResult message."
    """
    if data.transcription == "" or data.transcription == ASR_NOSPEECH:
      # If previous state says that Jibo ASR was still ongoing, then this is
      # the point at which we can start detecting that the person is done speaking.
      # if not self.jibo_asr_empty:
      #   # Trigger!
      #   self.start_eos_timer()
      # # Update state.
      # self.jibo_asr_empty = True
      pass
    else:
      print("Got Jibo ASR transcription:", data.transcription)
      self.asr_received()
      # If we get nothing after a certain amount of time after this, then assume end of speech.
      self.start_eos_timer()

      # Check for "I don't know"
      if data.slotAction == "idk":
        self.asr_idk_received()
      elif data.slotAction == "help":
        self.asr_help_received()
      elif data.slotAction == "wake_up":
        self.wake_up_received()

      self.streaming_transcription += data.transcription

      # Update state.
      # self.jibo_asr_empty = False
      
      # TODO: if we're in explore mode, this might be a question from the child,
      # and we'll need to respond to it accordingly.
      # Ideally will be able to just read the slotAction from ASR and have
      # simple cases to handle each one.


  """
  Triggers
  """
  def wake_up_received(self):
    print("trigger: wake_up_received")

  def asr_idk_received(self):
    print("trigger: asr_idk_received")

  def asr_help_received(self):
    print("trigger: asr_help_received")

  def storybook_selected(self):
    print("trigger: storybook_selected")

  def storybook_loaded(self):
    print("trigger: storybook_loaded")

  def storybook_loaded_with_continue(self):
    print("trigger: storybook_loaded_with_continue")

  def page_info_received(self):
    print("trigger: page_info_received")

  def child_audio_timeout(self):
    print("trigger: child_audio_timeout")

  def child_explore_page_timeout(self):
    print("trigger: child_explore_page_timeout")

  def child_read_audio_complete(self):
    print("trigger: child_read_audio_complete")

  def jibo_finish_tts(self):
    print("trigger: jibo_finish_tts")

  def child_end_page_question_timeout(self):
    print("trigger: child_end_page_question_timeout")

  def child_request_repeat_end_page_question(self):
    print("trigger: child_request_repeat_end_page_question")

  def tablet_explore_end_story(self):
    print("trigger: tablet_explore_end_story")

  def child_end_page_got_answer(self):
    print("trigger: child_end_page_got_answer")

  def end_page_jibo_response_complete(self):
    print("trigger: end_page_jibo_response_complete")

  def no_questions_go_to_next_page(self):
    print("trigger: no_questions_go_to_next_page")

  def asr_received(self):
    # When asr goes from silence to not silence.
    print("trigger: asr_received")
    # Check for state and stop timers.
    self.stop_eos_timer()
    if self.state == "WAITING_FOR_CHILD_AUDIO":
      self.start_child_audio_timer()
    elif self.state == "WAITING_FOR_END_PAGE_CHILD_RESPONSE":
      self.start_child_end_page_question_timer()

  def jibo_finish_child_asr(self):
    # When asr goes from not silence to silence.
    print("trigger: jibo_finish_child_asr")

  def begin_evaluate_mode(self):
    print("trigger: begin_evaluate_mode")
    self.clear_current_story_info()

  def begin_explore_mode(self):
    print("trigger: begin_explore_mode")
    self.clear_current_story_info()

  """
  Timer handlers.
  """
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


  def eos_timer_expire_handler(self):
    print("eos timer expired!")
    self.jibo_finish_child_asr()

    if self.state == "WAITING_FOR_END_PAGE_CHILD_RESPONSE":
      self.try_answer_question(EndPageQuestionType.SPEECH_REQUESTED, self.streaming_transcription)
      self.try_answer_question(EndPageQuestionType.WORD_PRONUNCIATION, self.streaming_transcription)

  def child_explore_page_timer_expire_handler(self):
    print("explore page timer expired!")
    # Trigger!
    self.child_explore_page_timeout()

  """
  Actions
  """
  def jibo_wake_up_and_welcome(self):
    print("action: jibo_wake_up_and_welcome")
    jibo_text = "<style set='enthusiastic'> Oh hello! </style> <es cat='happy'/> Sorry I must've been asleep. I don't think I've met you before, <style set='enthusiastic'> I'm Jibo! So, is it storytime now? <break size='0.5'/> <style set='enthusiastic'> I love stories. </style>"

    self.ros.send_jibo_command(JiboStorybookBehaviors.SPEAK, jibo_text)

  def jibo_stall_before_story(self):
    print("action: jibo_stall_before_story")
    if self.current_story_needs_download:
      self.ros.send_jibo_command(JiboStorybookBehaviors.SPEAK,
        "I have a feeling this is a good story! <es cat='happy'/> Let's go!")

  def tablet_show_library_panel(self):
    print("action: tablet_show_library_panel")
    self.ros.send_storybook_command(StorybookCommand.SHOW_LIBRARY_PANEL)

  def tablet_set_evaluate_mode(self):
    print("action: tablet_set_evaluate_mode")
    self.ros.send_storybook_command(StorybookCommand.SET_STORYBOOK_MODE, {"mode": StorybookState.EVALUATE_MODE})

  def jibo_start_story(self):
    print("action: jibo_start_story")
    self.ros.send_jibo_command(JiboStorybookBehaviors.SPEAK,
      "<style set='enthusiastic'> <es cat='happy'> Great, it's time to start, I'm so excited! </es> I would love it if you would read to me! </style> <break size='.3'/> Every time a sentence appears, read it as best as you can, then click the blue button to see the next sentence. <break size='1'/> Ready? Let's go!")

  def jibo_continue_story(self):
    print("action: jibo_continue_story")
    self.ros.send_jibo_command(JiboStorybookBehaviors.SPEAK,
      "<style set='confused'> Oops, <break size='.3'/> <es cat='confused'> I must have gotten confused. </es> <break size='.7'/> </style> <style set='enthusiastic'> Let's pick up where we left off. Remember, reed the sentences as they appear, and don't be afraid to ask for help. Let's go! </style>")

  def load_previous_stored_state(self):
    print("action: load_previous_stored_state")
    # This restores any state that we need for the state machine and student model.
    # For now, just print something.
    # TODO
    print("done loading previous stored state")

  def start_child_explore_page_timer(self):
    print("action: start_child_explore_page_timer")
    # Don't do anything yet.
    return
    self.child_explore_page_timer = threading.Timer(
      self.CHILD_EXPLORE_PAGE_TIMEOUT_SECONDS, self.child_explore_page_timer_expire_handler)
    self.child_explore_page_timer.start()

  def stop_child_explore_page_timer(self):
    print("action: stop_child_explore_page_timer")
    self.child_explore_page_timer.cancel()

  def jibo_prompt_explore_page(self):
    print("action: jibo_prompt_explore_page")
    jibo_text = None
    if self.current_explore_page_was_read:
      jibo_text = "Let's explore! You can tap words, tap the picture, or swipe on sentences! " + \
                  "When you're ready, move on to the next page."
    else:
      jibo_text = "Let's read! Click on a button for either me or you to read this page."
    self.ros.send_jibo_command(JiboStorybookBehaviors.SPEAK,
      )

  def tablet_next_page(self):
    print("action: tablet_next_page")
    self.reported_evaluating_sentence_index = -1
    self.ros.send_storybook_command(StorybookCommand.NEXT_PAGE)

  def tablet_continue_from_stored_page(self):
    print("action: tablet_continue_from_stored_page")
    self.reported_evaluating_sentence_index = -1
    if self.prev_saved_state is None:
      raise Exception("Impossible, requesting to continue from page when there is no saved state")
    params = {
      "page_number": self.prev_saved_state["page_number"]
    }
    self.ros.send_storybook_command(StorybookCommand.GO_TO_PAGE, params)

  def jibo_next_page(self):
    print("action: jibo_next_page")
    self.ros.send_jibo_command(JiboStorybookBehaviors.SPEAK,
      "<es cat='happy'>Ok, moving on to the next page! </es>")

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

  def stop_child_audio_timer(self):
    print("action: stop_child_audio_timer")
    # TODO: thinking about removing this timer entirely.
    return
    self.child_audio_evaluate_timer.cancel()

  def jibo_reprompt_child_audio(self):
    print("action: jibo_reprompt_child_audio")
    # TODO: if it's the second time or greater, tell them to say I need help
    self.ros.send_jibo_command(JiboStorybookBehaviors.SPEAK, "<es cat='happy'/>Come on, give it a try. And don't forget to press the button to move on! But if you're really stuck, just say <break size='.5'/> I need help <break size='.3'/> and I'll help you.")

  def jibo_help_with_current_sentence(self):
    """
    When child expresses that they don't know how to read the current sentence.
    """
    print("action: jibo_help_with_current_sentence")
    sentence = self.current_sentences[self.reported_evaluating_sentence_index]
    # TODO: Use JiboStatements to vary the beginning of this text.
    pre_sentence_prompt = JiboStatements.get_statement(JiboStatementType.PRE_HELP_WITH_SENTENCE)
    post_sentence_prompt = "<break size='.7'/> Now you try to read the sentence again! Go ahead."
    # Make sure to say the sentence slowly!
    jibo_text = pre_sentence_prompt + " <break size='.5'/> This sentence says <break size='.8/>' <duration stretch='1.3'>" + sentence + ". </duration>" + post_sentence_prompt
    self.ros.send_jibo_command(JiboStorybookBehaviors.SPEAK, jibo_text)

  def send_end_page_prompt(self):
    print("action: send_end_page_prompt")
    if self.end_page_question_idx == 0:
      self.end_page_questions = self.student_model.get_end_page_questions(prev_times_asked=0)
    if len(self.end_page_questions) == 0:
      # Trigger!
      self.no_questions_go_to_next_page()
    else:
      self.end_page_questions[self.end_page_question_idx].ask_question(self.ros)

  def start_child_end_page_question_timer(self):
    print("action: start_child_end_page_question_timer")

    # Stop timer first.
    if self.child_end_page_question_timer is not None and self.child_end_page_question_timer.is_alive():
      self.child_end_page_question_timer.cancel()

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
    """
    For when child has attempted to answer the asked question.
    """
    print("action: jibo_end_page_response_to_child")
    self.jibo_end_page_respond_to_child_helper(False)

  def jibo_end_page_response_to_child_idk(self):
    """
    For when the child expresses that they don't know the answer.
    """
    print("action: jibo_end_page_response_to_child_idk")
    self.jibo_end_page_respond_to_child_helper(True)

  def tablet_go_to_end_page(self):
    print("action: tablet_go_to_end_page")
    self.ros.send_storybook_command(StorybookCommand.GO_TO_END_PAGE)

  def jibo_end_story(self):
    print("action: jibo_end_story")
    self.ros.send_jibo_command(JiboStorybookBehaviors.SPEAK, "<style set='enthusiastic'> Wow! What a great story! </style> <break size='.5'/> <style set='curious'> I want to know, what was your favorite part? </style>")

  def jibo_respond_to_end_story(self):
    print("action: jibo_respond_to_end_story")
    self.ros.send_jibo_command(JiboStorybookBehaviors.SPEAK, "Cool, <es cat='happy'/> yeah that's an interesting point. That was fun really really fun!! I hope we can read again some time.")

  def start_waiting_for_child_response(self):
    print("action: start_waiting_for_child_response")
    rule = "rules/storybook.fst" # This rule checks for variants of "I don't know"
    self.ros.send_jibo_asr_command(JiboAsrCommand.START, rule)
    self.start_child_end_page_question_timer()
    # Every time we want to start listening for child response.
    self.streaming_transcription = ""
    # If the current question is waiting for re-read sentence, start the recording.
    if self.current_end_page_question().question_type == EndPageQuestionType.REREAD_SENTENCE:
      params = {
        "index": self.current_end_page_question().sentence_index
      }
      self.ros.send_storybook_command(StorybookCommand.START_RECORD, params)

  def start_listening_for_child_read_sentence(self):
    print("action: start_listening_for_child_read_sentence")
    rule = "TopRule = $* $HELP {%slotAction='help'%} $*; HELP = (help me) | (you help) | (some help) | (want help) | (need help) | (read for me) | (you read) | (read it) | (read this sentence) | (assist) | (tell me) | (need helpful);"
    # Start ASR with Hey Jibo required, so that the child needs to explicitly ask for help.
    # TODO: figure out why asking for Hey, Jibo doesn't work.
    self.ros.send_jibo_asr_command(JiboAsrCommand.START, rule)
    self.start_child_audio_timer()
    self.streaming_transcription = ""

  def stop_waiting_for_child_response(self):
    print("action: stop_waiting_for_child_response")
    self.stop_jibo_asr()
    self.stop_child_end_page_question_timer()
    self.ros.send_storybook_command(StorybookCommand.CANCEL_RECORD, {})

  def stop_listening_for_child_read_sentence(self):
    print("action: stop_listening_for_child_read_sentence")
    self.stop_jibo_asr()
    self.stop_child_audio_timer()

  def stop_jibo_asr(self):
    print("action: stop_jibo_asr")
    self.ros.send_jibo_asr_command(JiboAsrCommand.STOP)

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

  def more_end_page_questions_available(self):
    # At this point, the index has already been updated, so we just check if the
    # current index is still in range.
    available = self.end_page_question_idx < len(self.end_page_questions)
    print("condition: more_end_page_questions_available:", available)
    return available

  def continue_from_prev_state(self):
    print("condition: should_continue_from_prev_state", self.should_continue_from_prev_state)
    return self.should_continue_from_prev_state

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
   
  def start_eos_timer(self):
    print("start eos timer")
    if self.eos_timer is not None and self.eos_timer.is_alive():
      self.eos_timer.cancel()
    self.eos_timer = threading.Timer(
      self.EOS_TIMEOUT_SECONDS, self.eos_timer_expire_handler)
    self.eos_timer.start()

  def stop_eos_timer(self):
    self.eos_timer.cancel()

  def start_child_audio_timer(self):
    # TODO: I don't think we need this timer, we'll just wait for the kid to press button
    # or ask for help.
    return 
    # Stop it first.
    if self.child_audio_evaluate_timer is not None and self.child_audio_evaluate_timer.is_alive():
      self.child_audio_evaluate_timer.cancel()

    self.child_audio_evaluate_timer = threading.Timer(
      self.CHILD_AUDIO_SILENCE_TIMEOUT_SECONDS, self.child_audio_timer_expire_handler)
    self.child_audio_evaluate_timer.start()

  def clear_current_story_info(self):
    """
    This way when we save our state we know if we were in the middle of reading a story
    or not. If we don't clear this, then we can't tell if we were done or if were
    actually paused on the last page of the story.
    """
    self.current_storybook_mode = None
    self.current_story = None
    self.current_page_number = None

  def responding_to_tablet_interactions(self):
    """
    Returns true if we should be responding to child interacting (tapping/swiping)
    with the tablet.
    """
    return self.state == "EXPLORING_PAGE"

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
    print("answering question index", self.end_page_question_idx)
    self.current_end_page_question().try_answer(query, self.student_model)
    # Trigger!
    self.child_end_page_got_answer()

  def jibo_end_page_respond_to_child_helper(self, idk):
    # Update questions.
    self.end_page_questions = self.student_model.get_end_page_questions(self.end_page_question_idx + 1)

    # Add special case for pronunciation questions. If it was incorrect, add another question
    # into the queue to be asked immediately after.
    q = self.current_end_page_question()
    if q.question_type == EndPageQuestionType.WORD_PRONUNCIATION:
      if not q.correct and not q.already_asked:
        self.end_page_questions.insert(self.end_page_question_idx + 1, q.clone_for_repronounce())

    self.current_end_page_question().respond_to_child(self.ros, idk)
    print("Done with end page response, idk was", idk, "incrementing end_page_question_idx")
    self.end_page_question_idx += 1

