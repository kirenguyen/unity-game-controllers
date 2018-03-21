from transitions import Machine
import random

class StorybookEvaluateFSM(object):

  def __init__(self, child_id):
    self.child_id = child_id

    self.evaluate_states = ["BEGIN_EVALUATE",
                            "WAITING_FOR_STORY_LOAD",
                            "WAITING_FOR_NEXT_PAGE",
                            "WAITING_FOR_CHILD_AUDIO",
                            "WAITING_FOR_END_PAGE_JIBO_QUESTION",
                            "WAITING_FOR_END_PAGE_CHILD_RESPONSE",
                            "END_STORY",
                            "WAITING_FOR_END_STORY_CHILD_AUDIO",
                            "END_EVALUATE"]
                            
    self.states = self.evaluate_states
    self.initial_state = "END_STORY"

    self.transitions = [
      {
        "trigger":"storybook_selected", # After assets have downloaded 
        "source":"BEGIN_EVALUATE",
        "dest": "WAITING_FOR_STORY_LOAD",
        "before":"jibo_stall_story_loading" # Keep this short.
      },
      {
        "trigger":"storybook_loaded", # At this point, we're on the title screen.
        "source": "WAITING_FOR_STORY_LOAD",
        "dest": "WAITING_FOR_STORY_LOAD",
        "before": "jibo_start_story",
      },
      {
        "trigger":"jibo_finish_tts",
        "source": "WAITING_FOR_STORY_LOAD",
        "dest": "WAITING_FOR_NEXT_PAGE",
        "after":"tablet_next_page" # Go to the first page.
      },
      {
        "trigger":"page_info_received",
        "source":"WAITING_FOR_NEXT_PAGE",
        "dest": "WAITING_FOR_CHILD_AUDIO",
        "after": ["tablet_show_next_sentence", "tablet_begin_record", "start_child_audio_timer"],
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
        "after": ["start_child_audio_timer", "tablet_begin_record"],
      },
      {
        "trigger":"child_sentence_audio_complete",
        "source":"WAITING_FOR_CHILD_AUDIO",
        "dest": "WAITING_FOR_CHILD_AUDIO",
        # "before":"", # TODO: allow Jibo to respond? "Good job!"
        "after": ["tablet_show_next_sentence", "start_child_audio_timer"],
        "conditions": ["more_sentences_available"]
      },
      {
        "trigger":"child_sentence_audio_complete",
        "source":"WAITING_FOR_CHILD_AUDIO",
        "dest": "WAITING_FOR_END_PAGE_JIBO_QUESTION",
        "before":"stop_child_audio_timer",
        "after": "send_end_page_prompt", # Could involve commands to Jibo and tablet.
      },
      {
        "trigger":"jibo_finish_tts",
        "source":"WAITING_FOR_END_PAGE_JIBO_QUESTION",
        "dest": "WAITING_FOR_END_PAGE_CHILD_RESPONSE",
      },
      {
        "trigger":"child_end_page_response_complete",
        "source":"WAITING_FOR_END_PAGE_CHILD_RESPONSE",
        "dest": "WAITING_FOR_NEXT_PAGE",
        "after": "tablet_next_page", # Show the next page button, or navigate to the next page if automatic.
        "conditions": ["more_pages_available"]
      },
      {
        "trigger":"child_end_page_response_complete",
        "source": "WAITING_FOR_END_PAGE_CHILD_RESPONSE",
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
        "after": "jibo_respond_to_end_story"
      },
      {
        "trigger": "jibo_finish_tts",
        "source": "END_EVALUATE",
        "dest": "END_EVALUATE",
        "after": "tablet_show_library_panel"
      },
      {
        "trigger": "begin_evaluate_mode",
        "source": "*",
        "dest": "BEGIN_EVALUATE",
        "after": "tablet_show_library_panel"
      }

    ]


    self.machine = Machine(model=self, initial=self.initial_state,
                           states=self.states, transitions=self.transitions,
                           queued=True)

  """
  Triggers
  """
  def storybook_selected(self):
    print "trigger: storybook_selected"

  def storybook_loaded(self):
    print "trigger: storybook_loaded"

  def page_info_received(self):
    print "trigger: page_info_received"

  def child_audio_timeout(self):
    print "trigger: child_audio_timeout"

  def child_sentence_audio_complete(self):
    print "trigger: child_sentence_audio_complete"

  def jibo_finish_tts(self):
    print "trigger: jibo_finish_tts"

  def child_end_page_response_complete(self):
    print "trigger: child_end_page_response_complete"

  def jibo_finish_child_asr(self):
    print "trigger: jibo_finish_child_asr"

  def begin_evaluate_mode(self):
    print "trigger: begin_evaluate_mode"

  """
  Actions
  """
  def tablet_show_library_panel(self):
    print "action: tablet_show_library_panel"

  def jibo_stall_story_loading(self):
    print "action:jibo_stall_story_loading: 'Ooh I'm so excited, that's a good one!"

  def jibo_start_story(self):
    print "action: jibo_start_story: 'It's time to start! I would love it if you would read to me. Every time a sentence appears, read it as best as you can.'"

  def tablet_next_page(self):
    print "action: tablet_next_page"

  def tablet_show_next_sentence(self):
    print "action: tablet_show_next_sentence"

  def tablet_begin_record(self):
    print "action: tablet_begin_record"

  def tablet_stop_and_discard_record(self):
    print "action: tablet_stop_and_discard_record" # No speechace should be sent, not uploaded to S3 either.

  def start_child_audio_timer(self):
    print "action: start_child_audio_timer"

  def stop_child_audio_timer(self):
    print "action: stop_child_audio_timer"

  def jibo_reprompt_child_audio(self):
    print "action: jibo_reprompt_child_audio: 'Try to read the sentence."

  def send_end_page_prompt(self):
    print "action: send_end_page_prompt:"

  def tablet_go_to_end_page(self):
    print "action: tablet_go_to_end_page"

  def jibo_end_story(self):
    print "action: jibo_end_story: 'Wow that was a great story, what was your favorite part?'"

  def jibo_respond_to_end_story(self):
    print "action: jibo_respond_to_end_story: 'Cool, yeah, that was fun!"

  """
  Conditions
  """

  def more_sentences_available(self):
    available = random.random() < .8
    print "condition: more_sentences_available:", available
    return available

  def more_pages_available(self):
    available = random.random() < .8
    print "condition: more_pages_available:", available
    return available


