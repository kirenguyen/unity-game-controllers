"""
Define the structure of the fsm without any of the methods filled in.
"""

class StorybookFSMStructure(object):
  
  not_reading_states = ["APP_START"]

  explore_states = [
    "BEGIN_EXPLORE",
    "EXPLORING_PAGE", # Idle state while on a page.
    "WAITING_FOR_EXPLORE_CHILD_READ", # Child wants to read, we are waiting.
    "WAITING_FOR_EXPLORE_JIBO_READ", # Jibo requested to read, waiting for Jibo.
    "WAITING_FOR_JIBO_EXPLORE_PROMPT", # Jibo is giving a prompt.
    "END_EXPLORE" # End explore, immediately go back to BEGIN_EXPLORE.
  ]

  evaluate_states = [
    "BEGIN_EVALUATE", # After switching to evaluate mode.
    "WAITING_FOR_STORY_LOAD", # After a story selected and is loading.
    "WAITING_FOR_NEXT_PAGE", # After story loaded, waiting for page_info.
    "WAITING_FOR_CHILD_AUDIO", # After a sentence has been shown.
    "WAITING_FOR_JIBO_REPROMPT_CHILD_AUDIO", # Waiting for Jibo to prompt child.
    "WAITING_FOR_JIBO_HELP_WITH_SENTENCE", # While Jibo is helping the child.
    "WAITING_FOR_END_PAGE_JIBO_QUESTION", # Wait for Jibo tts to finish.
    "WAITING_FOR_END_PAGE_CHILD_RESPONSE", # Could be speech or tablet event.
    "WAITING_FOR_END_PAGE_JIBO_RESPONSE", # Jibo gives corrections/comments.
    "WAITING_FOR_NEXT_PAGE_JIBO_INTERLUDE", # After end page child response.
    "END_STORY", # Moved to the "The End" page, tell Jibo to ask a question,
    "WAITING_FOR_END_STORY_CHILD_RESPONSE", # Wait for child to respond
                                          # (This can be anywhere not just end of story).
    "END_EVALUATE" # End evaluate mode, immediately go back to BEGIN_EVALUATE.
  ]

  states = not_reading_states + explore_states + evaluate_states

  initial_state = "APP_START"

  not_reading_transitions = [
    {
      "trigger": "asr_wake_up_received",
      "source": "APP_START",
      "dest": "APP_START",
      "after": "jibo_wake_up_and_welcome"
    },
    {
      "trigger": "jibo_finish_tts",
      "source": "APP_START",
      "dest": "APP_START",
      "after": "stop_jibo_asr"
    }
  ]

  explore_transitions = [
    {
      "trigger": "storybook_selected",
      "source": ["BEGIN_EXPLORE"],
      "dest": "WAITING_FOR_STORY_LOAD",
      "after": "jibo_stall_before_story",
      "conditions": ["in_explore_mode"]
    },
    {
      "trigger": "jibo_finish_tts",
      "source": "WAITING_FOR_STORY_LOAD",
      "dest": "EXPLORING_PAGE",
      "conditions": ["in_explore_mode"]
    },
    {
      "trigger": "storybook_loaded",
      "source": "WAITING_FOR_STORY_LOAD",
      "dest": "EXPLORING_PAGE",
      "conditions": ["in_explore_mode"]
    },
    {
      "trigger": "storybook_loaded",
      "source": "EXPLORING_PAGE",
      "dest": "=",
      "conditions": ["in_explore_mode"]
    },
    {
      "trigger": "page_info_received",
      "source": "EXPLORING_PAGE",
      "dest": "EXPLORING_PAGE",
      # Commented out because explore mode does nothing right now. # "after": ["start_child_explore_page_timer"]
    },
    # Child will either read or ask Jibo to read.
    # If Jibo requested to read, then start reading.
    {
      "trigger": "explore_jibo_read_requested",
      "source": "EXPLORING_PAGE",
      "dest": "WAITING_FOR_EXPLORE_JIBO_READ",
      "after": ["jibo_explore_read_page", "stop_child_explore_page_timer"]
    },
    {
      "trigger": "jibo_finish_tts",
      "source": "WAITING_FOR_EXPLORE_JIBO_READ",
      "dest": "EXPLORING_PAGE",
      "after": ["start_child_explore_page_timer"]
    },
    # Child wants to start reading. We start listening.
    {
      "trigger": "explore_child_read_start",
      "source": "EXPLORING_PAGE",
      "dest": "WAITING_FOR_CHILD_AUDIO",
      "after": ["start_child_audio_timer"]
    },
    {
      "trigger": "child_read_audio_complete", # TODO: not sure if this is the correct trigger anymore, it's for when the done reading button is pressed.
      "source": "WAITING_FOR_CHILD_AUDIO",
      "dest": "EXPLORING_PAGE",
      "before": ["start_child_explore_page_timer"],
      "conditions": ["in_explore_mode"]
    },
    # Return to exploring the page.
    {
      "trigger":"end_page_jibo_response_complete",
      "source":"WAITING_FOR_END_PAGE_JIBO_RESPONSE",
      "dest": "EXPLORING_PAGE",
      "conditions": ["in_explore_mode"]
    },
    # While idling on a page, timeout and tell child to do something.
    {
      "trigger": "child_explore_page_timeout",
      "source": "EXPLORING_PAGE",
      "dest": "EXPLORING_PAGE",
      "after": ["jibo_choose_prompt"] # This should be a statement telling child to do something, or a question.
    },
    # Jibo's prompt is either a reprompt, a generic question or an actual question.
    # Case 1: Generic Prompt
    {
      "trigger": "jibo_explore_choose_generic_prompt",
      "source": "EXPLORING_PAGE",
      "dest": "WAITING_FOR_JIBO_EXPLORE_PROMPT",
      "after": "jibo_prompt_explore_page"
    },
    {
      "trigger": "jibo_finish_tts",
      "source": "WAITING_FOR_JIBO_EXPLORE_PROMPT",
      "dest": "EXPLORING_PAGE",
      "after": ["start_child_explore_page_timer"],
    },
    # Case 2: Generic Question (e.g. "How do you like the story so far?"")
    {
      "trigger": "jibo_explore_choose_generic_question",
      "source": "EXPLORING_PAGE",
      "dest": "WAITING_FOR_CHILD_GENERIC_RESPONSE"
    },
    {
      "trigger": "jibo_finish_child_asr", # TODO: make this trigger be after there's been a certain amount of silence
      "source": "WAITING_FOR_CHILD_GENERIC_RESPONSE",
      "dest": "EXPLORING_PAGE",
      "conditions": ["in_explore_mode"]
    },
    # Case 3: Specific Question (e.g. "Can you click on the toad in the picture?")
    {
      "trigger": "jibo_explore_choose_specific_question",
      "source": "EXPLORING_PAGE",
      "dest": "WAITING_FOR_END_PAGE_JIBO_QUESTION",
      "after": "send_end_page_prompt"
    },
    # The logic between WAITING_FOR_END_PAGE_JIBO_QUESTION and WAITING_FOR_END_PAGE_JIBO_RESPONSE
    # is handled in the transitions for evaluate mode.
    {
      "trigger":"end_page_jibo_response_complete",
      "source":"WAITING_FOR_END_PAGE_JIBO_RESPONSE",
      "dest": "EXPLORING_PAGE",
      "after": "start_child_explore_page_timer",
      "conditions": ["in_explore_mode"]
    },
    # When story ends, we use the same jibo_end_story as in evaluate mode.
    {
      "trigger": "tablet_explore_end_story",
      "source": "EXPLORING_PAGE",
      "dest": "END_STORY",
      "after": ["jibo_end_story", "start_waiting_for_child_response"]
    },
    # The logic from END_STORY to WAITING_FOR_CHILD_GENERIC_RESPONSE is handled in evaluate
    # mode transitions. Basically wait for child to say something, generically respond.
    {
      "trigger": "jibo_finish_child_asr",
      "source": "WAITING_FOR_END_STORY_CHILD_RESPONSE",
      "dest": "END_EXPLORE",
      "after": ["jibo_respond_to_end_story", "stop_waiting_for_child_response"],
      "conditions": ["in_explore_mode"]
    },
    {
      "trigger": "jibo_finish_tts",
      "source": "END_EXPLORE",
      "dest": "END_EXPLORE",
      # "after": ["begin_explore_mode"], # This should be handled when user presses home button.
      # "conditions": ["in_explore_mode"]
    },
    # Switching modes.
    {
      "trigger": "begin_explore_mode",
      "source": "*",
      "dest": "BEGIN_EXPLORE"
    },
  ]

  evaluate_transitions = [
    {
      "trigger":"storybook_selected", # After assets have downloaded 
      "source": ["BEGIN_EVALUATE"],
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
    # When we want to pick up from where we left off.
    {
      "trigger": "storybook_loaded_with_continue",
      "source": "WAITING_FOR_STORY_LOAD",
      "dest": "WAITING_FOR_STORY_LOAD",
      "before": ["jibo_continue_story", "load_previous_saved_state"],
      "conditions": ["in_evaluate_mode"]
    },
    {
      "trigger": "jibo_finish_tts",
      "source": "WAITING_FOR_STORY_LOAD",
      "dest": "WAITING_FOR_NEXT_PAGE",
      "before": "tablet_continue_from_stored_page",
      "conditions": ["in_evaluate_mode", "continue_from_prev_state"]
    },
    {
      "trigger":"jibo_finish_tts",
      "source": "WAITING_FOR_STORY_LOAD",
      "dest": "WAITING_FOR_NEXT_PAGE",
      "before":"tablet_next_page", # Go to the first page.
      "conditions": ["in_evaluate_mode"]
    },
    {
      "trigger":"page_info_received",
      "source":"WAITING_FOR_NEXT_PAGE",
      "dest": "WAITING_FOR_CHILD_AUDIO",
      # This action has really become more like start page, because it tells the
      # storybook to show the first sentence, then the storybook handles showing
      # subsequent sentences on its own.
      "after": ["tablet_show_next_sentence", "start_listening_for_child_read_sentence"],
      "conditions": ["in_evaluate_mode"]
    },
    {
      "trigger":"child_audio_timeout",
      "source":"WAITING_FOR_CHILD_AUDIO",
      "dest": "WAITING_FOR_JIBO_REPROMPT_CHILD_AUDIO",
      "before":["jibo_reprompt_child_audio", "tablet_stop_and_discard_record"],
    },
    {
      "trigger":"jibo_finish_tts", # This trigger will be used a lot, but it will have different effects based on source state.
      "source":"WAITING_FOR_JIBO_REPROMPT_CHILD_AUDIO",
      "dest": "WAITING_FOR_CHILD_AUDIO",
      "after": ["tablet_begin_record_for_current_sentence", "start_listening_for_child_read_sentence"], # After Jibo finishes prompting.
    },
    # When the child says they don't know when we're asking them to read a sentence.
    {
      "trigger": "asr_help_received",
      "source": "WAITING_FOR_CHILD_AUDIO",
      "dest": "WAITING_FOR_JIBO_HELP_WITH_SENTENCE",
      "before": ["stop_listening_for_child_read_sentence", "tablet_stop_and_discard_record"],
      "after": ["jibo_help_with_current_sentence"]
    },
    # Here, either trigger the next sentence or go to the end page questions.
    {
      "trigger": "jibo_finish_tts",
      "source": "WAITING_FOR_JIBO_HELP_WITH_SENTENCE",
      "dest": "WAITING_FOR_CHILD_AUDIO",
      "after": ["tablet_begin_record_for_current_sentence", "start_listening_for_child_read_sentence"],
    },
    # This is only for when the done recording button is pressed, so don't touch the asr stuff.
    {
      "trigger":"child_read_audio_complete",
      "source":"WAITING_FOR_CHILD_AUDIO",
      "dest": "WAITING_FOR_CHILD_AUDIO",
      "before": ["start_listening_for_child_read_sentence"],
      "conditions": ["more_sentences_available"]
    },
    {
      "trigger":"child_read_audio_complete",
      "source":"WAITING_FOR_CHILD_AUDIO",
      "dest": "WAITING_FOR_END_PAGE_JIBO_QUESTION",
      "before": ["stop_listening_for_child_read_sentence"],
      "after": "send_end_page_prompt" # Could involve commands to Jibo and tablet.
    },
    # Either Jibo is asking a question for the first time or is repeating it,
    # regardless, we need to start the timer when Jibo's done talking.
    {
      "trigger":"jibo_finish_tts",
      "source": "WAITING_FOR_END_PAGE_JIBO_QUESTION",
      "dest": "WAITING_FOR_END_PAGE_CHILD_RESPONSE",
      "after": ["start_waiting_for_end_page_child_response"]
    },
    {
      "trigger": "child_end_page_question_timeout",
      "source": "WAITING_FOR_END_PAGE_CHILD_RESPONSE",
      "dest": "WAITING_FOR_END_PAGE_JIBO_QUESTION",
      "after": "resend_end_page_prompt" # -> loops back to previous transition, where we wait for jibo_finish_tts then start waiting for answer
    },
    # No button has been implemented for this yet, but just add a button and this will work,
    # plus, this also happens when we get an asr with "repeat" in it, as shown in the next trigger.
    {
      "trigger": "child_request_repeat_end_page_question",
      "source": "WAITING_FOR_END_PAGE_CHILD_RESPONSE",
      "dest": "WAITING_FOR_END_PAGE_JIBO_QUESTION",
      "before": ["stop_waiting_for_child_response"],
      "after": "resend_end_page_prompt"
    },
    {
      "trigger": "asr_repeat_received",
      "source": "WAITING_FOR_END_PAGE_CHILD_RESPONSE",
      "dest": "=",
      "after": "child_request_repeat_end_page_question"
    },
    # When the child says I don't know in response to being asked a question.
    {
      "trigger": "asr_idk_received",
      "source": "WAITING_FOR_END_PAGE_CHILD_RESPONSE",
      "dest": "WAITING_FOR_END_PAGE_JIBO_QUESTION",
      "before": ["stop_waiting_for_child_response"],
      "after": "jibo_end_page_give_hint", # -> loops back to where we wait for jibo_finish_tts then start waiting for answer
      "conditions": ["hint_available_and_not_given"]
    },
    {
      "trigger": "asr_idk_received",
      "source": "WAITING_FOR_END_PAGE_CHILD_RESPONSE",
      "dest": "WAITING_FOR_END_PAGE_JIBO_RESPONSE",
      "before": ["stop_waiting_for_child_response"],
      "after": "jibo_end_page_response_to_child_idk" # -> loops back to where we wait for jibo_finish_tts
    },
    {
      "trigger":"child_end_page_got_answer",
      "source":"WAITING_FOR_END_PAGE_CHILD_RESPONSE",
      "dest":"WAITING_FOR_END_PAGE_JIBO_RESPONSE",
      "before":["stop_waiting_for_child_response"],
      "after":["jibo_end_page_response_to_child"]
    },
    {
      "trigger":"jibo_finish_tts",
      "source":"WAITING_FOR_END_PAGE_JIBO_RESPONSE",
      "dest":"WAITING_FOR_END_PAGE_JIBO_RESPONSE",
      "after":["end_page_jibo_response_complete"]
    },
    # If there are more questions, ask them.
    {
      "trigger": "end_page_jibo_response_complete",
      "source": "WAITING_FOR_END_PAGE_JIBO_RESPONSE",
      "dest": "WAITING_FOR_END_PAGE_JIBO_QUESTION",
      "after": "send_end_page_prompt", # -> loops back to waiting for jibo_finish_then waiting for an answer
      "conditions": ["more_end_page_questions_available", "in_evaluate_mode"]
    },
    # If there aren't any more questions, check if there are more pages.
    {
      "trigger":"end_page_jibo_response_complete",
      "source":"WAITING_FOR_END_PAGE_JIBO_RESPONSE",
      "dest": "WAITING_FOR_NEXT_PAGE_JIBO_INTERLUDE",
      "after": "jibo_next_page", # Jibo says something like 'Ok on to the next page!'
      "conditions": ["more_pages_available", "in_evaluate_mode"]
    },
    {
      "trigger":"end_page_jibo_response_complete",
      "source": "WAITING_FOR_END_PAGE_JIBO_RESPONSE",
      "dest": "END_STORY",
      "after": ["tablet_go_to_end_page", "jibo_end_story"]
    },
    # Analogous to above 2 triggers, but skipping asking questions entirely.
    # Either go to next page or go to end story.
    {
      "trigger": "no_questions_go_to_next_page",
      "source": "WAITING_FOR_END_PAGE_JIBO_QUESTION",
      "dest": "WAITING_FOR_NEXT_PAGE_JIBO_INTERLUDE",
      "after": "jibo_next_page",
      "conditions": ["more_pages_available", "in_evaluate_mode"]
    },
    {
      "trigger": "no_questions_go_to_next_page",
      "source": "WAITING_FOR_END_PAGE_JIBO_QUESTION",
      "dest": "END_STORY",
      "after": ["tablet_go_to_end_page", "jibo_end_story"]
    },
    {
      "trigger": "jibo_finish_tts",
      "source": "WAITING_FOR_NEXT_PAGE_JIBO_INTERLUDE",
      "dest": "WAITING_FOR_NEXT_PAGE",
      "after": "tablet_next_page" # Show the next page button, or navigate to the next page if automatic.
    },
    {
      "trigger": "jibo_finish_tts",
      "source": "END_STORY",
      "dest": "WAITING_FOR_END_STORY_CHILD_RESPONSE",
      "after": "start_waiting_for_child_response"
    },
    {
      "trigger": "jibo_finish_child_asr", 
      "source": "WAITING_FOR_END_STORY_CHILD_RESPONSE",
      "dest": "END_EVALUATE",
      "after": ["jibo_respond_to_end_story", "stop_waiting_for_child_response"],
      "conditions": ["in_evaluate_mode"]
    },
    {
      "trigger": "jibo_finish_tts",
      "source": "END_EVALUATE",
      "dest": "END_EVALUATE",
      "after": ["begin_evaluate_mode", "tablet_show_library_panel"],
      "conditions": ["in_evaluate_mode"]
    },
    # Switching modes.
    {
      "trigger": "begin_evaluate_mode",
      "source": "*",
      "dest": "BEGIN_EVALUATE"
    },
    # Catch all the triggers that might be called without regard
    # to the current state.
    {
      "trigger": "jibo_finish_tts",
      "source": "*",
      "dest": "="
    },
    {
      "trigger": "asr_received",
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
    },
    {
      "trigger": "child_end_page_got_answer",
      "source": "WAITING_FOR_END_PAGE_JIBO_RESPONSE",
      "dest": "="
    },
    {
      "trigger": "child_read_audio_complete", # Whenever child presses blue button.
      "source": "*",
      "dest": "="
    },
    {
      "trigger": "asr_idk_received",
      "source": "*",
      "dest": "="
    },
    {
      "trigger": "asr_help_received",
      "source": "*",
      "dest": "="
    }
  ]

  all_transitions = not_reading_transitions + explore_transitions + evaluate_transitions
