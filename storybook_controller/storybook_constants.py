"""
This file contains constants used in the Storybook controller.
"""

USE_JIBO = True

STORYBOOK_EVENT_TOPIC = "/storybook_event";
STORYBOOK_PAGE_INFO_TOPIC = "/storybook_page_info";
STORYBOOK_STATE_TOPIC = "/storybook_state";
STORYBOOK_COMMAND_TOPIC = "/storybook_command";

JIBO_STATE_TOPIC = "/jibo_state" # Messages from jibo, 10Hz.
JIBO_ACTION_TOPIC = "/jibo" # Messages to Jibo, send on command.
JIBO_ASR_COMMAND_TOPIC = "/jibo_asr_command" # Messages to Jibo to request ASR.
JIBO_ASR_RESULT_TOPIC = "/jibo_asr_result" # Messages from Jibo with ASR results.

# This is what Jibo returns if no speech was heard during the listening period,
# which defaultsto 16 seconds.
ASR_NOSPEECH = "NOSPEECH"
