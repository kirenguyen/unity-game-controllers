"""
This file contains constants used in the Storybook controller.
"""

USE_JIBO = True

STORYBOOK_TO_ROSCORE_TOPIC = "/storybook_to_roscore"
ROSCORE_TO_STORYBOOK_TOPIC = "/roscore_to_storybook"

JIBO_STATE_TOPIC = "jibo_state" # Messages from jibo, 10Hz.
JIBO_ACTION_TOPIC = "/jibo" # Messages to Jibo, send on command.
JIBO_ASR_TOPIC = "/jibo_asr" # Messages from jibo for ASR results.