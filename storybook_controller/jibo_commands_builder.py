from enum import Enum
import rospy

from jibo_msgs.msg import JiboAction
from jibo_msgs.msg import JiboAsrCommand
from std_msgs.msg import Header

# Different types of behaviors we can tell Jibo to perform.
class JiboStorybookBehaviors(Enum):
  SPEAK = 0
  HAPPY_ANIM = 1
  HAPPY_DANCE = 2 # HAPPY_DANCE involves more movement than HAPPY_ANIM.
  SAD_ANIM = 3
  QUESTION_ANIM = 4

# Helper class that constructs JiboAction and JiboAsrCommand messages.
class JiboCommandsBuilder(object):
  
  @staticmethod
  def get_message_from_behavior(command, *args):

    msg = JiboAction()
    msg.header = Header()
    msg.header.stamp = rospy.Time.now()

    if (command == JiboStorybookBehaviors.SPEAK):
      msg.do_motion = False
      msg.do_tts = True
      msg.do_lookat = False
      msg.do_sound_playback = False
      msg.tts_text = args[0]
      if len(args) > 1:
        msg.tts_duration_stretch = args[1]
      if len(args) > 2:
        msg.tts_pitch = args[2]

    elif (command == JiboStorybookBehaviors.HAPPY_ANIM):
      msg.do_motion = True
      msg.do_tts = False
      msg.do_lookat = False
      msg.do_sound_playback = False
      msg.motion = JiboAction.HAPPY_ANIM_AND_SOUND

    elif (command == JiboStorybookBehaviors.HAPPY_DANCE):
      msg.do_motion = True
      msg.do_tts = False
      msg.do_lookat = False
      msg.do_sound_playback = False
      msg.motion = JiboAction.CELEBRATE_ANIM

    elif (command == JiboStorybookBehaviors.SAD_ANIM):
      msg.do_motion = True
      msg.do_tts = False
      msg.do_lookat = False
      msg.do_sound_playback = False
      msg.motion = JiboAction.SAD_ANIM_AND_SHAKE_HEAD
    
    elif (command == JiboStorybookBehaviors.QUESTION_ANIM):
      msg.do_motion = True
      msg.do_tts = False
      msg.do_lookat = False
      msg.do_sound_playback = False
      msg.motion = JiboAction.QUESTION_ANIM_AND_EMOJI

    return msg

  @staticmethod
  def get_jibo_asr_command(command, heyjibo, continuous, rule):
    msg = JiboAsrCommand()
    msg.header = Header()
    msg.header.stamp = rospy.Time.now()

    msg.command = command
    msg.heyjibo = heyjibo
    msg.continuous = continuous
    msg.rule = rule

    return msg
