from enum import Enum
import rospy

from jibo_msgs.msg import JiboAction
from jibo_msgs.msg import JiboAsrCommand
from std_msgs.msg import Header

# Different types of behaviors we can tell Jibo to perform.
class JiboStorybookBehaviors(Enum):
	EXPLAIN_WORD = 0

# Helper class that constructs JiboAction and JiboAsrCommand messages.
class JiboCommandsBuilder(object):
	
  @staticmethod
  def get_message_from_behavior(command, *args):

    msg = JiboAction()
    msg.header = Header()
    msg.header.stamp = rospy.Time.now()

    if (command == JiboStorybookBehaviors.EXPLAIN_WORD):
      msg.do_motion = False
      msg.do_tts = True
      msg.do_lookat = False
      msg.do_sound_playback = False
      msg.tts_text = args[0]
      if len(args) > 1:
        msg.tts_duration_stretch = args[1]
      if len(args) > 2:
        msg.tts_pitch = args[2]

    return msg

  @staticmethod
  def get_jibo_asr_command(command, heyjibo, continuous, rule):
    print("creating jibo asr command")

    msg = JiboAsrCommand()
    msg.header = Header()
    msg.header.stamp = rospy.Time.now()

    msg.command = command
    msg.heyjibo = heyjibo
    msg.continuous = continuous
    msg.rule = rule

    return msg
