from enum import Enum
import rospy

from jibo_msgs.msg import JiboAction
from std_msgs.msg import Header

class JiboStorybookBehaviors(Enum):
	EXPLAIN_WORD = 0

class JiboCommands(object):
	
  @staticmethod
  def get_message_from_behavior(command, *args):
    print("getting message from behavior")

    msg = JiboAction()
    msg.header = Header()
    msg.header.stamp = rospy.Time.now()

    if (command == JiboStorybookBehaviors.EXPLAIN_WORD):
      msg.do_motion = False
      msg.do_tts = True
      msg.do_lookat = False
      msg.do_sound_playback = False
      msg.tts_text = args[0]

    return msg