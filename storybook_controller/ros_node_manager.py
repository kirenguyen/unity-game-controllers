"""
This file implements the ROS manager for the storybook app.

The ROS manager initiates a ROS node, manages subscriptions, listens for
messages on subscribed topics, and publishes messages to both the robot
agent and the tablet app.
"""

import json
import rospy
import threading

from unity_game_msgs.msg import StorybookCommand
from unity_game_msgs.msg import StorybookEvent
from unity_game_msgs.msg import StorybookPageInfo
from unity_game_msgs.msg import StorybookState

from std_msgs.msg import Header  # Standard ROS msg header

from jibo_msgs.msg import JiboAction # Commands to Jibo
from jibo_msgs.msg import JiboState # State from Jibo
from jibo_msgs.msg import JiboAsrCommand # ASR commands to Jibo
from jibo_msgs.msg import JiboAsrResult # ASR results from Jibo

from storybook_controller.storybook_constants import *
from storybook_controller.jibo_commands_builder import JiboCommandsBuilder
from storybook_controller.jibo_commands_builder import JiboStorybookBehaviors

class ROSNodeManager(object):
  def __init__(self, name):
    self.node_name = name

    # Keyed by the topic.
    self.subscribers = {}
    self.publishers = {}

    self.jibo_resend_tts_timer = None
    self.JIBO_RESEND_TTS_TIMEOUT_SECONDS = 10
    self.last_jibo_text = None


  def init_ros_node(self):
    rospy.init_node(self.node_name, anonymous=False)

  def start_listener(self, topic, message_type, callback):
    print("Starting subscriber node for:", topic)
    self.subscribers[topic] = rospy.Subscriber(topic, message_type, callback)

  def start_publisher(self, topic, message_type):
    print ("Starting publisher node for:", topic)
    self.publishers[topic] = rospy.Publisher(topic, message_type, queue_size = 10)
    rate = rospy.Rate(10)
    rate.sleep()

  def send_storybook_command(self, command, *args):
    """
    Send a command to the storybook.
    Args are any optional arguments, as a serialized JSON string.
    """
    msg = StorybookCommand()
    msg.header = Header()
    msg.header.stamp = rospy.Time.now()
    msg.command = command
    if len(args) > 0:
      if type(args[0]) == str:
        msg.params = args[0]
      else:
        msg.params = json.dumps(args[0])
    else:
      msg.params = ""
    self.publishers[STORYBOOK_COMMAND_TOPIC].publish(msg)
    # rospy.loginfo(msg)

  def send_jibo_command(self, command, *args):
    """
    Send a command to the robot.
    Args are any optional arguments, as a serialized JSON string.
    """
    msg = JiboCommandsBuilder.get_message_from_behavior(command, *args)
    self.publishers[JIBO_ACTION_TOPIC].publish(msg)
    if command == JiboStorybookBehaviors.SPEAK:
      # Save the last text.
      self.last_jibo_text = args[0]
      # Start the timer, might need to retry.
      self.start_jibo_resend_tts_timer()
    # rospy.loginfo(msg)

  def send_jibo_asr_command(self, command, rule="", heyjibo=False, incremental=True, continuous=True):
    """
    Sends a command to Jibo to tell it to start listening and to publish back
    ASR results.

    Note that JiboAsrCommand.START and JiboAsrCommand.STOP are the
    only ones that are supported in jibo-rosbridge-receiver right now.
    """
    print("Sending jibo asr command")
    msg = JiboCommandsBuilder.get_jibo_asr_command(command, heyjibo, continuous, incremental, rule)
    self.publishers[JIBO_ASR_COMMAND_TOPIC].publish(msg)

    # rospy.loginfo(msg)

  def start_jibo_resend_tts_timer(self):
    # Stop it first.
    self.stop_jibo_resend_tts_timer()
    self.jibo_resend_tts_timer = threading.Timer(
      self.JIBO_RESEND_TTS_TIMEOUT_SECONDS, self.jibo_resend_tts_timer_expire_handler)
    self.jibo_resend_tts_timer.start()

  def stop_jibo_resend_tts_timer(self):
    if self.jibo_resend_tts_timer is not None and self.jibo_resend_tts_timer.is_alive():
      self.jibo_resend_tts_timer.cancel()

  def jibo_resend_tts_timer_expire_handler(self):
    print("jibo resend tts timer expired, resending!")
    self.send_jibo_command(JiboStorybookBehaviors.SPEAK, self.last_jibo_text)
    self.start_jibo_resend_tts_timer()

  def report_jibo_tts_received(self, received_text):
    """
    Controller tells ros manager what latest tts message was, so ros manager
    can confirm the tts message was sent.
    """
    if received_text == self.last_jibo_text:
      self.stop_jibo_resend_tts_timer()