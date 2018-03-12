"""
This file implements the ROS manager for the storybook app.

The ROS manager initiates a ROS node, manages subscriptions, listens for
messages on subscribed topics, and publishes messages to both the robot
agent and the tablet app.
"""

import rospy

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
      msg.params = args[0]
    self.publishers[STORYBOOK_COMMAND_TOPIC].publish(msg)
    rospy.loginfo(msg)

  def send_jibo_command(self, command, *args):
    """
    Send a command to the robot.
    Args are any optional arguments, as a serialized JSON string.
    """
    msg = JiboCommandsBuilder.get_message_from_behavior(command, *args)
    self.publishers[JIBO_ACTION_TOPIC].publish(msg)
    rospy.loginfo(msg)

  def send_jibo_asr_command(self, command, heyjibo=False, continuous=True, rule=""):
    """
    Sends a command to Jibo to tell it to start listening and to publish back
    ASR results.

    Note that JiboAsrCommand.START and JiboAsrCommand.STOP are the
    only ones that are supported in jibo-rosbridge-receiver right now.
    """
    msg = JiboCommandsBuilder.get_jibo_asr_command(command, heyjibo, continuous, rule)
    self.publishers[JIBO_ASR_COMMAND_TOPIC].publish(msg)
    rospy.loginfo(msg)


