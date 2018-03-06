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

    self.storybook_publisher = None
    self.jibo_publisher = None
    self.jibo_asr_publisher = None
    self.storybook_event_listener = None
    self.storybook_page_info_listener = None
    self.storybook_state_listener = None
    self.jibo_state_listener = None
    self.jibo_asr_listener = None

  def init_ros_node(self):
    rospy.init_node(self.node_name, anonymous=True)

  def start_listener(self, topic, message_type, callback):
    print("Starting subscriber node for:", topic)
    self.subscribers[topic] = rospy.Subscriber(topic, message_type, callback)

  def start_publisher(self, topic, message_type):
    print ("Starting publisher node for:", topic)
    self.publishers[topic] = rospy.Publisher(topic, message_type, queue_size = 10)
    rate = rospy.Rate(10)
    rate.sleep()

  def start_storybook_event_listener(self, callback):
    """
    Starts the storybook event subscriber node.
    """
    print("Starting storybook event subscriber node.")
    self.storybook_event_listener = rospy.Subscriber(STORYBOOK_EVENT_TOPIC,
                                               StorybookEvent, callback)

  def start_storybook_page_info_listener(self, callback):
    """
    Starts the storybook page info subscriber node.
    """
    print("Starting storybook page info subscriber node.")
    self.storybook_page_info_listener = rospy.Subscriber(STORYBOOK_PAGE_INFO_TOPIC,
                                               StorybookPageInfo, callback)

  def start_storybook_state_listener(self, callback):
    """
    Starts the storybook state subscriber node.
    """
    print("Starting storybook subscriber node.")
    self.storybook_state_listener = rospy.Subscriber(STORYBOOK_STATE_TOPIC,
                                               StorybookState, callback)

  def start_jibo_state_listener(self, callback):
    """
    Starts the jibo state subscriber node.
    """
    print("Starting Jibo state subscriber node.")
    self.jibo_state_listener = rospy.Subscriber(JIBO_STATE_TOPIC,
                                               JiboState, callback)
  
  def start_jibo_asr_listener(self, callback):
    """
    Starts the jibo ASR subscriber node.
    """
    print("Starting Jibo ASR subscriber node.")
    self.jibo_asr_listener = rospy.Subscriber(JIBO_ASR_RESULT_TOPIC,
                                               JiboAsrResult, callback)

  def start_storybook_publisher(self):
    """
    Starts the storybook publisher node.
    """
    print("Starting storybook publisher node.")
    self.storybook_publisher = rospy.Publisher(STORYBOOK_COMMAND_TOPIC,
                                               StorybookCommand, queue_size=10)
    # Spin at 10Hz, wait for subscribers.
    rate = rospy.Rate(10)
    rate.sleep()

  def start_jibo_publisher(self):
    """
    Starts the JiboAction publisher node.
    """
    print("Starting Jibo action publisher node.")
    self.jibo_publisher = rospy.Publisher(JIBO_ACTION_TOPIC,
                                               JiboAction, queue_size=10)
    # Spin at 10Hz, wait for subscribers.
    rate = rospy.Rate(10)
    rate.sleep()

  def start_jibo_asr_publisher(self):
    """
    Starts the publisher for sending Jibo ASR commands.
    """
    print("Starting Jibo ASR publisher node.")
    self.jibo_asr_publisher = rospy.Publisher(JIBO_ASR_COMMAND_TOPIC,
                                              JiboAsrCommand, queue_size=10)
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
    self.storybook_publisher.publish(msg)
    rospy.loginfo(msg)

  def send_jibo_command(self, command, *args):
    """
    Send a command to the robot.
    Args are any optional arguments, as a serialized JSON string.
    """
    msg = JiboCommandsBuilder.get_message_from_behavior(command, *args)
    self.jibo_publisher.publish(msg)
    rospy.loginfo(msg)

  def send_jibo_asr_command(self, command, heyjibo=True, continuous=True, rule=""):
    """
    Sends a command to Jibo to tell it to start listening and to publish back
    ASR results.

    Note that JiboAsrCommand.START_FINAL and JiboAsrCommand.STOP_FINAL are the
    only ones that are supported in jibo-rosbridge-receiver right now.
    """
    msg = JiboCommandsBuilder.get_jibo_asr_command(command, heyjibo, continuous, rule)
    self.jibo_asr_publisher.publish(msg)
    rospy.loginfo(msg)


