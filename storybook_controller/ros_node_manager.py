"""
This file implements the ROS manager for the storybook app.

The ROS manager initiates a ROS node, manages subscriptions, listens for
messages on subscribed topics, and publishes messages to both the robot
agent and the tablet app.
"""

import rospy

from unity_game_msgs.msg import StorybookCommand
from unity_game_msgs.msg import StorybookInfo
from std_msgs.msg import Header  # Standard ROS msg header
from jibo_msgs.msg import JiboAction # Commands to Jibo
from jibo_msgs.msg import JiboState # State from Jibo
from jibo_msgs.msg import JiboAsr # ASR results from Jibo

from storybook_controller.storybook_constants import *
from storybook_controller.jibo_commands import JiboCommands
from storybook_controller.jibo_commands import JiboStorybookBehaviors

class ROSNodeManager(object):
  def __init__(self, name):
    self.node_name = name
    self.storybook_listener = None
    self.storybook_publisher = None
    self.jibo_state_listener = None
    self.jibo_asr_listener = None
    self.jibo_publisher = None

  def init_ros_node(self):
    rospy.init_node(self.node_name, anonymous=True)

  def start_storybook_listener(self, callback):
    """
    Starts the storybook subscriber node.
    """
    print("Starting storybook subscriber node.")
    self.storybook_listener = rospy.Subscriber(STORYBOOK_TO_ROSCORE_TOPIC,
                                               StorybookInfo, callback)

  def start_jibo_state_listener(self, callback):
    """
    Starts the jibo state subscriber node.
    """
    print("Starting jibo state subscriber node.")
    self.jibo_state_listener = rospy.Subscriber(JIBO_STATE_TOPIC,
                                               JiboState, callback)
  
  def start_jibo_asr_listener(self, callback):
    """
    Starts the jibo ASR subscriber node.
    """
    print("Starting jibo ASR subscriber node.")
    self.jibo_asr_listener = rospy.Subscriber(JIBO_ASR_TOPIC,
                                               JiboAsr, callback)

  def start_storybook_publisher(self):
    """
    Starts the storybook publisher node.
    """
    print("Starting storybook publisher node.")
    self.storybook_publisher = rospy.Publisher(ROSCORE_TO_STORYBOOK_TOPIC,
                                               StorybookCommand, queue_size=10)
    # Spin at 10Hz, wait for subscribers.
    rate = rospy.Rate(10)
    rate.sleep()

  def start_jibo_publisher(self):
    """
    Starts the robot publisher node.
    """
    print("Starting robot publisher node.")
    self.jibo_publisher = rospy.Publisher(JIBO_ACTION_TOPIC,
                                               JiboAction, queue_size=10)
    # Spin at 10Hz, wait for subscribers.
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
    msg = JiboCommands.get_message_from_behavior(command, *args)
    self.jibo_publisher.publish(msg)
    rospy.loginfo(msg)


