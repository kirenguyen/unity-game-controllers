"""
This file implements the ROS manager for the storybook app.

The ROS manager initiates a ROS node, manages subscriptions, listens for
messages on subscribed topics, and publishes messages to both the robot
agent and the tablet app.
"""

import rospy

from unity_game_msgs.msg import StorybookCommand
from unity_game_msgs.msg import StorybookGameInfo
# from unity_game_msgs.msg import StorybookRobotCommand
from jibo_msgs.msg import JiboAction
from std_msgs.msg import Header  # standard ROS msg header

from storybook_controller.storybook_constants import *

class ROSNodeManager(object):
  def __init__(self, name):
    self.node_name = name
    self.storybook_listener = None
    self.storybook_publisher = None
    self.robot_listener = None
    self.robot_publsher = None

  def init_ros_node(self):
    rospy.init_node(self.node_name, anonymous=True)

  def start_storybook_listener(self, callback):
    """
    Starts the storybook subscriber node.
    """
    print("Starting storybook subscriber node.")
    self.storybook_listener = rospy.Subscriber(STORYBOOK_TO_ROSCORE_TOPIC,
                                               StorybookGameInfo, callback)

  # def start_robot_listener(self, callback):
  #   """
  #   Starts the robot subscriber node.
  #   """
  #   print("Starting storybook subscriber node.")
  #   self.storybook_listener = rospy.Subscriber(ROBOT_TO_ROSCORE_TOPIC,
  #                             No module named _thread
  #               JiboStorybookLog, callback)
  
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

  def start_robot_publisher(self):
    """
    Starts the robot publisher node.
    """
    print("Starting robot publisher node.")
    self.robot_publisher = rospy.Publisher(ROSCORE_TO_STORYBOOK_TOPIC,
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

  def send_robot_command(self, command, *args):
    """
    Send a command to the robot.
    Args are any optional arguments, as a serialized JSON string.
    """

    msg = JiboBehaviors.get_msg_from_behavior(command, args)
    self.storybook_publisher.publish(msg)
    rospy.loginfo(msg)


