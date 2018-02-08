"""
This file implements the finite state machine (FSM) controller for the
interactive storybook app.

The FSM is responsible for state management, publication and subscription to
ROS topics, updating the student model, and sending commands to the robot agent
(Jibo or Tega).
"""

import transitions

from unity_game_msgs.msg import StorybookCommand
from unity_game_msgs.msg import StorybookLog

from storybook_constants import *

class StorybookFSM(object):
  def __init__(self, ros_node_manager, student_model):
    self.ros = ros_node_manager
    self.student_model = student_model

    self.ALLOWED_LOG_MESSAGES = []

    self.states = []
    self.initial_state = None
    self.transitions = [
      {
        "trigger": "",
        "source": "",
        "dest": "",
        "after": ""
      },
      {
        "trigger": "",
        "source": "",
        "dest": "",
        "after": ""
      }
    ]

    self.state_machine = transitions.machine(
      self, states=self.states, transitions=self.transitions,
      initial=self.initial_state)


  def ros_message_handler(self, data):
    """
    Define the callback function to pass to ROS node manager. This callback is
    called when the ROS node manager receives new data.
    """
    print data


    # CONCURRENCY WILL BE A HUGE PROBLEM FOR THIS
    #
    # Need to define all of the callback functions for conditions and after
    # functions.
    #
    # Triggers need to be called by name by other functions.
    #
    # Conditions with arguments are passed those arguments by the call to the
    # trigger.
