"""
This file contains classes used to generate feedback that the robot can
give to the student.
"""
from enum import Enum

class RobotFeedbackType(Enum):
  ASK_TO_CLICK = "ASK_TO_CLICK"
  ASK_TO_PRONOUNCE = "ASK_TO_PRONOUNCE"
  PRAISE = "PRAISE"

class RobotFeedback(object):
  def __init__(self, feedbackType, template):
    self.feedbackType = feedbackType
    self.template = template
    self.args = None

  def set_args(self, args):
    self.args = args

  def get_formatted_feedback(self):
    """
    Returns a string that is the template filled in with the arguments provided.
    
    The number of arguments should match the number of templated spots.
    There can be 0 arguments, common in the case of feedback tha requires no
    modification, such as "Good job"
    """
    return self.template.format(*self.args)
