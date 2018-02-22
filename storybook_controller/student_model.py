"""
This file implements the student model for the storybook controller.

The model attempts to update the belief that a student can correctly pronounce
a word, and the belief that a student knows the meaning of that word. These
beliefs are correlated but updated under different criteria.
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

class StudentModel(object):
  def __init__(self):
    self.vocabulary = None
    self.pronounce_dist = None
    self.meaning_dist = None

    self.feedback_templates = [
      RobotFeedback(RobotFeedbackType.ASK_TO_CLICK, "Can you click on {} {} in the image?"),
      RobotFeedback(RobotFeedbackType.ASK_TO_PRONOUNCE, "Can you pronounce this word?")
    ]

  def set_vocabulary(self, vocabulary):
    """
    Initialize this student model with a new vocabulary. Clears existing
    distributions.
    """
    raise NotImplementedError

  def init_distributions(self, correct_pronounce, correct_meaning):
    """
    Initialize distributions given a list of words pronouncec correctly and a
    list of words the student knew the meaning of.
    """
    raise NotImplementedError

  def update_with_speechace_result(self, speechace_result):
    """
    Updates the model given the result of a recent speechace analysis.
    """
    raise NotImplementedError

  def get_feedback(self):
    """
    Returns feedback for the robot to give.
    """
    self.feedback_template[0].set_args("the", "dog")
    return self.feedback_template[0]


  def plot_distribution(self):
    """
    Plot the current distribution.
    """
    raise NotImplementedError

