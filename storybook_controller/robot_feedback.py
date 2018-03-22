"""
This file contains classes used to generate feedback that the robot can
give to the student.
"""
from enum import Enum

from unity_game_msgs.msg import StorybookCommand
from storybook_controller.jibo_commands_builder import JiboStorybookBehaviors
from storybook_controller.jibo_commands_builder import JiboCommandsBuilder

def strip_punctuation(word):
  return "".join([c for c in word if c.isalnum()])

# Types of questions/prompts for Jibo to ask at the end of each page.
class EndPageQuestionType(Enum):
  WORD_TAP = 0,
  SCENE_OBJECT_TAP = 1,
  WORD_PRONOUNCE = 2

# Base class for end of page questions.
class EndPageQuestion(object):
  def __init__(self, question_type):
    self.question_type = question_type
    self.asked = False
    self.correct = False

  def ask_question(self, ros_manager):
    """
    Sends necessary commands on ros_manager to ask the question.
    """
    self.asked = True
    self.ask_question_impl(ros_manager)
  
  def ask_question_impl(self, ros_manager):
    raise NotImplementedError

  ########################################
  # TODO: add a mechanism for giving a hint, and a mechanism for responding
  # to the child's response!!
  ########################################

  def correct_answer(self):
    """
    Returns the expected correct answer to the question.
    """
    raise NotImplementedError

  def try_answer(self, query, student_model):
    """
    Checks if the child's response to the question is correct.
    Returns True if correct. Also updates as necessary.
    """
    self.correct = self.try_answer_impl(query, student_model)
    return self.correct
  
  def try_answer_impl(self, query, student_model):
    raise NotImplementedError

class EndPageQuestionWordTap(EndPageQuestion):
  def __init__(self, word):
    super(EndPageQuestionWordTap, self).__init__(EndPageQuestionType.WORD_TAP)
    self.expected_word = strip_punctuation(word.lower())

  def ask_question_impl(self, ros_manager):
    # Make all words light up (instead of having past sentences be greyed out).
    ros_manager.send_storybook_command(StorybookCommand.HIGHLIGHT_ALL_SENTENCES)
    # Will need to send jibo commands and storybook commands.
    ros_manager.send_jibo_command(JiboStorybookBehaviors.SPEAK, "Great! Let me ask you a question, because I'm a bit confused. Can you click on the word " + self.expected_word + "?")
    ros_manager.send_jibo_command(JiboStorybookBehaviors.QUESTION_ANIM)

  def correct_answer(self):
    return self.expected_word

  def try_answer_impl(self, query, student_model):
    correct = strip_punctuation(query.lower()) == strip_punctuation(self.expected_word.lower())
    if correct:
      print("Child got word correct!", query)
      student_model.update_with_correct_word_tapped(query)
    else:
      print("Child got word incorrect!", self.expected_word, query)
      student_model.update_with_incorrect_word_tapped(self.expected_word, query)
    return correct

class EndPageQuestionSceneObjectTap(EndPageQuestion):
  def __init__(self, label):
    super(EndPageQuestionSceneObjectTap, self).__init__(
      EndPageQuestionType.SCENE_OBJECT_TAP)
    self.expected_label = strip_punctuation(label.lower())

  def ask_question_impl(self, ros_manager):
    # Make all words light up (instead of having past sentences be greyed out).
    ros_manager.send_storybook_command(StorybookCommand.HIGHLIGHT_ALL_SENTENCES)
    # Will need to send jibo commands and storybook commands.
    ros_manager.send_jibo_command(JiboStorybookBehaviors.SPEAK, "Awesome! Hmm, I need some help. Can you click on " + self.expected_label + " in the picture?")
    ros_manager.send_jibo_command(JiboStorybookBehaviors.QUESTION_ANIM)

  def correct_answer(self):
    return self.expected_label

  def try_answer_impl(self, query, student_model):
    query = strip_punctuation(query.lower())
    correct = query in self.expected_label
    if correct:
      print("Child got scene object correct!", query)
      student_model.update_with_correct_scene_object_tapped(query)
    else:
      print("Child got scene object incorrect!", query, self.expected_label)
      student_model.update_with_incorrect_scene_object_tapped(self.expected_label, query)
    return correct

class EndPageQuestionWordPronounce(EndPageQuestion):
  def __init__(self, word):
    super(EndPageQuestionWordPronounce, self).__init__(
      EndPageQUestionType.WORD_PRONOUNCE)
    self.expected_word = word

  def correct_answer(self):
    return self.expected_word

  def ask_question_impl(self, ros_manager, student_model):
    pass

  def try_answer_impl(self, query, student_model):
    # TODO
    return query == self.expected_word

####################################################################

# TODO: this should be things like transitions for Jibo to say
# between pages, before starting the story, after starting, after
# a sentence, after child got something correct, etc.

# Generic types of feedback to give.
class RobotFeedbackType(Enum):
  END_PAGE_PRE_QUESTION = 0 # Say something generically encouraging before asking the question(s).
  PRAISE = 1

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
