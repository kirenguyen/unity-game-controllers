"""
This file contains classes used to generate feedback that the robot can
give to the student.
"""
from enum import Enum
import time

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
    self.answered = False
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
    Checks if the child's response (query) to the question is correct.
    Returns True if correct. Also updates as necessary.
    """
    self.answered = True
    self.correct = self.try_answer_impl(query, student_model)
    return self.correct
  
  def try_answer_impl(self, query, student_model):
    """
    Should be implemented in subclass.
    """
    raise NotImplementedError

  def respond_to_child(self, ros_manager):
    """
    Respond to the child depending on if they were correct or not.
    """
    time.sleep(1.5)
    if not self.answered:
      raise Exception("Can't respond to child if no response")
    if self.correct:
      print("respond_to_child_correct_impl")
      self.respond_to_child_correct_impl(ros_manager)
    else:
      print("respond_to_child_correct_impl")
      self.respond_to_child_incorrect_impl(ros_manager)

  def respond_to_child_correct_impl(self, ros_manager):
    """
    Should be implemented in subclass.
    """
    raise NotImplementedError

  def respond_to_child_incorrect_impl(self, ros_manager):
    """
    Should be implemented in subclass.
    """
    raise NotImplementedError

"""
Tap on a word.
"""
class EndPageQuestionWordTap(EndPageQuestion):
  def __init__(self, word, indexes):
    super(EndPageQuestionWordTap, self).__init__(EndPageQuestionType.WORD_TAP)
    self.expected_word = strip_punctuation(word.lower())
    self.expected_indexes = indexes

  def ask_question_impl(self, ros_manager):
    # Make all words light up (instead of having past sentences be greyed out).
    ros_manager.send_storybook_command(StorybookCommand.HIGHLIGHT_ALL_SENTENCES)
    # Will need to send jibo commands and storybook commands.
    ros_manager.send_jibo_command(JiboStorybookBehaviors.SPEAK, "Great! Let me ask you a question. Can you tap the word " + self.expected_word + "?")
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

  def respond_to_child_correct_impl(self, ros_manager):
    ros_manager.send_jibo_command(JiboStorybookBehaviors.HAPPY_DANCE)
    ros_manager.send_jibo_command(JiboStorybookBehaviors.SPEAK, "That's right! This is the word " + self.expected_word)
    time.sleep(2)
    params = {"indexes": self.expected_indexes}
    ros_manager.send_storybook_command(StorybookCommand.HIGHLIGHT_WORD, params)

  def respond_to_child_incorrect_impl(self, ros_manager):
    ros_manager.send_jibo_command(JiboStorybookBehaviors.SPEAK, "Nope, good try, but this is the word " + self.expected_word)
    time.sleep(2)
    params = {"indexes": self.expected_indexes}
    ros_manager.send_storybook_command(StorybookCommand.HIGHLIGHT_WORD, params)

"""
Tap on an object in the image.
"""
class EndPageQuestionSceneObjectTap(EndPageQuestion):
  def __init__(self, label, ids):
    super(EndPageQuestionSceneObjectTap, self).__init__(
      EndPageQuestionType.SCENE_OBJECT_TAP)
    self.expected_label = strip_punctuation(label.lower())
    self.expected_ids = ids

  def ask_question_impl(self, ros_manager):
    # Make all words light up (instead of having past sentences be greyed out).
    ros_manager.send_storybook_command(StorybookCommand.HIGHLIGHT_ALL_SENTENCES)
    # Will need to send jibo commands and storybook commands.
    ros_manager.send_jibo_command(JiboStorybookBehaviors.SPEAK, "Awesome! I have a question for you. Can you tap on " + self.expected_label + " in the picture?")
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
      student_model.update_with_incorrect_scene_object_tapped(
        self.expected_label, query)
    return correct

  def respond_to_child_correct_impl(self, ros_manager):
    ros_manager.send_jibo_command(JiboStorybookBehaviors.HAPPY_DANCE)
    ros_manager.send_jibo_command(JiboStorybookBehaviors.SPEAK,
      "That's right! This is " + self.expected_label)
    time.sleep(2)
    params = {"ids": self.expected_ids}
    ros_manager.send_storybook_command(StorybookCommand.HIGHLIGHT_SCENE_OBJECT, params)

  def respond_to_child_incorrect_impl(self, ros_manager):
    ros_manager.send_jibo_command(JiboStorybookBehaviors.SPEAK,
      "Not quite. Actually, this is " + self.expected_label)
    time.sleep(2)
    params = {"ids": self.expected_ids}
    ros_manager.send_storybook_command(StorybookCommand.HIGHLIGHT_SCENE_OBJECT, params)

"""
Pronounce a word.
"""
class EndPageQuestionWordPronounce(EndPageQuestion):
  def __init__(self, word, index):
    super(EndPageQuestionWordPronounce, self).__init__(
      EndPageQUestionType.WORD_PRONOUNCE)
    self.expected_word = word
    self.expected_index = index

  def correct_answer(self):
    return self.expected_word

  def ask_question_impl(self, ros_manager):
    ros_manager.send_jibo_command(JiboStorybookBehaviors.SPEAK,
      "Can you pronounce this word for me?")
    ros_manager.send_storybook_command(StorybookCommand.HIGHLIGHT_WORD, params)

  def try_answer_impl(self, query, student_model):
    return query in self.expected_word

  def respond_to_child_correct_impl(self, ros_manager):
    params = {"indexes": [self.expected_index]}
    ros_manager.send_jibo_command(JiboStorybookBehaviors.SPEAK,
      "Ok, cool, yes the pronunciation of this word is " + self.expected_word)
    ros_manager.send_storybook_command(Storybook.HIGHLIGHT_WORD, params)

  def respond_to_child_incorrect_impl(self, ros_manager):
    # We can't 100% trust ASR responses, so don't outright say if
    # the child was correct or not, just give some acknowledgment
    # and provide the correct answer.
    self.respond_to_child_correct_impl(ros_manager)

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
