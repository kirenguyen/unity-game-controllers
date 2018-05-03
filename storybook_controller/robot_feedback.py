"""
This file contains classes used to generate feedback that the robot can
give to the student.
"""
from enum import Enum
import time

from unity_game_msgs.msg import StorybookCommand
from storybook_controller.jibo_commands_builder import JiboStorybookBehaviors
from storybook_controller.jibo_commands_builder import JiboCommandsBuilder
from storybook_controller.jibo_statements import JiboStatements
from storybook_controller.jibo_statements import JiboStatementType

def strip_punctuation(word):
  return "".join([c for c in word if c.isalnum()])

# Types of questions/prompts for Jibo to ask at the end of each page.
class EndPageQuestionType(Enum):
  WORD_TAP = 0,
  SCENE_OBJECT_TAP = 1,
  SPEECH_REQUESTED = 2,
  WORD_PRONUNCIATION = 3,
  REREAD_SENTENCE = 4,

# Base class for end of page questions.
class EndPageQuestion(object):
  def __init__(self, question_type):
    self.question_type = question_type
    self.hint_exists = False
    self.asked = False
    self.hint_given = False
    self.answered = False
    self.correct = False
    self.should_ask_again_on_incorrect_answer = False # Only true for word pronunciation questions right now

  def __eq__(self, other):
    if not isinstance(other, self.__class__):
      return False
    if self.question_type != other.question_type:
      return False
    return self.is_equal(other)

  def __ne__(self, other):
    return not self.__eq__(other)

  def is_equal(self, other):
    raise NotImplementedError

  def ask_question(self, ros_manager):
    """
    Sends necessary commands on ros_manager to ask the question.
    """
    pre_question_prompt = None
    if self.asked:
      # This is the second type (at least) that we're asking the question.
      pre_question_prompt = JiboStatements.get_statement(
        JiboStatementType.PRE_END_PAGE_QUESTION_REPROMPT)
    else:
      pre_question_prompt = JiboStatements.get_statement(
        JiboStatementType.PRE_END_PAGE_QUESTION)

    # Make all words light up (instead of having past sentences be greyed out).
    ros_manager.send_storybook_command(StorybookCommand.HIGHLIGHT_ALL_SENTENCES)
    self.ask_question_impl(ros_manager, pre_question_prompt)
    self.asked = True
  
  def ask_question_impl(self, ros_manager, pre_question_prompt):
    raise NotImplementedError

  def clone_for_ask_again(self):
    if self.should_ask_again_on_incorrect_answer:
      return self.clone_for_ask_again_impl()
    else:
      raise Exception("Should not be cloning a question that isn't meant to be repeated")

  def clone_for_ask_again_impl(self, ros_manager):
    raise NotImplementedError

  def give_hint(self, ros_manager):
    self.hint_given = True
    if not self.hint_exists:
      raise Exception("Cannot give hint, no hint exists")
    else:
      pre_hint_prompt = JiboStatements.get_statement(
        JiboStatementType.PRE_END_PAGE_QUESTION_HINT)
      self.give_hint_impl(ros_manager, pre_hint_prompt)

  def give_hint_impl(self, ros_manager, pre_hint_prompt):
    raise NotImplementedError

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

  def respond_to_child(self, ros_manager, idk=False):
    """
    Respond to the child depending on if they were correct or not.

    Parameter idk is True if the child expressed that they didn't know the answer.
    Otherwise, the child attempted to answer, and now Jibo should respond.
    """
    time.sleep(1)
    if not self.answered and not idk:
      raise Exception("Can't respond to child if no response.")
    pre_response_prompt = None
    # TODO: use Jibo Statements to get more variation.
    if idk:
      pre_response_prompt = "That's ok, I can help you out."
    else:
      if self.correct:
        pre_response_prompt = "Yup, nice job!"
      else:
        pre_response_prompt = "Good try, but that's not quite right. <break size='.2'/>"
    self.respond_to_child_impl(ros_manager, pre_response_prompt)

  def respond_to_child_impl(self, ros_manager, pre_response_prompt):
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

  def is_equal(self, other):
    return self.expected_word == other.expected_word

  def ask_question_impl(self, ros_manager, pre_question_prompt):
    # Will need to send jibo commands and storybook commands.
    jibo_text = pre_question_prompt + "Can you tap the word " + self.expected_word + "?"
    ros_manager.send_jibo_command(JiboStorybookBehaviors.SPEAK, jibo_text)
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

  def respond_to_child_impl(self, ros_manager, pre_response_prompt):
    jibo_text = pre_response_prompt + ". This is the word " + self.expected_word
    ros_manager.send_jibo_command(JiboStorybookBehaviors.SPEAK, jibo_text)
    time.sleep(2)
    params = {"indexes": self.expected_indexes}
    ros_manager.send_storybook_command(StorybookCommand.HIGHLIGHT_WORD, params)

"""
Tap on an object in the image.
"""
class EndPageQuestionSceneObjectTap(EndPageQuestion):
  def __init__(self, label, ids, question_text=None, response_text=None):
    super(EndPageQuestionSceneObjectTap, self).__init__(
      EndPageQuestionType.SCENE_OBJECT_TAP)
    self.expected_label = " ".join(map(lambda w: strip_punctuation(w.lower()), label.split(" ")))
    self.expected_ids = ids
    self.question_text = question_text if question_text is not None \
      else "Can you tap on " + self.expected_label + " in the picture?"
    self.response_text = response_text if response_text is not None \
      else ". This is " + self.expected_label

  def is_equal(self, other):
    return self.expected_label == other.expected_label

  def ask_question_impl(self, ros_manager, pre_question_prompt):
    # Will need to send jibo commands and storybook commands.
    jibo_text = pre_question_prompt + self.question_text
    ros_manager.send_jibo_command(JiboStorybookBehaviors.SPEAK, jibo_text)

  def correct_answer(self):
    return self.expected_label

  def try_answer_impl(self, query, student_model):
    query = " ".join(map(lambda w: strip_punctuation(w.lower()), query.split(" ")))
    correct = query in self.expected_label
    if correct:
      print("Child got scene object correct!", query)
      student_model.update_with_correct_scene_object_tapped(query)
    else:
      print("Child got scene object incorrect!", query, self.expected_label)
      student_model.update_with_incorrect_scene_object_tapped(
        self.expected_label, query)
    return correct

  def respond_to_child_impl(self, ros_manager, pre_response_prompt):
    jibo_text = pre_response_prompt + self.response_text
    ros_manager.send_jibo_command(JiboStorybookBehaviors.SPEAK, jibo_text)
    time.sleep(2)
    params = {"ids": self.expected_ids}
    print("sending scene object")
    ros_manager.send_storybook_command(StorybookCommand.HIGHLIGHT_SCENE_OBJECT, params)

"""
Pronounce a word.
"""
class EndPageQuestionOpenEndedVerbalResponse(EndPageQuestion):
  def __init__(self, question, response_text, hint_text="", extra_response_function=None):
    """
    Parameter extra_response_function is added behavior on top of simply
    saying the response. The function should take as argument the ros manager.
    """
    super(EndPageQuestionOpenEndedVerbalResponse, self).__init__(
      EndPageQuestionType.SPEECH_REQUESTED)

    self.open_ended_question = question
    self.response_text = response_text
    self.hint_text = hint_text
    if self.hint_text is not None and self.hint_text != "":
      self.hint_exists = True
    self.extra_response_function = extra_response_function

  def is_equal(self, other):
    # Don't really need this function, we only compare to auto generated
    # not open ended questions.
    return self.open_ended_question == other.open_ended_question

  def correct_answer(self):
    return self.response_text

  def ask_question_impl(self, ros_manager, pre_question_prompt):
    jibo_text = pre_question_prompt + self.open_ended_question
    ros_manager.send_jibo_command(JiboStorybookBehaviors.SPEAK, jibo_text)

  def give_hint_impl(self, ros_manager, pre_hint_prompt):
    jibo_text = pre_hint_prompt + self.hint_text
    ros_manager.send_jibo_command(JiboStorybookBehaviors.SPEAK, jibo_text)

  def try_answer_impl(self, query, student_model):
    return query in self.response_text

  def respond_to_child_impl(self, ros_manager, pre_response_prompt):
    # We won't know for sure if the child was right, so just start with
    # something generic then state the correct response to make sure the
    # point gets across.
    generic_prompt = "Good thought."
    jibo_text = generic_prompt + self.response_text
    jibo_text = self.response_text
    ros_manager.send_jibo_command(JiboStorybookBehaviors.SPEAK, jibo_text)
    if self.extra_response_function is not None:
      self.extra_response_function(ros_manager)


class EndPageQuestionWordPronounce(EndPageQuestion):
  def __init__(self, word, indexes, already_asked=False):
    super(EndPageQuestionWordPronounce, self).__init__(
      EndPageQuestionType.WORD_PRONUNCIATION)
    self.expected_word = word
    self.expected_indexes = indexes
    self.already_asked = already_asked
    self.should_ask_again_on_incorrect_answer = True

  def clone_for_ask_again_impl(self):
    """
    Returns a copy version of the question, with already_asked set to True,
    but self.asked and self.answered and self.correct reset to blank.
    """
    return EndPageQuestionWordPronounce(self.expected_word, self.expected_indexes, True)

  def is_equal(self, other):
    return self.expected_word == other.expected_word

  def correct_answer(self):
    return self.expected_word

  def ask_question_impl(self, ros_manager, pre_question_prompt):
    jibo_text = "Can you pronounce the blue word for me?"
    if not self.already_asked:
      jibo_text = pre_question_prompt + jibo_text
    ros_manager.send_jibo_command(JiboStorybookBehaviors.SPEAK, jibo_text)
    params = {"indexes": self.expected_indexes, "stay_on": True}
    ros_manager.send_storybook_command(StorybookCommand.HIGHLIGHT_WORD, params)

  def try_answer_impl(self, query, student_model):
    return query in self.expected_word or self.expected_word in query

  def respond_to_child_impl(self, ros_manager, pre_response_prompt):
    jibo_text = "the pronunciation of this word is <break size='.4'/> <duration stretch='1.3'>" + \
      self.expected_word + " </duration>."
    # If it's the second time getting it wrong, don't tell them that it's wrong again because
    # that's discouraging.
    if self.already_asked and not self.correct:
      jibo_text = "Ok, I'll try again too. <break size='.2'/>" + jibo_text
    # Otherwise, just add the normal pre_response_prompt.
    else:
      jibo_text = pre_response_prompt + jibo_text

    if not self.correct:
      if self.already_asked:
        jibo_text += "<break size='.5/> Let's continue."
      else:
        jibo_text += "<break size='.5'/> Maybe we can try again."

    ros_manager.send_jibo_command(JiboStorybookBehaviors.SPEAK, jibo_text)
    time.sleep(6)
    params = {"indexes": self.expected_indexes, "unhighlight": True}
    ros_manager.send_storybook_command(StorybookCommand.HIGHLIGHT_WORD, params)

class EndPageQuestionRereadSentence(EndPageQuestion):
  def __init__(self, page_num, sentence_index, sentence_text_array, word_index_offset):
    super(EndPageQuestionRereadSentence, self).__init__(
      EndPageQuestionType.REREAD_SENTENCE)
    self.page_num = page_num
    self.sentence_index = sentence_index
    self.sentence_text_array = sentence_text_array
    # Figure out exactly which words are in this sentence,
    # so we know which ones to highlight.
    self.word_indexes = []
    for i in range(word_index_offset, word_index_offset + len(sentence_text_array)):
       self.word_indexes.append(i)

  def is_equal(self, other):
    return self.page_num == other.page_num and self.sentence_index == other.sentence_index

  def correct_answer(self):
    return self.sentence_text

  def ask_question_impl(self, ros_manager, pre_question_prompt):
    jibo_text = pre_question_prompt + "The blue sentence says <break size='.5'/> " + \
                "<duration stretch='1.3'>" + " ".join(self.sentence_text_array) + "</duration>" + \
                "<break size='.5'/> Can you read it again? And press the button when you're done. Go ahead."
    ros_manager.send_jibo_command(JiboStorybookBehaviors.SPEAK, jibo_text)
    params = {"indexes": self.word_indexes, "stay_on": True}
    print("Indexes for sentence", self.sentence_index, "are:", self.word_indexes)
    ros_manager.send_storybook_command(StorybookCommand.HIGHLIGHT_WORD, params)

  def try_answer_impl(self, query, student_model):
    # Note that this will almost always be false, unless they say it 100% correct.
    return query in " ".join(self.sentence_text_array)

  def respond_to_child_impl(self, ros_manager, pre_response_prompt):
    jibo_text = "All right, good job."
    ros_manager.send_jibo_command(JiboStorybookBehaviors.SPEAK, jibo_text)
    time.sleep(3)
    params = {"indexes": self.word_indexes, "unhighlight": True}
    ros_manager.send_storybook_command(StorybookCommand.HIGHLIGHT_WORD, params)

