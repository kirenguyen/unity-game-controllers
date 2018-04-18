"""
The class JiboStatements returns generic transitional statements
for Jibo to say, to provide some variety in Jibo's speech.

Possible opportunities for this are between pages,
before starting the story, after starting the story, after a sentence,
before asking a question, after child got something correct, etc.
"""

from enum import Enum
import random

class JiboStatementType(Enum):
  PRE_END_PAGE_QUESTION = 0,
  PRE_END_PAGE_QUESTION_REPROMPT = 1
  PRE_HELP_WITH_SENTENCE = 2

class JiboStatements(object):
  # The more generic fallback statement.
  default_statement = "Hmm, cool!"

  pre_end_page_question_prompts = [
    "Awesome! I have a question again.",
    "Let me ask you a question.",
    "Nice, how about I ask you something?",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
  ]

  pre_end_page_question_reprompts = [
    "Here's the question again, and if you don't know just say <break size='.4'/> I don't know <break size='.4'/> and I'll help out: ",
    "Ok, let me repeat the question, and if you're not sure just say <break size='.4'/> I don't know <break size='.4'/> and I'll tell you what I think: ",
  ]

  pre_help_with_sentence_prompts = [
    "No worries, <break size='.3'/> let me help!",
    "Ok, sure, <break size='.2'/> I think I can help.",
    "I'm glad you asked. I'm happy to help!"
  ]

  type_to_statements_map = {
    JiboStatementType.PRE_END_PAGE_QUESTION: pre_end_page_question_prompts,
    JiboStatementType.PRE_END_PAGE_QUESTION_REPROMPT: pre_end_page_question_reprompts,
    JiboStatementType.PRE_HELP_WITH_SENTENCE: pre_help_with_sentence_prompts,
  }

  @staticmethod
  def get_statement(statement_type):
    """
    Returns a string that is a statement of the desired type.
    """
    if statement_type not in JiboStatements.type_to_statements_map:
      print("No such statement type:", statement_type)
      return JiboStatements.default_statement
    possible_prompts = JiboStatements.type_to_statements_map[statement_type]
    if len(possible_prompts) == 0:
      print("No prompts available for statement type:", statement_type)
      return JiboStatements.default_statement
    # Note that randint is inclusive on both ends.
    return possible_prompts[random.randint(0, len(possible_prompts) - 1)]
