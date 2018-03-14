"""
This file implements the student model for the storybook controller.

The model attempts to update the belief that a student can correctly pronounce
a word, and the belief that a student knows the meaning of that word. These
beliefs are correlated but updated under different criteria.
"""

from storybook_controller.robot_feedback import RobotFeedbackType, RobotFeedback

class StudentModel(object):
  def __init__(self):
    # Dictionary representing the vocabulary.
    # Choose dictionary instead of list for faster existence checks.
    self.vocabulary = {}
    
    self.word_scores = {}
    self.phoneme_scores = {}

    # Keep track of trend of overall scores per page.
    self.overall_quality_scores = []

    # Current sentences on the page in evaluate mode.
    self.sentences = []

    self.feedback_templates = [
      RobotFeedback(RobotFeedbackType.ASK_TO_CLICK, "Can you click on {} {} in the image?"),
      RobotFeedback(RobotFeedbackType.ASK_TO_PRONOUNCE, "Can you pronounce this word?")
    ]

  def update_with_speechace_result(self, speechace_result):
    """
    Updates the model given the result of a recent speechace analysis.
    """
    print("Updating with speech ace results!")
    text_score = speechace_result["text_score"]
    # Record overall quality score.
    self.overall_quality_scores.append(text_score["quality_score"])
    # Record scores for each word.
    for word_score in text_score["word_score_list"]:
      # Convert all words to lowercase.
      word = word_score["word"].lower()
      if word not in self.word_scores:
        self.word_scores[word] = []
      self.word_scores[word].append(word_score["quality_score"])
      # Record scores for each phoneme of each word.
      for phoneme_score in word_score["phone_score_list"]:
        phoneme = phoneme_score["phone"]
        if phoneme not in self.phoneme_scores:
          self.phoneme_scores[phoneme] = []
        self.phoneme_scores[phoneme].append(phoneme_score["quality_score"])


    self.plot_distribution()

    # TODO something more sophisticated given this score information?

  def update_with_word_tapped(self, word):
    """
    Update model given that the child tapped on a certain word while exploring.
    """
    pass

  def is_child_turn(self, sentence_index):
    return True

  def update_current_sentences(self, sentences):
    self.sentences = []
    for sentence in sentences:
      self.sentences.append(sentence.split())

  def get_lowest_pronunciation_score_word(words):
    """
    Given an array of words (likely representing a sentence), return the word
    that the child performs the worst on.
    TODO: add in meaning/phonemes as well as simple average word scores.
    """
    word_to_return = None
    lowest_score = 100
    for word in words:
      if word in self.word_scores:
        if self.word_scores[word] < lowest_score:
          lowest_score = self.word_scores[word]
          word_to_return = word
    print("Found word with lowest score:", word_to_return, lowest_score)
    return word_to_return


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
    print(self.word_scores)

