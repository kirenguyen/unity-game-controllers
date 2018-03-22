"""
This file implements the student model for the storybook controller.

The model attempts to update the belief that a student can correctly pronounce
a word, and the belief that a student knows the meaning of that word. These
beliefs are correlated but updated under different criteria.
"""

import storybook_controller.robot_feedback as robot_feedback

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
    self.current_sentences = []
    self.sentences_by_page_number = {}

  def update_with_duration(self, duration, text):
    """
    Update model given that the given text took <duration> seconds for
    the child to speak.
    """
    pass

  def update_with_speechace_result(self, speechace_result):
    """
    Updates the model given the result of a recent speechace analysis.
    """
    print("Updating with speech ace results!")
    if "text_score" not in speechace_result:
      print("--- Got speech ace results but no text score, likely speech was empty")
      return
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

  def update_with_correct_word_tapped(self, word):
    """
    Update model given that the child tapped on a word correctly.
    """
    pass

  def update_with_incorrect_word_tapped(self, expected_word, word):
    """
    Update model given that the child tapped on <word> when asked to tap on
    <expected_word>.
    """
    pass

  def update_with_correct_scene_object_tapped(self, label):
    """
    Update model given that the child tapped on a scene object correctly.
    """
    pass


  def update_with_incorrect_scene_object_tapped(self, expected_label, label):
    """
    Update model given that the child tapped on a scene object with label <label>
    when asked to tap on an object with label <expected_label>
    """
    pass

  def update_with_correct_word_pronounced(self, word):
    """
    Update model given that the child was shown the text of a word and they
    pronounced it correctly.
    """
    pass

  def update_with_explore_word_tapped(self, word):
    """
    Update model given that child tapped on a word while in explore mode.
    """
    pass

  def update_with_explore_scene_object_tapped(self, label):
    """
    Update model given that child tapped on a scene object with the given label
    while in explore mode.
    """

  def is_child_turn(self, sentence_index):
    """
    In a turn taking exercise, returns whether or not the child should be
    asked to read the sentence at the given index.
    """
    return True

  def update_sentences(self, page_num, sentences):
    self.sentences_by_page_number[page_num] = []
    for sentence in sentences:
      self.sentences_by_page_number[page_num].append(sentence.split())
    self.current_sentences = self.sentences_by_page_number[page_num]
    # print("Sentences by page number: " + str(self.sentences_by_page_number))

  def get_end_page_questions(self, max_num_questions=1):
    """
    Returns an array of at most max_num_questions EndPageQuestion objects.
    """
    # For now, just return one question, always asking child to tap on a word.
    word = self.get_lowest_pronunciation_score_word()
    return [robot_feedback.EndPageQuestionWordTap(word)]

  def get_lowest_pronunciation_score_word(self, sentence_index=None):
    """
    Given an array of words (likely representing a sentence), return the word
    that the child performs the worst on.
    TODO: add in meaning/phonemes as well as simple average word scores.
    """
    sentences = None
    if sentence_index is None:
      # Default is to use current sentences.
      sentences = self.current_sentences
    elif sentence_index < 0 or sentence_index >= len(self.sentences_by_page_number.keys()):
      print("No such sentence_index")
      raise Exception("No such sentence index", sentence_index, len(self.sentences_by_page_number.keys()))
    else:
      sentences = self.sentences_by_page_number[sentence_index]

    words = []
    for sentence in sentences:
      words += sentence
    word_to_return = None
    lowest_score = 100
    for word in words:
      if word in self.word_scores:
        avg_score = sum(self.word_scores[word]) * 1.0 / len(self.word_scores[word])
        if avg_score < lowest_score:
          lowest_score = avg_score
          word_to_return = word
    print("Found word with lowest score:", word_to_return, lowest_score)
    if word_to_return is None:
      print("No speechace results yet, choosing longest word")
      max_length = max(map(len, words))
      word_to_return = [w for w in words if len(w) == max_length][0]
    print("Returning word:", word_to_return)
    return word_to_return

  def plot_distribution(self):
    """
    Plot the current distribution.
    """
    print(self.word_scores)


