"""
This file implements the student model for the storybook controller.

The model attempts to update the belief that a student can correctly pronounce
a word, and the belief that a student knows the meaning of that word. These
beliefs are correlated but updated under different criteria.
"""

import random
import sys
import storybook_controller.robot_feedback as robot_feedback
import nltk

"""
Set up phoneme information.
"""

# Takes about 6 seconds to load this.
# ARPABET = nltk.corpus.cmudict.dict()

# # These are the 39 phonemes we should expect to see.
# # Doesn't include stress, which is 0, 1 or 2 immediately following a phoneme.
# ARPABET_PHONEMES = [
#   "aa", "ae", "ah", "ao", "aw", "ay", "b", "ch", "d", "dh", "eh", "er", "ey",
#   "f", "g", "hh", "ih", "iy", "jh", "k", "l", "m", "n", "ng", "ow", "oy", "p",
#   "r", "s", "sh", "t", "th", "uh", "uw", "v", "w", "y", "z", "zh"
# ]

# print("Loaded arpabet phonemes dict!")

"""
Updates model with inputs from SpeechACE and the child's answers to questions.
Can be queried for appropriate questions Jibo should ask the child.
"""
class StudentModel(object):
  def __init__(self):
    # Threshold above which we don't consider the word as a priority for improving,
    # meaning that it is likely not to be asked about by Jibo.
    self.GOOD_PRONOUNCE_THRESHOLD = 90.0 # Out of 100

    # Dictionary representing the vocabulary.
    # Choose dictionary instead of list for faster existence checks.
    self.vocabulary = {}
    
    self.word_pronunciation_scores = {}
    self.phoneme_scores = {}
    self.word_tap_scores = {} # Map word to array of ints where 1 means correct, 0 incorrect.
    self.scene_object_tap_scores = {} # Map label to array of ints similar to above.
    self.confused_word_pairs = [] # Array of tuples of expected word and child's (wrong) word.
    self.confused_label_pairs = [] # Array of tuples of expected label and child's label.

    # Keep track of trend of overall scores per page.
    self.overall_quality_scores = []

    # Current sentences on the page in evaluate mode.
    self.current_sentences = []
    self.sentences_by_page_number = {}
    self.current_scene_objects = []

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
      if word not in self.word_pronunciation_scores:
        self.word_pronunciation_scores[word] = []
      self.word_pronunciation_scores[word].append(word_score["quality_score"])
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
    if word not in self.word_tap_scores:
      self.word_tap_scores[word] = []
    self.word_tap_scores[word].append(1)

  def update_with_incorrect_word_tapped(self, expected_word, word):
    """
    Update model given that the child tapped on <word> when asked to tap on
    <expected_word>.
    """
    if word not in self.word_tap_scores:
      self.word_tap_scores[word] = []
    self.word_tap_scores[word].append(0)
    self.confused_word_pairs.append((expected_word, word))

  def update_with_correct_scene_object_tapped(self, label):
    """
    Update model given that the child tapped on a scene object correctly.
    """
    if label not in self.scene_object_tap_scores:
      self.scene_object_tap_scores[label] = []
    self.scene_object_tap_scores[label].append(1)


  def update_with_incorrect_scene_object_tapped(self, expected_label, label):
    """
    Update model given that the child tapped on a scene object with label <label>
    when asked to tap on an object with label <expected_label>
    """
    if label not in self.scene_object_tap_scores:
      self.scene_object_tap_scores[label] = []
    self.scene_object_tap_scores[label].append(0)
    self.confused_label_pairs.append((expected_label, label))

  def update_with_correct_word_pronounced(self, word):
    """
    Update model given that the child was shown the text of a word and they
    pronounced it correctly.
    """
    pass

  def update_with_incorrect_word_pronounced(self, word):
    """
    Update model given that the child was shown the text of a word and they
    pronounced it incorrectly.
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
    pass

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

  def update_scene_objects(self, scene_objects):
    self.current_scene_objects = scene_objects

  def get_end_page_questions(self, max_num_questions=1):
    """
    Returns an array of at most max_num_questions EndPageQuestion objects.
    """
    word = self.get_lowest_pronunciation_score_word()
    use_word = random.random()
    print("use word", use_word)
    if word is not None and use_word < .5:
      indexes = self.get_word_indexes(word)
      return [robot_feedback.EndPageQuestionWordTap(word, indexes)]
    else:
      label = self.get_scene_object_label_to_evaluate()
      if label is None:
        # Must ask for a word if no scene objects exist.
        word = self.get_lowest_pronunciation_score_word(True)
        indexes = self.get_word_indexes(word)
        return [robot_feedback.EndPageQuestionWordTap(word, indexes)]
      else:
        ids = self.get_scene_object_ids(label)
        return [robot_feedback.EndPageQuestionSceneObjectTap(label, ids)]

  def get_lowest_pronunciation_score_word(self, force=False):
    """
    Return a word from a sentence on the current page that has a low
    pronunciation score.

    TODO: add in phonemes as well as simple average word scores.
    Meaning will be handle by tapping.

    If force=False, the returned word can be None if there is no evaluation yet or
    if none of the words have a score lower than some threshold.

    If force=True, the returned word will always be a valid word.
    """
    print("all scores:", self.word_pronunciation_scores)
    print("current sentences", self.current_sentences)
    words = []
    for sentence in self.current_sentences:
      for w in sentence:
        words.append(self.strip_punctuation(w.lower()))
    word_to_return = None
    lowest_score = 100
    for word in words:
      if word in self.word_pronunciation_scores:
        print("scores: ", self.word_pronunciation_scores[word])
        avg_score = sum(self.word_pronunciation_scores[word]) * 1.0 / len(self.word_pronunciation_scores[word])
        if avg_score < lowest_score:
          lowest_score = avg_score
          word_to_return = word
    print("Found word with lowest score:", word_to_return, lowest_score)
    if word_to_return is None or lowest_score > self.GOOD_PRONOUNCE_THRESHOLD:
      if force:
        print("No speechace results yet, choosing longest word")
        max_length = max(map(len, words))
        word_to_return = [w for w in words if len(w) == max_length][0]
        print("Longest word:", word_to_return)
      else:
        return None
    return word_to_return

  def get_scene_object_label_to_evaluate(self, force_in_text=True):
    """
    Returns a label from the current scene objects on the page.

    If force_in_text is True, can ONLY return labels that appear in the text.
    """

    # Give priority to words the child didn't get right before, and words
    # we haven't asked the child to try yet. Or words the child confused before?
    lowest_score = 1
    label_to_return = None
    not_asked = []
    all_valid_labels = []
    for scene_object in self.current_scene_objects:
      if scene_object.in_text or not force_in_text:
        label = scene_object.label
        if label in self.scene_object_tap_scores:
          # Percentage of correct responses.
          score = sum(self.scene_object_tap_scores[label]) * 1.0 / len(self.scene_object_tap_scores[label])
          if score < lowest_score:
            lowest_score = score
            label_to_return = label
        else:
          not_asked.append(label)
        all_valid_labels.append(label)
    if label_to_return is not None:
      print("Found label with low score:", label_to_return, lowest_score)
      return label_to_return
    else:
      # If there are no labels at all.
      if len(all_valid_labels) == 0:
        return None 
      # If there are no labels that we haven't asked about, return a random
      # label from the valid set.
      if len(not_asked) == 0:
        random_label = all_valid_labels[random.randint(0, len(all_valid_labels) - 1)]
        print("Using random label:", random_label)
        return random_label
      # Otherwise, pick a label that we haven't asked about.
      else:
        # Note that randint is inclusive on both endpoints.
        not_asked_label = not_asked[random.randint(0, len(not_asked)-1)]
        print("Using a label that we haven't used before:", not_asked_label)
        return not_asked_label

  """"
  Helpers
  """

  def get_word_indexes(self, word):
    # Go through all sentences and find the words that match.
    indexes = []
    i = 0
    for sentence in self.current_sentences:
      for w in sentence:
        if self.strip_punctuation(w.lower()) == word:
          indexes.append(i)
        i += 1
    print("get indexes", self.current_sentences, word, indexes)
    return indexes

  def get_scene_object_ids(self, label):
    ids = []
    stripped_label = self.strip_punctuation(label.lower())
    for s in self.current_scene_objects:
      if self.strip_punctuation(s.label.lower()) == stripped_label:
        ids.append(s.id)
    print("get ids", self.current_scene_objects, label, ids)
    return ids

  def get_phonemes(self, word):
    if word not in ARPABET:
      return None
    phonemes_raw = ARPABET[word][0]
    # Remove stress and turn phonemes to lowercase.
    phonemes = ["".join(filter(lambda c: c.isalpha(), p)).lower() \
                for p in phonemes_raw]
    return phonemes

  def plot_distribution(self):
    """
    Plot the current distribution.
    """
    print(self.word_pronunciation_scores)

  def strip_punctuation(self, word):
    return "".join([c for c in word if c.isalnum()])
