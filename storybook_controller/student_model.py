"""
This file implements the student model for the storybook controller.

The model attempts to update the belief that a student can correctly pronounce
a word, and the belief that a student knows the meaning of that word. These
beliefs are correlated but updated under different criteria.
"""

class StudentModel(object):
  def __init__(self):
    self.vocabulary = None
    self.pronounce_dist = None
    self.meaning_dist = None

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

  def get_pronounce_belief(self, word):
    """
    Returns the likelihood that the student can pronounce the word correctly.
    """
    raise NotImplementedError

  def get_meaning_belief(self, word):
    """
    Returns the likelihood that the student knows the meaning of the word.
    """
    raise NotImplementedError

  def update_with_correct_pronunciation(self, word):
    """
    Updates the model given that the student pronounced the given word
    correctly.
    """
    raise NotImplementedError

  def update_with_correct_meaning(self, word):
    """
    Updates the model given that the student demonstrated knowledge of the
    meaning of the given word.
    """
    raise NotImplementedError

  def plot_distribution(self):
    """
    Plot the current distribution.
    """
    raise NotImplementedError

