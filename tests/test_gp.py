# -*- coding: utf-8 -*-

from .context import StudentModel
from .context import Curriculum
import numpy as np

import unittest


class GPTestSuite(unittest.TestCase):
    """Advanced test cases."""

    def test_GP(self):
        my_GP = StudentModel()
        valid_words = [p for p in dir(Curriculum)
                       if isinstance(getattr(Curriculum, p), str)
                       and not p.startswith('__')]
        self.assertEqual(my_GP.get_next_best_word() in valid_words, True)


    def test_kernel(self):
        my_GP = StudentModel()

        self.assertGreater(my_GP.get_word_cov('DOG', 'DADA'), my_GP.get_word_cov('DOG', 'ALPHABET'))

    def graph_test(self):
        msm = StudentModel()
        Xtrain = ['BEE', 'SNAKE', 'TIGER']  # these numbers are just labels
        Ytrain = [.66, .66, .66]  # these numbers correspond to 'Correct' demonstrations

        msm.train_and_compute_posterior(Xtrain, Ytrain)
        msm.plot_curricular_distro()

        Xtrain = ['BEE', 'SNAKE', 'TIGER']  # these numbers are just labels
        Ytrain = [1, 1, 1]  # these numbers correspond to 'Correct' demonstrations

        msm.train_and_compute_posterior(Xtrain, Ytrain)
        msm.plot_curricular_distro()



if __name__ == '__main__':
    unittest.main()
