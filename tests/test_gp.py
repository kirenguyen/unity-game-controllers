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

        self.assertEqual(my_GP.get_word_cov('dog', 'dada'), .43)
        self.assertEqual(my_GP.get_word_cov('SNAKE', 'SILK'), .44)

    def graph_test(self):
        my_sm = StudentModel()
        my_sm.plot_curricular_distro([1] * 8, [1] * 8)



if __name__ == '__main__':
    unittest.main()
