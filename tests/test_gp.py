# -*- coding: utf-8 -*-

from .context import StudentModel
from .context import Curriculum

import unittest


class GPTestSuite(unittest.TestCase):
    """Advanced test cases."""

    def test_GP(self):
        my_GP = StudentModel()
        self.assertEqual(my_GP.get_next_best_word(), Curriculum.CHICKEN)


if __name__ == '__main__':
    unittest.main()
