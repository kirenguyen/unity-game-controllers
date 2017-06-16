# -*- coding: utf-8 -*-

from .context import StudentModel
from .context import Curriculum

import unittest


class GPTestSuite(unittest.TestCase):
    """Advanced test cases."""

    def test_GP(self):
        my_GP = StudentModel()
        valid_words = [p for p in dir(Curriculum)
                       if isinstance(getattr(Curriculum, p), str)
                       and not p.startswith('__')]
        self.assertEqual(my_GP.get_next_best_word() in valid_words, True)


if __name__ == '__main__':
    unittest.main()
