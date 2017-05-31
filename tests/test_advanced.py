# -*- coding: utf-8 -*-

from .context import GameController

import unittest


class AdvancedTestSuite(unittest.TestCase):
    """Advanced test cases."""

    def test_thoughts(self):
        my_GC = GameController('a', 'b')
        self.assertIsNone(my_GC.hmm())


if __name__ == '__main__':
    unittest.main()
