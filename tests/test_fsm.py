# -*- coding: utf-8 -*-

from .context import TapGameFSM

import unittest


class FSMTestSuite(unittest.TestCase):
    """Advanced test cases."""

    def test_FSM(self):
        my_FSM = TapGameFSM()
        self.assertEqual(my_FSM.state, 'GAME_START')

        my_FSM.initFirstRound()
        self.assertEqual(my_FSM.state, 'ROUND_START')

        my_FSM.startRound()
        self.assertEqual(my_FSM.state, 'ROUND_ACTIVE')

        my_FSM.robotRingIn()
        self.assertEqual(my_FSM.state, 'ROUND_END')

        my_FSM.evaluate_round()

        self.assertEqual(my_FSM.state, 'ROUND_START')

        my_FSM.startRound()
        self.assertEqual(my_FSM.state, 'ROUND_ACTIVE')

        my_FSM.robotRingIn()
        self.assertEqual(my_FSM.state, 'ROUND_END')

        my_FSM.evaluate_round()
        self.assertEqual(my_FSM.state, 'GAME_FINISHED')



if __name__ == '__main__':
    unittest.main()
