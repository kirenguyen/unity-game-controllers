"""
This is a module that exports certain global runtime settings
"""
# -*- coding: utf-8 -*-

USE_ROS = True

class TapGameLog(): # pylint: disable=too-few-public-methods
    """
    this is a mock class that allows tests to pass in a non-ROS environment
    """

    CHECK_IN = "CHECK_IN"
    GAME_START_PRESSED = "GAME_START_PRESSED"
    INIT_ROUND_DONE = "INIT_ROUND_DONE"
    START_ROUND_DONE = "START_ROUND_DONE"
    ROBOT_RING_IN = "ROBOT_RING_IN"
    PLAYER_RING_IN = "PLAYER_RING_IN"
    RESET_NEXT_ROUND_DONE = "RESET_NEXT_ROUND_DONE"
    SHOW_GAME_END_DONE = "SHOW_GAME_END_DONE"
    START_PRONUNCIATION_PANEL_DONE = "START_PRONUNCIATION_PANEL_DONE"
    END_PRONUNCIATION_PANEL_DONE = "END_PRONUNCIATION_PANEL_DONE"
    SHOW_RESULTS_DONE = "SHOW_RESULTS_DONE"

    def __init__(self):
        pass

class TapGameCommand(): # pylint: disable=too-few-public-methods
    """
    this is a mock class that allows tests to pass in a non-ROS environment
    """

    INIT_ROUND = "INIT_ROUND"
    START_ROUND = "START_ROUND"
    ROBOT_RING_IN = "ROBOT_RING_IN"
    RESET_NEXT_ROUND = "RESET_NEXT_ROUND"
    SHOW_GAME_END = "SHOW_GAME_END"
    START_PRONUNCIATION_PANEL = "START_PRONUNCIATION_PANEL"
    SHOW_RESULTS = "START_PRONUNCIATION_PANEL"

    def __init__(self):
        pass