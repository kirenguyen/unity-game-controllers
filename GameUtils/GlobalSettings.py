"""
This is a module that exports certain global runtime settings
"""
# -*- coding: utf-8 -*-

USE_ROS = True
USE_TEGA = False # if False, we are using Jibo
USE_SPACY = False # if False, do not load the full SpaCy language model
USE_USB_MIC = True #if True, use the external USB microphone

DO_EPSILON_INCREASING_POLICY = True # if True, agent use a increasing-ratio of Ringing / Waiting
                                    # actions. Otherwise, chooses randomly

DO_ACTIVE_LEARNING = True #if True, use the active learning protocol in the StudentModel
                          # otherwise choose words randomly

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
    END_ROUND_DONE = "END_ROUND_DONE"
    SHOW_RESULTS_DONE = "SHOW_RESULTS_DONE"
    PLAYER_BEAT_ROBOT = "PLAYER_BEAT_ROBOT"
    RESTART_GAME = "RESTART_GAME"

    def __init__(self):
        pass

class iSpyGameStates(): # pylint: disable=too-few-public-methods
    """
    this is a mock class that allows tests to pass in a non-ROS environment
    """

    GAME_START = "gameSTART"
    EXPLORATION_MODE = "explorationMODE"
    MISSION_MODE = "missionMODE"
    PRONUNCIATION_PANEL = "pronunciationPANEL"
    PRONUNCIATION_RESULT = "pronunciationRESULT"
    WORD_DISPLAY = "wordDISPLAY"

    class Triggers():
        SCREEN_MOVED = "SCREEN_MOVED"

        START_BUTTON_PRESSED = "START_BUTTON_PRESSED"
        TOPLEFT_BUTTON_PRESSED = "TOPLEFT_BUTTON_PRESSED"
        OBJECT_CLICKED = "OBJECT_CLICKED"
        SAY_BUTTON_PRESSED = "SAY_BUTTON_PRESSED"
        TARGET_OBJECT_COLLECTED = "TARGET_OBJECT_COLLECTED"
        NONTARGET_OBJECT_COLLECTED = "NONTARGET_OBJECT_COLLECTED"
        PRONUNCIATION_PANEL_CLOSED = "PRONUNCIATION_PANEL_CLOSED"
        PRACTICE_FINISHED = "PRACTICE_FINISHED"
        PRACTICE_FAILED = "PRACTICE_FAILED"
        N_SECONDS_LATER = "N_SECONDS_LATER"
        
        triggers = [N_SECONDS_LATER, PRACTICE_FAILED,PRACTICE_FINISHED ,PRONUNCIATION_PANEL_CLOSED,NONTARGET_OBJECT_COLLECTED, TARGET_OBJECT_COLLECTED, SAY_BUTTON_PRESSED,SCREEN_MOVED, START_BUTTON_PRESSED ,TOPLEFT_BUTTON_PRESSED,OBJECT_CLICKED]

    def __init__(self):
        pass

class iSpyRobotInteractionStates():
    ROBOT_TURN="robotTURN"
    CHILD_TURN="childTURN"
    ROBOT_HELP = "robotHelp"
    CHILD_HELP = "childHelp"
    QUESTION_ASKING="questionAsking"

    class Triggers():
        CHILD_TURN_DONE="childTurnDone"
        ROBOT_TURN_DONE="robotTurnDone"
        ROBOT_HELP_TRIGGER="robotHelpTigger"
        CHILD_HELP_TRIGGER="childHelpTrigger"
        ROBOT_QUESTION="robotQuestion"

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

class iSpyGameCommand():
    #Recording Time Constant
    RECORD_TIME_MS = 3500

    # COMMAND CONSTANTS
    RESET = 0
    SHOW_PRONOUNCIATION_PANEL = 1
    SHOW_OBJECT_DESCR_PANEL = 2
    ROBOT_EXPERT_ROLE = 3
    SEND_PRONOUNCIATION_ACCURACY_TO_UNITY = 10
    SEND_TASKS_TO_UNITY = 20
    GAME_FINISHED = 99
    BUTTON_DISABLED=31


class TegaAction(): # pylint: disable=too-few-public-methods
    """
    this is a mock class for Tega Actions that allows tests to pass in a non-ROS environment
    """

    do_motion = True
    CONFIRM = "CONFIRM"

    def __init__(self):
        pass

class JiboAction(): # pylint: disable=too-few-public-methods
    """
    this is a mock class  for Jibo Actions that allows tests to pass in a non-ROS environment
    """

    do_motion = True
    CONFIRM = "CONFIRM"
