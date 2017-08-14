"""
This is a module that exports certain global runtime settings
"""
# -*- coding: utf-8 -*-

USE_ROS = True

# True is using USB mic, False if using Tega microphone
USE_USB_MIC = False

# Scene options are "BEDROOM", "OUTDOORS", "INDOORS"
# Use this to change the tasks to correspond with what scene you are using
SCENE = "BEDROOM"


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
        START_BUTTON_PRESSED = "startButtonPressed"
        TOPLEFT_BUTTON_PRESSED = "topLeftButtonPressed"
        OBJECT_CLICKED = "objectClicked"
        SAY_BUTTON_PRESSED = "sayButtonPressed"
        CLOSE_BUTTON_PRESSED = "closeButtonPressed"
        PRACTICE_FINISHED = "practiceFinished"
        PRACTICE_FAILED = "practiceFailed"
        N_SECONDS_LATER = "secondsLater"
        triggers = ["startButtonPressed","topLeftButtonPressed","objectClicked","sayButtonPressed", "closeButtonPressed", "practiceFinished", "practiceFailed", "secondsLater"]

    def __init__(self):
        pass