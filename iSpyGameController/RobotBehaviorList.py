"""
This is a class defines different "cosmetic" (i.e. not necessarily in the Agent ActionSpace)
Robot Behaviors
"""
# -*- coding: utf-8 -*-
# pylint: disable=import-error, invalid-name


class RobotBehaviors:  # pylint: disable=no-member, too-many-instance-attributes
    """
    A Class definition for "cosmetic" robot behavior strings, which get translated by the ROSNodeMgr
    """

    # Look Ats
    LOOK_AT_TABLET = 'LOOK_AT_TABLET'
    LOOK_CENTER= 'LOOK_CENTER'

    # Action Space Actions
    RING_ANSWER_CORRECT = 'RING_ANSWER_CORRECT'
    LATE_RING = 'RING_ANSWER_CORRECT'

    #Pronunciation Actions the robot can do 
    PRONOUNCE_CORRECT = 'PRONOUNCE_CORRECT'

    #Reaction Actions the robot can do after its results are revealed
    REACT_ROBOT_CORRECT = 'REACT_ROBOT_CORRECT'
    REACT_CHILD_CORRECT = 'REACT_CHILD_CORRECT'

    # Reactions to the robot getting beaten on the buzz
    REACT_TO_BEAT = 'REACT_TO_BEAT' # played when the robot buzzes in, but player already got there
    REACT_TO_BEAT_CORRECT = 'REACT_TO_BEAT_CORRECT' # played when players results are revealed
    REACT_TO_BEAT_WRONG = 'REACT_TO_BEAT_WRONG' # played when players results are revealed
