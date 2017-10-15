"""
This is a class defines different "cosmetic" (i.e. not necessarily in the Agent ActionSpace)
Robot Behaviors
"""
# -*- coding: utf-8 -*-
# pylint: disable=import-error, invalid-name
from unity_game_msgs.msg import iSpyCommand
from enum import Enum

class RobotBehaviors:  # pylint: disable=no-member, too-many-instance-attributes
    """
    A Class definition for "cosmetic" robot behavior strings, which get translated by the ROSNodeMgr
    """

    # Look Ats
    LOOK_AT_TABLET = 'LOOK_AT_TABLET'
    LOOK_CENTER= 'LOOK_CENTER'
    WIN_SPEECH = "WIN_SPEECH"
    

    #Pronunciation Actions the robot can do 
    PRONOUNCE_CORRECT = 'PRONOUNCE_CORRECT'

    #Reaction Actions the robot can do after its results are revealed
    REACT_ROBOT_CORRECT = 'REACT_ROBOT_CORRECT'
    REACT_CHILD_CORRECT = 'REACT_CHILD_CORRECT'

    

    # virtual actions on the app
    VIRTUALLY_CLICK_CORRECT_OBJ = "CLICK_CORRECT_OBJ" # click correct obj

class RobotRoles(Enum):
    '''
    contains a list of social roles that are avaiable to robot to perform
    '''
    CUR_EXPERT = 1
    #CUR_NOVICE = 2
    #NCUR_EXPERT = 3
    #NCUR_NOVICE = 4

class RobotRolesBehaviorsMap:
    '''
    mapping between robot's social role and robot's specific behaviors
    '''  
    def __init__(self):
        self.mapping = {}
        self.mapping.update({RobotRoles.CUR_EXPERT:{'physical':RobotBehaviors.LOOK_AT_TABLET, 'virtual': RobotBehaviors.VIRTUALLY_CLICK_CORRECT_OBJ}})

    def get_actions(self,role):
        if isinstance(role, RobotRoles) and role in self.mapping.keys():
            # if the role is an instance of robotRoles
            # return a sequence of specific actions that the robot needs to execute for a given role
            return self.mapping[role]
        else:
            return []