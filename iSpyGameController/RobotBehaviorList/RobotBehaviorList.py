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

    ROBOT_TURN_SPEECH="ROBOT_TURN_SPEECH"
    REACT_CHILD_ANSWER_CORRECT="REACT_CHILD_ANSWER_CORRECT"
    REACT_CHILD_ANSWER_WRONG="REACT_CHILD_ANSWER_WRONG"
    REACT_GAME_START ="REACT_GAME_START"
    REACT_GAME_START2="REACT_GAME_START2"

    WIN_MOTION="WIN_MOTION"
    EYE_FIDGET="EYE_FIDGET"

    RING_ANSWER_CORRECT="RING_ANSWER_CORRECT"


    #Pronunciation Actions the robot can do 
    PRONOUNCE_CORRECT = 'PRONOUNCE_CORRECT'

    #Reaction Actions the robot can do after its results are revealed
    REACT_ROBOT_CORRECT = 'REACT_ROBOT_CORRECT'
    REACT_CHILD_CORRECT = 'REACT_CHILD_CORRECT'



    # virtual actions on the app
    VIRTUALLY_CLICK_CORRECT_OBJ = "CLICK_CORRECT_OBJ" # click correct obj
    VIRTUALLY_CLICK_WRONG_OBJ = "CLICK_WRONG_OBJ"

class RobotRoles(Enum):
    '''
    contains a list of social roles that are avaiable to robot to perform
    '''
    COMPLETE_EXPERT = 0
    COMPLETE_NOVICE = 1
    #SEMI_EXPERT = 2
    #SEMI_NOVICE = 3

class RobotActionSequence:
    TURN_STARTED = "turnStarted"
    SCREEN_MOVED = "screenMoved"
    OBJECT_FOUND = "objectFound"
    OBJECT_CLICKED = "objectClicked"
    OBJECT_PRONOUNCED = "objectPronounced"
    RESULTS_RETURNED = "ResultsReturned"
    TURN_FINISHED = "TurnFinished"
    WRONG_OBJECT_FAIL ="WRONG_OBJECT_FAIL"

    class Triggers:
        NEXT = "Next"
        RESET = "Reset"
            

class RobotRolesBehaviorsMap:
    '''
    mapping between robot's social role and robot's specific behaviors
    '''  
    def __init__(self):
        self.mapping = {}
        self.complete_expert_role()
        self.complete_novice_role()
        self.robot_response()
        
    def get_actions(self,role):
        if role in self.mapping.keys():
            # if the role is an instance of robotRoles
            # return a sequence of specific actions that the robot needs to execute for a given role
            return self.mapping[role]
        else:
            return []

    def complete_expert_role(self):
        '''
        expert role for robot's turn
        '''
        self.mapping.update({
            RobotRoles.COMPLETE_EXPERT:{
            'physical':{
                RobotActionSequence.TURN_STARTED: [ RobotBehaviors.LOOK_AT_TABLET, RobotBehaviors.ROBOT_TURN_SPEECH ],
                RobotActionSequence.SCREEN_MOVED: [],
                RobotActionSequence.OBJECT_FOUND: [],
                RobotActionSequence.OBJECT_CLICKED: [RobotBehaviors.EYE_FIDGET,RobotBehaviors.RING_ANSWER_CORRECT], 
                RobotActionSequence.OBJECT_PRONOUNCED: [RobotBehaviors.PRONOUNCE_CORRECT],
                RobotActionSequence.RESULTS_RETURNED: [RobotBehaviors.WIN_SPEECH],
                RobotActionSequence.TURN_FINISHED:[]
                }, 
            'virtual':
                RobotBehaviors.VIRTUALLY_CLICK_CORRECT_OBJ
                
            }
        })

    def complete_novice_role(self):
        '''
        novice role for robot's turn
        '''
        self.mapping.update({
            RobotRoles.COMPLETE_NOVICE:{
            'physical':{
                RobotActionSequence.TURN_STARTED: [ RobotBehaviors.LOOK_AT_TABLET, RobotBehaviors.ROBOT_TURN_SPEECH ],
                RobotActionSequence.SCREEN_MOVED: [],
                RobotActionSequence.OBJECT_FOUND: [],
                RobotActionSequence.OBJECT_CLICKED: [RobotBehaviors.EYE_FIDGET,RobotBehaviors.RING_ANSWER_CORRECT], 
                RobotActionSequence.OBJECT_PRONOUNCED: [RobotBehaviors.PRONOUNCE_CORRECT],
                RobotActionSequence.RESULTS_RETURNED: [RobotBehaviors.WIN_SPEECH],
                RobotActionSequence.TURN_FINISHED:[]
                }, 
            'virtual':
                RobotBehaviors.VIRTUALLY_CLICK_WRONG_OBJ
                
            }
        })
    
    def robot_response(self):
        '''
        robot's response to child during child's turn
        '''
        self.mapping.update({
            "Response":{
            'physical':{
                RobotActionSequence.TURN_STARTED: [  ],
                RobotActionSequence.SCREEN_MOVED: [],
                RobotActionSequence.OBJECT_FOUND: [],
                RobotActionSequence.OBJECT_CLICKED: [RobotBehaviors.EYE_FIDGET], 
                RobotActionSequence.OBJECT_PRONOUNCED: [],
                RobotActionSequence.RESULTS_RETURNED: [],
                RobotActionSequence.TURN_FINISHED:[RobotBehaviors.REACT_CHILD_ANSWER_CORRECT, RobotBehaviors.RING_ANSWER_CORRECT],
                RobotActionSequence.WRONG_OBJECT_FAIL: [RobotBehaviors.REACT_CHILD_ANSWER_WRONG, RobotBehaviors.LOOK_CENTER]
                }, 
            'virtual':
                RobotBehaviors.VIRTUALLY_CLICK_CORRECT_OBJ
                
            }
        })


