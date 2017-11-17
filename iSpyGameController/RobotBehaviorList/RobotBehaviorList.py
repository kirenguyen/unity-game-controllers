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
    LOOK_CENTER = 'LOOK_CENTER'
    LOOK_LEFT_RIGHT = 'LOOK_LEFT_RIGHT'
    LOOK_DOWN_CENTER = 'LOOK_DOWN_CENTER'

    # Positive Emotions
    ROBOT_EXCITED = 'ROBOT_EXCITED'
    ROBOT_INTERESTED = 'ROBOT_INTERESTED'
    ROBOT_YES = 'ROBOT_YES'
    ROBOT_HAPPY_DANCE = 'ROBOT_HAPPY_DANCE'
    ROBOT_CURIOUS = 'ROBOT_CURIOUS'
    ROBOT_ATTENTION = 'ROBOT_ATTENTION' # Pose Forward 
    ROBOT_CELEBRATION = 'ROBOT_CELEBRATION'
    ROBOT_ENCOURAGING = 'ROBOT_ENCOURAGING'
    ROBOT_WINK = 'ROBOT_WINK'
    ROBOT_THINKING = 'ROBOT_THINKING'
    
    # Negative Emotions
    ROBOT_SAD = 'ROBOT_SAD'
    ROBOT_UNSURE = 'ROBOT_UNSURE'
    ROBOT_COMFORT = 'ROBOT_COMFORT'
    ROBOT_ASK_HELP = 'ROBOT_ASK_HELP'
    ROBOT_DISAPPOINTED = 'ROBOT_DISAPPOINTED'

    # Speech
    # Game explanation speech 
    ROBOT_SAY_WORD = 'ROBOT_SAY_WORD'
    ROBOT_EXPLANATION_SPEECH = ''

    # Emotion speech
    ROBOT_ENCOURAGING_SPEECH = ''
    ROBOT_ATTENTION_SPEECH = ''
    ROBOT_COMFORT_SPEECH = ''
    ROBOT_UNCERTAIN_SPEECH = ''
    ROBOT_ASK_HELP_SPEECH = ''
    ROBOT_CONFIDENCE_SPEECH = ''
    ROBOT_HAPPY_SPEECH = ''
    ROBOT_SURPRISED_SPEECH = ''
    ROBOT_CONFIRM_SPEECH = ''
    ROBOT_THANKYOU_SPEECH = ''

    # Turn speech
    ROBOT_CHILD_TURN_SPEECH = ''
    ROBOT_TURN_SPEECH = ''

    # virtual actions on the app
    VIRTUALLY_CLICK_CORRECT_OBJ = "CLICK_CORRECT_OBJ" # click correct obj


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

    """

class RobotRoles(Enum):
    '''
    contains a list of social roles that are avaiable to robot to perform
    '''

    EXPERT = 0
    COMPETENT = 1
    NOVICE = 2


class RobotActionSequence:

    TURN_STARTED = "turnStarted" 
    SCREEN_MOVED = "screenMoved"
    OBJECT_FOUND = "objectFound"
    OBJECT_CLICKED = "objectClicked"
    OBJECT_PRONOUNCED = "objectPronounced" 
    RESULTS_RETURNED = "ResultsReturned" 
    TURN_FINISHED = "TurnFinished" 
    PRONOUNCE_CORRECT = "PronounceCorrect"
    WRONG_OBJECT_FAIL ="WRONG_OBJECT_FAIL" 

    class Triggers:
        NEXT = "Next"
        RESET = "Reset"

    """
    TURN_STARTED = "turnStarted"
    SCREEN_MOVED = "screenMoved"
    OBJECT_FOUND = "objectFound"
    OBJECT_CLICKED = "objectClicked"
    OBJECT_PRONOUNCED = "objectPronounced"
    RESULTS_RETURNED = "ResultsReturned"
    TURN_FINISHED = "TurnFinished"
    WRONG_OBJECT_FAIL ="WRONG_OBJECT_FAIL"
    """
            

class RobotRolesBehaviorsMap:
    '''
    mapping between robot's social role and robot's specific behaviors
    '''  
    def __init__(self):
        self.mapping = {}

        self.expert_role()
        self.competent_role()
        self.novice_role()
        self.robot_response()

        """
        self.complete_expert_role()
        self.complete_novice_role()
        """
        
    def get_actions(self,role):
        if role in self.mapping.keys():
            # if the role is an instance of robotRoles
            # return a sequence of specific actions that the robot needs to execute for a given role
            return self.mapping[role]
        else:
            print("ERROR: Cannot find a set of actions for the chosen role: "+role)
            return []

    def expert_role(self):
        '''
        expert role for robot's turn
        '''
        self.mapping.update({
            RobotRoles.EXPERT:{
            'physical':{
                RobotActionSequence.TURN_FINISHED: [RobotBehaviors.LOOK_AT_TABLET, RobotBehaviors.ROBOT_EXCITED],
                RobotActionSequence.TURN_STARTED: [RobotBehaviors.LOOK_AT_TABLET, RobotBehaviors.ROBOT_ATTENTION],
                RobotActionSequence.SCREEN_MOVED: [],
                RobotActionSequence.OBJECT_CLICKED: [RobotBehaviors.ROBOT_CURIOUS], 
                RobotActionSequence.OBJECT_FOUND: [],
                RobotActionSequence.OBJECT_PRONOUNCED: [RobotBehaviors.ROBOT_SAY_WORD], 
                RobotActionSequence.PRONOUNCE_CORRECT: [RobotBehaviors.ROBOT_EXPLANATION_SPEECH],
                RobotActionSequence.RESULTS_RETURNED:[],
                RobotActionSequence.WRONG_OBJECT_FAIL: [],
            },
            'virtual': RobotBehaviors.VIRTUALLY_CLICK_CORRECT_OBJ
            }
        })


    def competent_role(self):
        '''
        competent role for robot's turn 
        '''
        self.mapping.update({
            RobotRoles.COMPETENT:{
            'physical':{
                RobotActionSequence.TURN_FINISHED: [RobotBehaviors.ROBOT_TURN_SPEECH, RobotBehaviors.LOOK_AT_TABLET, RobotBehaviors.ROBOT_EXCITED],
                RobotActionSequence.TURN_STARTED: [RobotBehaviors.ROBOT_THINKING, RobotBehaviors.LOOK_AT_TABLET],
                RobotActionSequence.SCREEN_MOVED: [],
                RobotActionSequence.OBJECT_CLICKED: [RobotBehaviors.ROBOT_UNCERTAIN_SPEECH], 
                RobotActionSequence.OBJECT_FOUND: [],
                RobotActionSequence.OBJECT_PRONOUNCED: [RobotBehaviors.ROBOT_SAY_WORD], 
                RobotActionSequence.PRONOUNCE_CORRECT: [RobotBehaviors.ROBOT_HAPPY_DANCE, RobotBehaviors.ROBOT_HAPPY_SPEECH],
                RobotActionSequence.RESULTS_RETURNED:[],
                RobotActionSequence.WRONG_OBJECT_FAIL: [RobotBehaviors.ROBOT_SURPRISED_SPEECH, RobotBehaviors.ROBOT_DISAPPOINTED, RobotBehaviors.ROBOT_CONFIDENCE_SPEECH]
            },
            'virtual': RobotBehaviors.VIRTUALLY_CLICK_CORRECT_OBJ
            }
        })


    def novice_role(self):
        '''
        novice role for robot's turn
        '''
        self.mapping.update({
            RobotRoles.NOVICE:{
            'physical':{
                RobotActionSequence.TURN_FINISHED: [RobotBehaviors.ROBOT_TURN_SPEECH, RobotBehaviors.LOOK_AT_TABLET, RobotBehaviors.ROBOT_EXCITED],
                RobotActionSequence.TURN_STARTED: [RobotBehaviors.LOOK_LEFT_RIGHT],
                RobotActionSequence.SCREEN_MOVED: [],
                RobotActionSequence.OBJECT_CLICKED: [RobotBehaviors.ROBOT_UNSURE, RobotBehaviors.ROBOT_UNCERTAIN_SPEECH], 
                RobotActionSequence.OBJECT_FOUND: [],
                RobotActionSequence.OBJECT_PRONOUNCED: [RobotBehaviors.ROBOT_SAY_WORD], 
                RobotActionSequence.PRONOUNCE_CORRECT: [RobotBehaviors.ROBOT_EXPLANATION_SPEECH],
                RobotActionSequence.RESULTS_RETURNED:[],
                RobotActionSequence.WRONG_OBJECT_FAIL: [RobotBehaviors.ROBOT_SAD, RobotBehaviors.ROBOT_ASK_HELP, RobotBehaviors.ROBOT_CONFIDENCE_SPEECH]
            },
            'virtual': RobotBehaviors.VIRTUALLY_CLICK_CORRECT_OBJ
            }
        })

    def robot_response(self):
        '''
        novice robot's response to child during child's turn
        '''
        self.mapping.update({
            'Response':{
            'physical':{
                RobotActionSequence.TURN_FINISHED: [RobotBehaviors.ROBOT_CHILD_TURN_SPEECH, RobotBehaviors.ROBOT_ASK_HELP_SPEECH],
                RobotActionSequence.TURN_STARTED: [RobotBehaviors.LOOK_AT_TABLET],
                RobotActionSequence.SCREEN_MOVED: [],
                RobotActionSequence.OBJECT_CLICKED: [RobotBehaviors.LOOK_AT_TABLET], 
                RobotActionSequence.OBJECT_FOUND: [RobotBehaviors.LOOK_AT_TABLET],
                RobotActionSequence.OBJECT_PRONOUNCED: [RobotBehaviors.LOOK_AT_TABLET], 
                RobotActionSequence.PRONOUNCE_CORRECT: [RobotBehaviors.ROBOT_CELEBRATION, RobotBehaviors.ROBOT_HAPPY_DANCE],
                RobotActionSequence.RESULTS_RETURNED:[],
                RobotActionSequence.WRONG_OBJECT_FAIL: [RobotBehaviors.ROBOT_COMFORT_SPEECH]
            },
            'virtual': RobotBehaviors.VIRTUALLY_CLICK_CORRECT_OBJ
            }
        })



    # def complete_expert_role(self):
    #     '''
    #     expert role for robot's turn
    #     '''
    #     self.mapping.update({
    #         RobotRoles.COMPLETE_EXPERT:{
    #         'physical':{
    #             RobotActionSequence.TURN_STARTED: [ RobotBehaviors.LOOK_AT_TABLET, RobotBehaviors.ROBOT_TURN_SPEECH ],
    #             RobotActionSequence.SCREEN_MOVED: [],
    #             RobotActionSequence.OBJECT_FOUND: [],
    #             RobotActionSequence.OBJECT_CLICKED: [RobotBehaviors.EYE_FIDGET,RobotBehaviors.RING_ANSWER_CORRECT], 
    #             RobotActionSequence.OBJECT_PRONOUNCED: [RobotBehaviors.PRONOUNCE_CORRECT],
    #             RobotActionSequence.RESULTS_RETURNED: [RobotBehaviors.WIN_SPEECH],
    #             RobotActionSequence.TURN_FINISHED:[]
    #             }, 
    #         'virtual':
    #             RobotBehaviors.VIRTUALLY_CLICK_CORRECT_OBJ
                
    #         }
    #     })

    # def complete_novice_role(self):
    #     '''
    #     novice role for robot's turn
    #     '''
    #     self.mapping.update({
    #         RobotRoles.COMPLETE_NOVICE:{
    #         'physical':{
    #             RobotActionSequence.TURN_STARTED: [ RobotBehaviors.LOOK_AT_TABLET, RobotBehaviors.ROBOT_TURN_SPEECH ],
    #             RobotActionSequence.SCREEN_MOVED: [],
    #             RobotActionSequence.OBJECT_FOUND: [],
    #             RobotActionSequence.OBJECT_CLICKED: [RobotBehaviors.EYE_FIDGET,RobotBehaviors.RING_ANSWER_CORRECT], 
    #             RobotActionSequence.OBJECT_PRONOUNCED: [RobotBehaviors.PRONOUNCE_CORRECT],
    #             RobotActionSequence.RESULTS_RETURNED: [RobotBehaviors.WIN_SPEECH],
    #             RobotActionSequence.TURN_FINISHED:[]
    #             }, 
    #         'virtual':
    #             RobotBehaviors.VIRTUALLY_CLICK_WRONG_OBJ
                
    #         }
    #     })

    """
    
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

    """


