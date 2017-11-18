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

    TURN_STARTED = "turnStarted" #
    SCREEN_MOVED = "screenMoved"
    OBJECT_FOUND = "objectFound"
    OBJECT_CLICKED = "objectClicked" #
    OBJECT_PRONOUNCED = "objectPronounced" #
    RESULTS_RETURNED = "ResultsReturned" 
    TURN_FINISHED = "TurnFinished" #
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
        # robot's actions during its turn
        self.robot_turn_mapping = {}
        # robot's actions during child's turn
        self.child_turn_mapping = {}

        self._expert_role()
        self._competent_role()
        self._novice_role()
        self._robot_general_responses()


        self.backup_behaviors =  {'physical':{
                RobotActionSequence.TURN_FINISHED: [RobotBehaviors.LOOK_AT_TABLET],
                RobotActionSequence.TURN_STARTED: [RobotBehaviors.ROBOT_EXCITED],
                RobotActionSequence.SCREEN_MOVED: [], 
                RobotActionSequence.OBJECT_CLICKED: [RobotBehaviors.ROBOT_UNSURE], 
                RobotActionSequence.OBJECT_FOUND: [RobotBehaviors.ROBOT_YES], # Nothing
                RobotActionSequence.OBJECT_PRONOUNCED: [RobotBehaviors.ROBOT_ATTENTION], 
                RobotActionSequence.PRONOUNCE_CORRECT: [RobotBehaviors.ROBOT_CELEBRATION],
                RobotActionSequence.RESULTS_RETURNED:[], # Nothing
                RobotActionSequence.WRONG_OBJECT_FAIL: [RobotBehaviors.ROBOT_SAD],
            },
            'virtual': ""
            }


        """
        self.complete_expert_role()
        self.complete_novice_role()
        """
        
    def get_actions(self,role,robot_turn):
        print("role is: ")
        print(role)
        if robot_turn == True:
            print("robot turn mapping")
            # if the role is an instance of robotRoles
            # return a sequence of specific actions that the robot needs to execute for a given role
            return self.robot_turn_mapping[role] if role in self.robot_turn_mapping.keys() else self.backup_behaviors
        else:
            print("child turn mapping")
            return self.child_turn_mapping[role] if role in self.child_turn_mapping.keys() else self.backup_behaviors


    def _expert_role(self):
        '''
        expert role for robot's turn
        '''
        self.robot_turn_mapping.update({
            RobotRoles.EXPERT:{
            'physical':{
                RobotActionSequence.TURN_FINISHED: [RobotBehaviors.ROBOT_ENCOURAGING, RobotBehaviors.ROBOT_ENCOURAGING_SPEECH],
                RobotActionSequence.TURN_STARTED: [RobotBehaviors.ROBOT_INTERESTED, RobotBehaviors.LOOK_AT_TABLET],
                RobotActionSequence.SCREEN_MOVED: [],
                RobotActionSequence.OBJECT_CLICKED: [RobotBehaviors.ROBOT_CURIOUS], # Always correct
                RobotActionSequence.OBJECT_FOUND: [RobotBehaviors.ROBOT_CURIOUS], # Always correct
                RobotActionSequence.OBJECT_PRONOUNCED: [RobotBehaviors.ROBOT_SAY_WORD], 
                RobotActionSequence.PRONOUNCE_CORRECT: [RobotBehaviors.ROBOT_CELEBRATION],
                RobotActionSequence.RESULTS_RETURNED:[], # Nothing
                RobotActionSequence.WRONG_OBJECT_FAIL: [], # Nothing
            },
            'virtual': RobotBehaviors.VIRTUALLY_CLICK_CORRECT_OBJ
            }
        })

        self.child_turn_mapping.update({
            RobotRoles.EXPERT:{
            'physical':{
                RobotActionSequence.TURN_FINISHED: [RobotBehaviors.ROBOT_EXCITED],
                RobotActionSequence.TURN_STARTED: [RobotBehaviors.ROBOT_ATTENTION],
                RobotActionSequence.SCREEN_MOVED: [],
                RobotActionSequence.OBJECT_CLICKED: [RobotBehaviors.ROBOT_UNSURE], # Incorrect object
                RobotActionSequence.OBJECT_FOUND: [RobotBehaviors.ROBOT_YES], # Correct Object
                RobotActionSequence.OBJECT_PRONOUNCED: [RobotBehaviors.ROBOT_ATTENTION], 
                RobotActionSequence.PRONOUNCE_CORRECT: [RobotBehaviors.ROBOT_HAPPY_DANCE],
                RobotActionSequence.RESULTS_RETURNED:[], # Nothing
                RobotActionSequence.WRONG_OBJECT_FAIL: [RobotBehaviors.ROBOT_SAD, RobotBehaviors.ROBOT_COMFORT],
            },
            'virtual': ""
            }
        })



    def _competent_role(self):
        '''
        competent role for robot's turn 
        '''
        self.robot_turn_mapping.update({
            RobotRoles.COMPETENT:{
            'physical':{
                RobotActionSequence.TURN_FINISHED: [RobotBehaviors.ROBOT_CURIOUS],
                RobotActionSequence.TURN_STARTED: [RobotBehaviors.ROBOT_EXCITED, RobotBehaviors.LOOK_AT_TABLET],
                RobotActionSequence.SCREEN_MOVED: [],
                RobotActionSequence.OBJECT_CLICKED: [RobotBehaviors.ROBOT_UNSURE], # Incorrect
                RobotActionSequence.OBJECT_FOUND: [RobotBehaviors.ROBOT_THINKING], # Correct object 
                RobotActionSequence.OBJECT_PRONOUNCED: [RobotBehaviors.ROBOT_SAY_WORD], 
                RobotActionSequence.PRONOUNCE_CORRECT: [RobotBehaviors.ROBOT_HAPPY_DANCE],
                RobotActionSequence.RESULTS_RETURNED:[], # Nothing
                RobotActionSequence.WRONG_OBJECT_FAIL: [RobotBehaviors.ROBOT_SAD, RobotBehaviors.ROBOT_CONFIDENCE_SPEECH]
            },
            'virtual': RobotBehaviors.VIRTUALLY_CLICK_CORRECT_OBJ
            }
        })

        self.child_turn_mapping.update({
            RobotRoles.COMPETENT:{
            'physical':{
                RobotActionSequence.TURN_FINISHED: [RobotBehaviors.ROBOT_ENCOURAGING],
                RobotActionSequence.TURN_STARTED: [RobotBehaviors.LOOK_CENTER, RobotBehaviors.ROBOT_INTERESTED],
                RobotActionSequence.SCREEN_MOVED: [],
                RobotActionSequence.OBJECT_CLICKED: [RobotBehaviors.ROBOT_UNCERTAIN_SPEECH], # Incorrect object
                RobotActionSequence.OBJECT_FOUND: [RobotBehaviors.ROBOT_THINKING], # Correct object
                RobotActionSequence.OBJECT_PRONOUNCED: [RobotBehaviors.ROBOT_ATTENTION], 
                RobotActionSequence.PRONOUNCE_CORRECT: [RobotBehaviors.ROBOT_CELEBRATION],
                RobotActionSequence.RESULTS_RETURNED:[], # Nothing
                RobotActionSequence.WRONG_OBJECT_FAIL: [RobotBehaviors.ROBOT_COMFORT],
            },
            'virtual': ""
            }
        })


    def _novice_role(self):
        '''
        novice role for robot's turn
        '''
        self.robot_turn_mapping.update({
            RobotRoles.NOVICE:{
            'physical':{
                RobotActionSequence.TURN_FINISHED: [RobotBehaviors.ROBOT_CURIOUS],
                RobotActionSequence.TURN_STARTED: [RobotBehaviors.ROBOT_EXCITED, RobotBehaviors.LOOK_AT_TABLET],
                RobotActionSequence.SCREEN_MOVED: [],
                RobotActionSequence.OBJECT_CLICKED: [RobotBehaviors.ROBOT_UNSURE], # Incorrect Object
                RobotActionSequence.OBJECT_FOUND: [RobotBehaviors.ROBOT_UNSURE],  # Correct Object
                RobotActionSequence.OBJECT_PRONOUNCED: [RobotBehaviors.ROBOT_SAY_WORD], 
                RobotActionSequence.PRONOUNCE_CORRECT: [RobotBehaviors.ROBOT_HAPPY_DANCE],
                RobotActionSequence.RESULTS_RETURNED:[], # Nothing
                RobotActionSequence.WRONG_OBJECT_FAIL: [RobotBehaviors.ROBOT_ASK_HELP]
            },
            'virtual': RobotBehaviors.VIRTUALLY_CLICK_CORRECT_OBJ
            }
        })

        self.child_turn_mapping.update({
            RobotRoles.NOVICE:{
            'physical':{
                RobotActionSequence.TURN_FINISHED: [RobotBehaviors.ROBOT_WINK],
                RobotActionSequence.TURN_STARTED: [RobotBehaviors.LOOK_CENTER, RobotBehaviors.ROBOT_INTERESTED],
                RobotActionSequence.SCREEN_MOVED: [],
                RobotActionSequence.OBJECT_CLICKED: [RobotBehaviors.LOOK_AT_TABLET], # Incorrect Object
                RobotActionSequence.OBJECT_FOUND: [RobotBehaviors.LOOK_AT_TABLET], # Correct Object
                RobotActionSequence.OBJECT_PRONOUNCED: [RobotBehaviors.ROBOT_ATTENTION], 
                RobotActionSequence.PRONOUNCE_CORRECT: [RobotBehaviors.ROBOT_HAPPY_DANCE, RobotBehaviors.ROBOT_THANKYOU_SPEECH],
                RobotActionSequence.RESULTS_RETURNED:[], # Nothing
                RobotActionSequence.WRONG_OBJECT_FAIL: [RobotBehaviors.ROBOT_COMFORT],
            },
            'virtual': ""
            }
        })

    def _robot_general_responses(self):
        '''
        novice robot's response to child during child's turn
        '''
        self.general_responses = {
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
        
    def get_robot_general_responses(self):
        return self.general_responses


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


