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

    ROBOT_SAY_WORD = 'ROBOT_SAY_WORD'
    
    # Negative Emotions
    ROBOT_SAD = 'ROBOT_SAD'
    ROBOT_UNSURE = 'ROBOT_UNSURE'
    ROBOT_COMFORT = 'ROBOT_COMFORT'
    ROBOT_ASK_HELP = 'ROBOT_ASK_HELP'
    ROBOT_DISAPPOINTED = 'ROBOT_DISAPPOINTED'

 

    # virtual actions on the app
    VIRTUALLY_CLICK_CORRECT_OBJ = "CLICK_CORRECT_OBJ" # click correct obj
    VIRTUALLY_CLICK_WRONG_OBJ = "CLICK_WRONG_OBJ"
    VIRTUALLY_EXPLORE = "EXPLORING"
    VIRTUALLY_CLICK_SAY_BUTTON = "CLICK_SAY_BUTTON"
    VIRTUALLY_HELP_CHILD = "HELP_CHILD"

    ## Tega Speech for Curiosity Assessment
    GENERAL_CURIOSITY_SPEECH = "GENERAL_CURIOSITY_SPEECH"
    BASED_ON_PROMPTS_SPEECH = "BASED_ON_PROMPTS_SPEECH"
    TRY_PRONOUNCE = "TRY_PRONOUNCE"
    BASED_ON_OBJECTS = "BASED_ON_OBJECTS"
    OBJECTS = "OBJECTS"


    ROBOT_CUSTOM_SPEECH = "ROBOT_CUSTOM_SPEECH"


    ROBOT_CHILD_TURN_SPEECH =""
    ROBOT_ASK_HELP_SPEECH = ""
    ROBOT_COMFORT_SPEECH =""

    
     ### robot's actions that can be optional, and are not necessary to play every single time
    OPTIONAL_ACTIONS = [GENERAL_CURIOSITY_SPEECH,BASED_ON_PROMPTS_SPEECH]

class RobotRoles(Enum):
    '''
    contains a list of social roles that are avaiable to robot to perform
    '''
    EXPERT = 0
    #COMPETENT = 1
    NOVICE = 1


class RobotActionSequence:

    TURN_STARTED = "turn_starting" #
    SCREEN_MOVED = "screen_moving"
    OBJECT_FOUND = "object_found"
    OBJECT_CLICKED = "object_clicked" #
    OBJECT_PRONOUNCED = "object_pronounced" #
    RESULTS_RETURNED = "results_returned" 
    TURN_FINISHED = "turn_switching" #
    PRONOUNCE_CORRECT = "pronounce_correct"
    WRONG_OBJECT_FAIL ="wrong_object_fail" 

    class Triggers:
        NEXT = "Next"
        RESET = "Reset"

            
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
        #self._curious_role()
        #self._competent_role()
        self._novice_role()
        self._robot_general_responses()




        self.backup_behaviors =  {'physical':{
                RobotActionSequence.TURN_FINISHED: [RobotBehaviors.GENERAL_CURIOSITY_SPEECH],
                RobotActionSequence.TURN_STARTED: [RobotBehaviors.ROBOT_EXCITED],
                RobotActionSequence.SCREEN_MOVED: [RobotBehaviors.ROBOT_COMFORT, RobotBehaviors.LOOK_CENTER], 
                RobotActionSequence.OBJECT_CLICKED: [RobotBehaviors.ROBOT_UNSURE], 
                RobotActionSequence.OBJECT_FOUND: [RobotBehaviors.ROBOT_YES], 
                RobotActionSequence.OBJECT_PRONOUNCED: [RobotBehaviors.LOOK_CENTER, RobotBehaviors.ROBOT_ATTENTION], 
                RobotActionSequence.PRONOUNCE_CORRECT: [RobotBehaviors.ROBOT_CELEBRATION],
                RobotActionSequence.RESULTS_RETURNED:[], # Nothing
                RobotActionSequence.WRONG_OBJECT_FAIL: [RobotBehaviors.ROBOT_SAD],
            },
            'virtual': ""
            }


        
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

    def _get_virtual_behaviors(self):
        # get virtual behaviors for different robot roles
        virtual_behaviors = RobotBehaviors.VIRTUALLY_CLICK_CORRECT_OBJ
        return virtual_behaviors

    def _expert_role(self):
        '''
        expert role for robot's turn
        '''
        self.robot_turn_mapping.update({
            RobotRoles.EXPERT:{
            'physical':{
                RobotActionSequence.TURN_FINISHED: [RobotBehaviors.ROBOT_ENCOURAGING],
                RobotActionSequence.TURN_STARTED: [RobotBehaviors.ROBOT_INTERESTED, RobotBehaviors.LOOK_AT_TABLET],
                RobotActionSequence.SCREEN_MOVED: [], # Nothing
                RobotActionSequence.OBJECT_CLICKED: [RobotBehaviors.ROBOT_CURIOUS], # Always correct
                RobotActionSequence.OBJECT_FOUND: [RobotBehaviors.ROBOT_CURIOUS], # Always correct
                RobotActionSequence.OBJECT_PRONOUNCED: [RobotBehaviors.LOOK_CENTER, RobotBehaviors.ROBOT_SAY_WORD], 
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
                RobotActionSequence.SCREEN_MOVED: [RobotBehaviors.ROBOT_COMFORT, RobotBehaviors.LOOK_CENTER],
                RobotActionSequence.OBJECT_CLICKED: [RobotBehaviors.ROBOT_UNSURE], # Incorrect object
                RobotActionSequence.OBJECT_FOUND: [RobotBehaviors.ROBOT_YES], # Correct Object
                RobotActionSequence.OBJECT_PRONOUNCED: [RobotBehaviors.LOOK_CENTER, RobotBehaviors.ROBOT_ATTENTION], 
                RobotActionSequence.PRONOUNCE_CORRECT: [RobotBehaviors.ROBOT_HAPPY_DANCE],
                RobotActionSequence.RESULTS_RETURNED:[], # Nothing
                RobotActionSequence.WRONG_OBJECT_FAIL: [RobotBehaviors.ROBOT_SAD],
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
                RobotActionSequence.TURN_FINISHED: [RobotBehaviors.LOOK_CENTER, RobotBehaviors.ROBOT_CURIOUS],
                RobotActionSequence.TURN_STARTED: [RobotBehaviors.ROBOT_EXCITED, RobotBehaviors.LOOK_AT_TABLET],
                RobotActionSequence.SCREEN_MOVED: [], # Nothing
                RobotActionSequence.OBJECT_CLICKED: [RobotBehaviors.ROBOT_THINKING], # Incorrect
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
                RobotActionSequence.TURN_STARTED: [RobotBehaviors.LOOK_CENTER, RobotBehaviors.ROBOT_EXCITED],
                RobotActionSequence.SCREEN_MOVED: [RobotBehaviors.ROBOT_COMFORT, RobotBehaviors.LOOK_CENTER],
                RobotActionSequence.OBJECT_CLICKED: [RobotBehaviors.ROBOT_UNCERTAIN_SPEECH], # Incorrect object
                RobotActionSequence.OBJECT_FOUND: [RobotBehaviors.ROBOT_THINKING], # Correct object
                RobotActionSequence.OBJECT_PRONOUNCED: [RobotBehaviors.LOOK_CENTER, RobotBehaviors.ROBOT_ATTENTION], 
                RobotActionSequence.PRONOUNCE_CORRECT: [RobotBehaviors.ROBOT_CELEBRATION],
                RobotActionSequence.RESULTS_RETURNED:[], # Nothing
                RobotActionSequence.WRONG_OBJECT_FAIL: [RobotBehaviors.ROBOT_SAD],
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
                RobotActionSequence.TURN_FINISHED: [RobotBehaviors.LOOK_CENTER, RobotBehaviors.ROBOT_CURIOUS],
                RobotActionSequence.TURN_STARTED: [RobotBehaviors.ROBOT_EXCITED, RobotBehaviors.LOOK_AT_TABLET],
                RobotActionSequence.SCREEN_MOVED: [], # Nothing
                RobotActionSequence.OBJECT_CLICKED: [RobotBehaviors.ROBOT_UNSURE], # Incorrect Object
                RobotActionSequence.OBJECT_FOUND: [RobotBehaviors.ROBOT_UNSURE],  # Correct Object
                RobotActionSequence.OBJECT_PRONOUNCED: [RobotBehaviors.ROBOT_SAY_WORD], 
                RobotActionSequence.PRONOUNCE_CORRECT: [RobotBehaviors.ROBOT_HAPPY_DANCE],
                RobotActionSequence.RESULTS_RETURNED:[], # Nothing
                RobotActionSequence.WRONG_OBJECT_FAIL: [RobotBehaviors.ROBOT_ASK_HELP]
            },
            'virtual': RobotBehaviors.VIRTUALLY_CLICK_WRONG_OBJ
            }
        })

        self.child_turn_mapping.update({
            RobotRoles.NOVICE:{
            'physical':{
                RobotActionSequence.TURN_FINISHED: [RobotBehaviors.ROBOT_WINK],
                RobotActionSequence.TURN_STARTED: [RobotBehaviors.LOOK_CENTER, RobotBehaviors.ROBOT_INTERESTED],
                RobotActionSequence.SCREEN_MOVED: [RobotBehaviors.ROBOT_COMFORT, RobotBehaviors.LOOK_CENTER],
                RobotActionSequence.OBJECT_CLICKED: [RobotBehaviors.LOOK_AT_TABLET], # Incorrect Object
                RobotActionSequence.OBJECT_FOUND: [RobotBehaviors.LOOK_AT_TABLET, RobotBehaviors.LOOK_CENTER], # Correct Object
                RobotActionSequence.OBJECT_PRONOUNCED: [RobotBehaviors.LOOK_CENTER, RobotBehaviors.ROBOT_ATTENTION], 
                RobotActionSequence.PRONOUNCE_CORRECT: [RobotBehaviors.ROBOT_HAPPY_DANCE],
                RobotActionSequence.RESULTS_RETURNED:[], # Nothing
                RobotActionSequence.WRONG_OBJECT_FAIL: [RobotBehaviors.ROBOT_SAD],
            },
            'virtual': ""
            }
        })

    def _curious_role(self):
        '''
        expert role for robot's turn
        '''
        self.robot_turn_mapping.update({
            RobotRoles.CURIOUS:{
            'physical':{
                RobotActionSequence.TURN_FINISHED: [RobotBehaviors.ROBOT_EXCITED],
                RobotActionSequence.TURN_STARTED: [ RobotBehaviors.LOOK_AT_TABLET],
                RobotActionSequence.SCREEN_MOVED: [ RobotBehaviors.ROBOT_CURIOUS, RobotBehaviors.GENERAL_CURIOSITY_SPEECH], # not working
                RobotActionSequence.OBJECT_CLICKED: [RobotBehaviors.ROBOT_CURIOUS, RobotBehaviors.TRY_PRONOUNCE], # Always correct
                RobotActionSequence.OBJECT_FOUND: [RobotBehaviors.ROBOT_CURIOUS], # Always correct
                RobotActionSequence.OBJECT_PRONOUNCED: [RobotBehaviors.LOOK_CENTER, RobotBehaviors.ROBOT_SAY_WORD], 
                RobotActionSequence.PRONOUNCE_CORRECT: [RobotBehaviors.ROBOT_CELEBRATION],
                RobotActionSequence.RESULTS_RETURNED:[], # Nothing
                RobotActionSequence.WRONG_OBJECT_FAIL: [], # Nothing
            },
            'virtual': RobotBehaviors.VIRTUALLY_CLICK_CORRECT_OBJ
            }
        })


        self.child_turn_mapping.update({
            RobotRoles.CURIOUS:{
            'physical':{
                RobotActionSequence.TURN_FINISHED: [RobotBehaviors.ROBOT_WINK],
                RobotActionSequence.TURN_STARTED: [RobotBehaviors.LOOK_CENTER, RobotBehaviors.ROBOT_INTERESTED],
                RobotActionSequence.SCREEN_MOVED: [RobotBehaviors.ROBOT_CURIOUS, RobotBehaviors.BASED_ON_PROMPTS_SPEECH],
                RobotActionSequence.OBJECT_CLICKED: [RobotBehaviors.ROBOT_CURIOUS], # Incorrect object
                RobotActionSequence.OBJECT_FOUND: [RobotBehaviors.ROBOT_YES], # Correct Object
                RobotActionSequence.OBJECT_PRONOUNCED: [RobotBehaviors.LOOK_CENTER, RobotBehaviors.ROBOT_ATTENTION], 
                RobotActionSequence.PRONOUNCE_CORRECT: [RobotBehaviors.ROBOT_HAPPY_DANCE],
                RobotActionSequence.RESULTS_RETURNED:[], # Nothing
                RobotActionSequence.WRONG_OBJECT_FAIL: [RobotBehaviors.ROBOT_SAD],
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




