"""
This is a class defines different "cosmetic" (i.e. not necessarily in the Agent ActionSpace)
Robot Behaviors
"""
# -*- coding: utf-8 -*-
# pylint: disable=import-error, invalid-name
from unity_game_msgs.msg import iSpyCommand
from enum import Enum
import json



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

    TURN_STARTED = "TURN_STARTED"
    SCREEN_MOVED = "SCREEN_MOVED"
    OBJECT_FOUND = "OBJECT_FOUND"
    OBJECT_CLICKED = "OBJECT_CLICKED" #
    OBJECT_PRONOUNCED = "OBJECT_PRONOUNCED" #
    RESULTS_RETURNED = "RESULTS_RETURNED" 
    TURN_FINISHED = "TURN_FINISHED" #
    PRONOUNCE_CORRECT = "PRONOUNCE_CORRECT"
    WRONG_OBJECT_FAIL ="WRONG_OBJECT_FAIL" 

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


        robot_actions_file = open("iSpyGameController/res/robot_actions.json")
        self.robot_actions_dict = json.loads(robot_actions_file.read())

        #test = getattr(RobotBehaviors,pre)
      
    def get_action_name(self,action):
        '''
        return the correct action name (convert the action name in json file to the action name in RobotBehaviorList)
        '''
        try: 
            return getattr(RobotBehaviors,action)
        except:
            return action

    def get_robot_general_responses(self):
        pass   
        
    def get_actions(self,role,robot_turn,physical_virtual):
    
        try:
            role = role.name
        except:
            role = role
        print("role...")
        print(type(role))
        print(role)
        return self.robot_actions_dict[role][robot_turn][physical_virtual]
        
  


