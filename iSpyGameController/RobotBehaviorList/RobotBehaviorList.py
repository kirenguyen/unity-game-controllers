"""
This is a class defines different "cosmetic" (i.e. not necessarily in the Agent ActionSpace)
Robot Behaviors
"""
# -*- coding: utf-8 -*-
# pylint: disable=import-error, invalid-name
from unity_game_msgs.msg import iSpyCommand
from enum import Enum
import json
import random



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

    # Silent Emotions
    ROBOT_SILENT_NOD = 'ROBOT_SILENT_NOD'
    ROBOT_SILENT_HAPPY_DANCE = 'ROBOT_SILENT_HAPPY_DANCE'
    ROBOT_SILENT_YES = 'ROBOT_SILENT_YES'
    ROBOT_SILENT_PUZZLED = 'ROBOT_SILENT_PUZZLED'
    ROBOT_SILENT_FRUSTRATED = 'ROBOT_SILENT_FRUSTRATED'
    ROBOT_SILENT_SAD = 'ROBOT_SILENT_SAD'
    ROBOT_SILENT_INTERESTED = 'ROBOT_SILENT_INTERESTED'
    ROBOT_SILENT_EXCITED = 'ROBOT_SILENT_EXCITED'
 

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


    ROBOT_HINT_BUTTON_REMINDER = "ROBOT_HINT_BUTTON_REMINDER"
    ROBOT_CUSTOM_SPEECH = "ROBOT_CUSTOM_SPEECH"
    Q_ROBOT_INDUCE_SPEECH = "Q_ROBOT_INDUCE_SPEECH"
    ROBOT_INDUCE_SPEECH_RESPONSE = "ROBOT_INDUCE_SPEECH_RESPONSE"
    ROBOT_TASK_SPEECH_RESPONSE = "ROBOT_TASK_SPEECH_RESPONSE"

    ### ============== Tega Speech for Role Switching Project ================== ###
    BEFORE_GAME_SPEECH = "ROBOT_BEFORE_GAME_SPEECH"
    VOCAB_EXPLANATION_SPEECH = "VOCAB_EXPLANATION_SPEECH"
    HINT_SPEECH = "HINT_SPEECH"
    KEYWORD_DEFINITION_SPEECH = "KEYWORD_DEFINITION_SPEECH"
    REMINDER_SPEECH = "REMINDER_SPEECH"

    NOVICE_ROLE_KEYWORD = "NOVICE_ROLE_KEYWORD" 


   ### ====== Tega Question Asking =================== ####
    Q_ROBOT_OFFER_HELP = "Q_ROBOT_OFFER_HELP"
    Q_ROBOT_ASK_WHY_CHOOSE_IT="Q_ROBOT_ASK_WHY_CHOOSE_IT"
    Q_ROBOT_WANT_LEARN="Q_ROBOT_WANT_LEARN"
    Q_ROBOT_ASK_HELP="Q_ROBOT_ASK_HELP"
    Q_ROBOT_ASK_WHY_WRONG="Q_ROBOT_ASK_WHY_WRONG"
    Q_END_OF_TURN="Q_END_OF_TURN"

    ### ===== no ispy action alert === ###
    NO_ISPY_ACTION_ALERT = "NO_ISPY_ACTION_ALERT"
    ROBOT_TASK_END_BEHAVIOR = "ROBOT_TASK_END_BEHAVIOR" # Deprecated; not for Jibo

    ### ===== task end behaviors === ###
    ROBOT_PLAY_MUSIC = "ROBOT_PLAY_MUSIC"
    ROBOT_DANCE = "ROBOT_DANCE"
    ROBOT_TASK_END_RESPONSE = "ROBOT_TASK_END_RESPONSE"

    Q_ROBOT_TASK_END_REMINDER = "Q_ROBOT_TASK_END_REMINDER"
    Q_ROBOT_TASK_END_ASSESSMENT = "Q_ROBOT_TASK_END_ASSESSMENT"


class RobotRoles(Enum):
    '''
    contains a list of social roles that are avaiable to robot to perform
    '''
    EXPERT = 0
    NOVICE = 1


class RobotActionSequence:

    TURN_STARTING = "TURN_STARTING"
    SCREEN_MOVING = "SCREEN_MOVING"
    RIGHT_OBJECT_FOUND = "RIGHT_OBJECT_FOUND"
    WRONG_OBJECT_CLICKED = "WRONG_OBJECT_CLICKED" #
    OBJECT_PRONOUNCED = "OBJECT_PRONOUNCED" #
    RESULTS_RETURNED = "RESULTS_RETURNED" 
    TURN_SWITCHING = "TURN_SWITCHING" #
    PRONOUNCE_CORRECT = "PRONOUNCE_CORRECT"
    WRONG_OBJECT_FAIL ="WRONG_OBJECT_FAIL" 

    class Triggers:
        NEXT = "Next"
        RESET = "Reset"


            
class RobotRolesBehaviorsMap:
    '''
    mapping between robot's social role and robot's specific behaviors
    '''  
    def __init__(self,game_round):
        # robot's actions during its turn
        self.robot_turn_mapping = {}
        # robot's actions during child's turn
        self.child_turn_mapping = {}

        # action_file = "iSpyGameController/res/robot_actions_practice_round.json" if game_round == "practice" else "iSpyGameController/res/robot_actions.json"
        action_file = "iSpyGameController/res/robot_actions-test-tran.json"
        # action_file = "iSpyGameController/res/robot_actions.json"
        

        robot_actions_file = open(action_file)
        self.robot_actions_dict = json.loads(robot_actions_file.read())

        question_answer_file = open("iSpyGameController/res/question_answer.json")
        self.question_answer_dict = json.loads(question_answer_file.read())

        self.current_action_name = ""


      
    def get_action_name(self,action):
        '''
        return the correct action name (convert the action name in json file to the action name in RobotBehaviorList)
        '''
        try: 
            self.current_action_name = getattr(RobotBehaviors,action)
            return self.current_action_name
        except:
            self.current_action_name = action
            return self.current_action_name

    def get_robot_general_responses(self):
        pass   

    def get_robot_question(self,question_query_path):
        '''
        get question query result 
        '''
        self.current_question_query_path = question_query_path
        if question_query_path in self.question_answer_dict.keys():
            self.question_query = self.question_answer_dict[question_query_path]
            question_name = random.choice(self.question_query['question'])

            return question_name
        else:
            print("ERROR: Cannot find the question query.\n")
            return ""


    def get_robot_response_to_help(self,child_help_response):
        '''
        return bool for child answer to helping the robot
        '''
        print("RESPONSE_TO_HELP: "+child_help_response)
        if self.current_question_query_path in self.question_answer_dict.keys():
            if "no_response_" in child_help_response: # no response from child
                return False
            else:
                yes_response = self.question_query["user_input"][0]
                if any(m in child_help_response for m in yes_response["en_US"]): # found child's answer
                    return True
                return False

    def get_child_answer_type(self,asr_input):
        '''
        return bool for child answer to helping the robot
        '''
        
        if self.current_question_query_path in self.question_answer_dict.keys():
            if "no_response_" in asr_input: # no response from child
                return "absence"
            else:
                yes_response = self.question_query["user_input"][0]
                if any(m in asr_input for m in yes_response["en_US"]): # found child's answer
                    return "positive"
                no_response = self.question_query["user_input"][1]
                if any(m in asr_input for m in no_response["en_US"]): # found child's answer
                    return "negative"
                return "others"


    def get_question_type(self):
        if self.current_question_query_path in self.question_answer_dict.keys():
            return self.question_query['other']['type']
        else:
            return ["",False]

    def get_robot_response_to_answer(self,child_answer):
        '''
        get robot's contigent response to the child's answer
        '''

        print("CHILD ANSWER: "+child_answer)
        if self.current_question_query_path in self.question_answer_dict.keys():
            if "no_response_" in child_answer: # no response from child
                return random.choice(self.question_query[child_answer])
            else:
                for i in self.question_query["user_input"]:
                    if any(m in child_answer for m in i["en_US"]): # found child's answer
                        return random.choice(i["response"])
                        break
                print("INFO: Cannot find child's answer. Return other response\n")
                return random.choice(self.question_query["other_input"])
        else:
            print("ERROR: Cannot find the question query\n")
            return ""

        
    def get_actions(self,role,robot_turn,physical_virtual):
    
        try:
            role = role.name
        except:
            role = "BACKUP"
        return self.robot_actions_dict[role][robot_turn][physical_virtual]
        
  


