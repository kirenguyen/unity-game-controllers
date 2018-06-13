"""
This is a class defines different "cosmetic" (i.e. not necessarily in the Agent ActionSpace)
Robot Behaviors
"""
# -*- coding: utf-8 -*-
# pylint: disable=import-error, invalid-name

from .RobotBehaviorList import RobotBehaviors
from GameUtils import GlobalSettings

if GlobalSettings.USE_ROS:
    import rospy
    import json
    import random
    from std_msgs.msg import Header  # standard ROS msg header
    from random import randint
    from unity_game_msgs.msg import TapGameCommand
    from unity_game_msgs.msg import TapGameLog
    from r1d1_msgs.msg import TegaAction
    from r1d1_msgs.msg import Vec3
    from jibo_msgs.msg import JiboAction

else:
    TapGameLog = GlobalSettings.TapGameLog  # Mock object, used for testing in non-ROS environments
    TapGameCommand = GlobalSettings.TapGameCommand
    JiboAction = GlobalSettings.JiboAction

jibo_tts_data = "iSpyGameController/res/jibo_speech.json"
jibo_tts_file = open(jibo_tts_data)
jibo_tts_dict = json.loads(jibo_tts_file.read())


class JiboBehaviors:  # pylint: disable=no-member, too-many-instance-attributes
    """
    A Class definition for "cosmetic" robot behavior strings, which get translated by the ROSNodeMgr
    """

    @staticmethod
    def get_msg_from_behavior(command, *args):  # pylint: disable=too-many-branches, too-many-statements

        msg = JiboAction()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()

        # Look Ats
        if command == RobotBehaviors.LOOK_AT_TABLET:
            msg.do_motion = True
            msg.do_tts = False
            msg.do_lookat = False
            msg.do_sound_playback = False
            msg.motion = JiboAction.LOOK_DOWN

        elif command == RobotBehaviors.LOOK_CENTER:
            msg.do_motion = True
            msg.do_tts = False
            msg.do_lookat = False
            msg.do_sound_playback = False
            msg.motion = JiboAction.DEFAULT

        elif command == RobotBehaviors.LOOK_LEFT_RIGHT:
            msg.do_motion = True
            msg.do_tts = False
            msg.do_lookat = False
            msg.do_sound_playback = False
            msg.motion = JiboAction.LOOK_LEFT_RIGHT

        elif command == RobotBehaviors.LOOK_DOWN_CENTER:
            msg.do_motion = True
            msg.do_tts = False
            msg.do_lookat = False
            msg.do_sound_playback = False
            msg.motion = JiboAction.LOOK_DOWN_CENTER

        # Positive Emotions
        elif command == RobotBehaviors.ROBOT_EXCITED:
            msg.do_motion = True
            msg.do_tts = False
            msg.do_lookat = False
            msg.do_sound_playback = False
            msg.motion = JiboAction.ROBOT_EXCITED

        elif command == RobotBehaviors.ROBOT_INTERESTED:
            msg.do_motion = True
            msg.do_tts = True
            msg.do_lookat = False
            msg.do_sound_playback = False
            msg.do_motion = JiboAction.ROBOT_INTERESTED

        elif command == RobotBehaviors.ROBOT_YES:
            msg.do_motion = True
            msg.do_tts = True
            msg.do_lookat = False
            msg.do_sound_playback = False
            msg.tts_text = "Yes"
            msg.motion = JiboAction.ROBOT_YES

        elif command == RobotBehaviors.ROBOT_HAPPY_DANCE:   #TODO: check if happy dance has audio
            msg.do_motion = True
            msg.do_tts = False
            msg.do_lookat = False
            msg.do_sound_playback = False
            msg.motion = JiboAction.ROBOT_HAPPY_DANCE

        elif command == RobotBehaviors.ROBOT_CURIOUS:
            msg.do_motion = True
            msg.do_tts = False
            msg.do_lookat = False
            msg.do_sound_playback = False
            msg.motion = JiboAction.ROBOT_CURIOUS

        elif command == RobotBehaviors.ROBOT_ATTENTION:
            msg.do_motion = True
            msg.do_tts = False
            msg.do_lookat = False
            msg.do_sound_playback = False
            msg.motion = JiboAction.ROBOT_ATTENTION

        elif command == RobotBehaviors.ROBOT_CELEBRATION:
            msg.do_motion = True
            msg.do_tts = False
            msg.do_sound_playback = False
            msg.motion = JiboAction.ROBOT_CELEBRATION

        elif command == RobotBehaviors.ROBOT_ENCOURAGING:
            msg.do_motion = True
            msg.do_tts = False
            msg.do_sound_playback = False
            msg.motion = JiboAction.ROBOT_ENCOURAGING

        elif command == RobotBehaviors.ROBOT_WINK:
            msg.do_motion = True
            msg.do_tts = False
            msg.do_sound_playback = False
            msg.motion = JiboAction.ROBOT_WINK

        elif command == RobotBehaviors.ROBOT_THINKING:
            msg.do_motion = True
            msg.do_tts = True
            msg.do_sound_playback = False
            msg.motion = JiboAction.ROBOT_THINKING


        # Negative Emotions
        elif command == RobotBehaviors.ROBOT_SAD:
            msg.do_motion = True
            msg.do_tts = False
            msg.do_sound_playback = False
            msg.motion = JiboAction.ROBOT_SAD

        elif command == RobotBehaviors.ROBOT_UNSURE:
            msg.do_motion = True
            msg.do_tts = True
            msg.do_sound_playback = False
            msg.tts_text = "I'm not really sure."
            msg.motion = JiboAction.ROBOT_UNSURE

        elif command == RobotBehaviors.ROBOT_COMFORT:
            msg.do_motion = True
            msg.do_tts = False
            msg.do_sound_playback = False
            msg.motion = JiboAction.ROBOT_COMFORT

        elif command == RobotBehaviors.ROBOT_ASK_HELP:
            msg,do_motion = True
            msg.do_tts = False
            msg.do_sound_playback = False
            msg.motion = JiboAction.ROBOT_ASK_HELP

        elif command == RobotBehaviors.ROBOT_DISAPPOINTED:
            msg.do_motion = True
            msg.do_tts = False
            msg.do_sound_playback = False
            msg.motion = JiboAction.ROBOT_DISAPPOINTED

        # Silent Emotions
        elif command == RobotBehaviors.ROBOT_SILENT_NOD:
            msg.do_motion = True
            msg.do_tts = False
            msg.do_sound_playback = False
            msg.motion = JiboAction.SILENT_NOD

        elif command == RobotBehaviors.ROBOT_SILENT_HAPPY_DANCE:
            msg.do_motion = True
            msg.do_tts = False
            msg.do_sound_playback = False
            msg.motion = JiboAction.SILENT_HAPPY_DANCE

        elif command == RobotBehaviors.ROBOT_SILENT_YES:
            msg.do_motion = True
            msg.do_tts = False
            msg.do_sound_playback = False
            msg.motion = JiboAction.SILENT_YES

        elif command == RobotBehaviors.ROBOT_SILENT_PUZZLED:
            msg.do_motion = True
            msg.do_tts = False
            msg.do_sound_playback = False
            msg.motion = JiboAction.SILENT_PUZZLED

        elif command == RobotBehaviors.ROBOT_SILENT_FRUSTRATED:
            msg.do_motion = True
            msg.do_tts = False
            msg.do_sound_playback = False
            msg.motion = JiboAction.SILENT_FRUSTRATED

        elif command == RobotBehaviors.ROBOT_SILENT_SAD:
            msg.do_motion = True
            msg.do_tts = False
            msg.do_sound_playback = False
            msg.motion = JiboAction.SILENT_SAD


        elif command == RobotBehaviors.ROBOT_SILENT_INTERESTED:
            msg.do_motion = True
            msg.do_tts = False
            msg.do_sound_playback = False
            msg.motion = JiboAction.SILENT_INTERESTED

        elif command == RobotBehaviors.ROBOT_SILENT_EXCITED:
            msg.do_motion = True
            msg.do_tts = False
            msg.do_sound_playback = False
            msg.motion = JiboAction.SILENT_EXCITED


        # Jibo Speech commands
        elif command == RobotBehaviors.ROBOT_HINT_BUTTON_REMINDER:
            msg.do_motion = True
            msg.do_tts = True
            msg.do_sound_playback = False
            msg.motion = JiboAction.SILENT_PUZZLED
            msg.tts_text = jibo_tts_dict["others"]["misson_reminder"]

        elif command == RobotBehaviors.ROBOT_CUSTOM_SPEECH:
            msg.do_motion = False
            msg.do_tts = True
            msg.do_sound_playback = False
            print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
            print("CUSTOM SPEECH:", args)
            print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
            try: #attempts to send TTS command from def _get_tega_speech
                difficulty = args[0][0][1].lower()
                current_state = args[0][0][2].lower()
                msg1 = args[0][0][0].lower()
                msg.tts_text = jibo_tts_dict["custom_speech"][difficulty][current_state][msg1]
            except: #accounts for other calls to ROBOT_CUSTOM_SPEECH
                path = args[0][0][0].lower()
                msg1 = args[0][0][1].lower()
                msg.tts_text = jibo_tts_dict[path][msg1]

        elif command == RobotBehaviors.NOVICE_ROLE_KEYWORD:
            msg.do_motion = True
            msg.do_tts = True
            msg.do_sound_playback = False
            msg.motion = JiboAction.SILENT_PUZZLED
            msg.tts_text = jibo_tts_dict["novice_keyword"][args[0][0].lower()]
            print("NOVICE_ROLE_KEYWORD", args[0][0].lower())

        ### ========== Jibo Speech for Role Switching Project ========== ###

        elif command == RobotBehaviors.BEFORE_GAME_SPEECH:
            msg.do_motion = False
            msg.do_tts = True
            msg.do_sound_playback = False
            hint_num = random.choice(["1", "3"])
            msg.tts_text = jibo_tts_dict["before_game"][hint_num]

        elif command == RobotBehaviors.VOCAB_EXPLANATION_SPEECH:
            msg.do_tts = True
            msg.do_motion = True
            msg.do_sound_playback = False
            vocab_word = args[0][0][0].lower()
            itype = args[0][0][1].lower()

            # Checks to see if there is a grammatical exception to an explanation or a general explanation.
            # 'except' defaults to modular explanation
            try:
                key = vocab_word + "_" + itype
                msg.tts_text = jibo_tts_dict["explanation"][key]
                print("VOCAB_EXPLANATION_SPEECH_TRY")
            except KeyError:
                key = vocab_word
                msg1 = jibo_tts_dict["explanation"][vocab_word]
                msg2 = msg1.replace("*", itype)
                msg.tts_text = msg2
                print("VOCAB_EXPLANATION_SPEECH", vocab_word, itype, msg2)

            msg.motion = JiboAction.SILENT_HAPPY_DANCE

        elif command == RobotBehaviors.HINT_SPEECH:     #TODO: ADD WIGGLE EMOTION
            msg.do_tts = True
            msg.do_motion = False   #should be True, set to wiggle
            msg.do_sound_playback = False
            vocab_word = args[0][0].lower()
            msg.tts_text = jibo_tts_dict["hint"][vocab_word]
            print("HINT_SPEECH", args[0][0].lower())

        elif command == RobotBehaviors.KEYWORD_DEFINITION_SPEECH:
            msg.do_tts = True
            msg.do_motion = True
            msg.do_sound_playback = False
            msg.motion = JiboAction.SILENT_NOD
            vocab_word = args[0][0].lower()
            msg.tts_text = jibo_tts_dict["definition"][vocab_word]
            print("KEYWORD_DEFINITION_SPEECH", args[0][0].lower())

        elif command == RobotBehaviors.REMINDER_SPEECH:
            msg.do_tts = True
            msg.do_motion = False
            msg.do_sound_playback = False
            vocab_word = args[0][0].lower()
            hint_num = random.choice(["1", "2", "3"])
            # Checks to see if there is a grammatical exception to a reminder
            # 'except' defaults to modular reminder
            try:
                key = vocab_word + "_" + hint_num
                msg.tts_text = jibo_tts_dict["reminder"][key]
                print("REMINDER_SPEECH's TRY", key)
            except KeyError:
                key = hint_num
                msg1 = jibo_tts_dict["reminder"][hint_num]
                msg2 = msg1.replace("*", vocab_word)
                msg.tts_text = msg2
                print("REMINDER_SPEECH", msg2)


        ### ========== Jibo End of Task Vocab Reminder ========== ###

        elif command == RobotBehaviors.Q_ROBOT_TASK_END_REMINDER:
            msg.do_motion = False
            msg.do_tts = True
            msg.do_sound_playback = False
            review_num = random.choice(["1", "2", "3"])
            msg.tts_text = jibo_tts_dict["review"][review_num]
            print("Q_ROBOT_TASK_END_REMINDER", args[0][0].lower())

        elif command == RobotBehaviors.ROBOT_TASK_END_RESPONSE:
            msg.do_motion = False
            msg.do_tts = True
            msg.do_sound_playback = False
            vocab_word = args[0][0].lower()
            key = "answer_" + random.choice(["1", "2"])
            msg1 = jibo_tts_dict["review"][key]
            msg2 = msg1.replace("*", vocab_word)
            msg.tts_text = msg2
            print("ROBOT_TASK_END_REMINDER", args[0][0].lower())

        elif command == RobotBehaviors.Q_ROBOT_TASK_END_ASSESSMENT:
            msg.do_motion = False
            msg.do_tts = True
            msg.do_sound_playback = False
            vocab_word = args[0][0].lower()
            key = "question_" + random.choice(["1", "2"])
            msg1 = jibo_tts_dict["review"][key]
            msg2 = msg1.replace("*", vocab_word)
            msg.tts_text = msg2
            print("Q_ROBOT_TASK_END_ASSESSMENT", msg2)

        ### ========== Jibo Speech Induction ========== ###

        elif command == RobotBehaviors.Q_ROBOT_INDUCE_SPEECH:
            msg.do_motion = True
            msg.do_tts = True
            msg.do_sound_playback = False
            msg.motion = JiboAction.SILENT_PUZZLED
            msg.tts_text = jibo_tts_dict["induce"][random.choice(["1", "2", "3", "4", "5", "6"])]
            print("Q_ROBOT_INDUCE_SPEECH", args[0][0].lower())

        elif command == RobotBehaviors.ROBOT_INDUCE_SPEECH_RESPONSE:
            msg.do_motion = False
            msg.do_tts = True
            msg.do_sound_playback = False
            vocab_word = args[0][0].lower()
            msg1 = jibo_tts_dict["review"]["answer_2"]
            msg2 = msg1.replace("*", vocab_word)
            msg.tts_text = msg2
            print("ROBOT_INDUCE_SPEECH_RESPONSE", msg2)

        ### ========== Jibo Question Asking ========== ###

        elif command == RobotBehaviors.Q_ROBOT_OFFER_HELP:
            msg.do_motion = False
            msg.do_tts = True
            msg.do_sound_playback = False
            help_msg_code = args[0][0].lower()
            msg.tts_text = jibo_tts_dict["questions"][help_msg_code]
            print("Q_ROBOT_OFFER_HELP", help_msg_code)

        elif command == RobotBehaviors.Q_ROBOT_ASK_WHY_CHOOSE_IT:
            msg.do_motion = True
            msg.do_tts = True
            msg.do_sound_playback = False
            help_msg_code = args[0][0].lower()
            msg.tts_text = jibo_tts_dict["questions"][help_msg_code]
            msg.motion = JiboAction.SILENT_PUZZLED
            print("Q_ROBOT_ASK_WHY_CHOOSE_IT", help_msg_code)

        elif command == RobotBehaviors.Q_ROBOT_WANT_LEARN:
            msg.do_motion = True
            msg.do_tts = True
            msg.do_sound_playback = False
            help_msg_code = args[0][0].lower()
            msg.tts_text = jibo_tts_dict["questions"][help_msg_code]
            msg.motion = JiboAction.SILENT_INTERESTED
            print("Q_ROBOT_ASK_HELP", args[0][0].lower())

        elif command == RobotBehaviors.Q_ROBOT_ASK_HELP:
            msg.do_motion = False
            msg.do_tts = True
            msg.do_sound_playback = False
            help_msg_code = args[0][0].lower()
            msg.tts_text = jibo_tts_dict["questions"][help_msg_code]
            print("Q_ROBOT_ASK_HELP", args[0][0].lower())

        elif command == RobotBehaviors.Q_ROBOT_ASK_WHY_WRONG:
            msg.do_motion = True
            msg.do_tts = True
            msg.do_sound_playback = False
            help_msg_code = args[0][0].lower()
            msg.tts_text = jibo_tts_dict["questions"][help_msg_code]
            msg.motion = JiboAction.SILENT_PUZZLED
            print("Q_ROBOT_ASK_WHY_WRONG", args[0][0].lower())

        elif command == RobotBehaviors.Q_END_OF_TURN:
            msg.do_motion = False
            msg.do_tts = True
            msg.do_sound_playback = False
            vocab_word = args[0][0].lower()
            msg1 = jibo_tts_dict["questions"]["end_of_turn_question"]
            msg2 = msg1.replace("*", vocab_word)
            msg.tts_text = msg2
            print("Q_END_OF_TURN", msg2)


        elif command == RobotBehaviors.ROBOT_SAY_WORD:
            msg.do_motion = False
            msg.do_tts = True
            msg.do_sound_playback = False
            msg.tts_text = args[0][0].lower()
            print("ROBOT_SAY_WORD", args[0][0].lower())

        ### ========== No iSpy Action Alert ========== ###

        elif command == RobotBehaviors.NO_ISPY_ACTION_ALERT:
            msg.do_motion = False
            msg.do_tts = True
            msg.do_sound_playback = False
            msg_code = random.choice(["no_ispy_action_alert1_response_1", "no_ispy_action_alert1_response_2"])
            msg.tts_text = args[0][0].lower()
            print("No iSpy Action Alert", args[0][0].lower())



        return msg


