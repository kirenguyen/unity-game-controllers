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

        # elif command == RobotBehaviors.LOOK_LEFT_RIGHT:
        #     msg.do_motion = True
        #     msg.do_tts = False
        #     msg.do_lookat = False
        #     msg.do_sound_playback = False
        #     msg.motion = JiboAction.
        #
        # elif command == RobotBehaviors.LOOK_DOWN_CENTER:
        #     msg.do_motion = True
        #     msg.do_tts = False
        #     msg.do_lookat = False
        #     msg.do_sound_playback = False
        #     msg.motion = JiboAction.
        #
        # # Positive Emotions
        # elif command == RobotBehaviors.ROBOT_EXCITED:
        #     msg.do_motion = True
        #     msg.do_tts = False
        #     msg.do_lookat = False
        #     msg.do_sound_playback = True
        #     msg.audio_filename =
        #     msg.motion = JiboAction
        #
        # elif command == RobotBehaviors.ROBOT_INTERESTED:
        #     msg.do_motion = True
        #     msg.do_tts = True
        #     msg.do_lookat = False
        #     msg.do_sound_playback = False
        #     msg.tts_text =
        #
        # elif command == RobotBehaviors.ROBOT_YES:
        #     msg.do_motion = True
        #     msg.do_tts = True
        #     msg.do_lookat = False
        #     msg.do_sound_playback = False
        #     msg.tts_text =
        #     msg.motion = JiboAction.
        #
        # elif command == RobotBehaviors.ROBOT_HAPPY_DANCE:   #TODO: check if happy dance has audio
        #     msg.do_motion = True
        #     msg.do_tts = False
        #     msg.do_lookat = False
        #     msg.do_sound_playback = True
        #     msg.audio_filename =
        #     msg.motion = JiboAction.
        #
        # elif command == RobotBehaviors.ROBOT_CURIOUS:
        #     msg.do_motion = True
        #     msg.do_tts = True
        #     msg.do_lookat = False
        #     msg.do_sound_playback = False
        #     msg.tts_text =
        #     msg.motion = JiboAction.
        #
        # elif command == RobotBehaviors.ROBOT_ATTENTION:
        #     msg.do_motion = True
        #     msg.do_tts = True
        #     msg.do_lookat = False
        #     msg.do_sound_playback = False
        #     msg.tts_text =
        #     msg.motion = JiboAction.
        #
        # elif command == RobotBehaviors.ROBOT_CELEBRATION:
        #     msg.do_motion = True
        #     msg.do_tts = False
        #     msg.do_sound_playback = False
        #     msg.motion = JiboAction.
        #
        # elif command == RobotBehaviors.ROBOT_ENCOURAGING:
        #     msg.do_motion = True
        #     msg.do_tts = True
        #     msg.do_sound_playback = False
        #     msg.tts_text =
        #     msg.motion = JiboAction.
        #
        # elif command == RobotBehaviors.ROBOT_WINK:
        #     msg.do_motion = True
        #     msg.do_tts = False
        #     msg.do_sound_playback = False
        #     msg.motion = JiboAction.
        #
        # elif command == RobotBehaviors.ROBOT_THINKING:
        #     msg.do_motion = True
        #     msg.do_tts = True
        #     msg.do_sound_playback = False
        #     msg.motion = JiboAction.

        elif command == RobotBehaviors.ROBOT_SAY_WORD:
            msg.do_motion = False
            msg.do_tts = True
            msg.do_sound_playback = False
            msg.tts_text = args[0][0].lower()

        # # Negative Emotions
        # elif command == RobotBehaviors.ROBOT_SAD:
        #     msg.do_motion = True
        #     msg.do_tts = False
        #     msg.do_sound_playback = True
        #     msg.audio_filename =
        #     msg.motion = JiboAction
        #
        # elif command == RobotBehaviors.ROBOT_UNSURE:
        #     msg.do_motion = True
        #     msg.do_tts = True
        #     msg.do_sound_playback = False
        #     msg.tts_text =
        #     msg.motion = JiboAction
        #
        # elif command == RobotBehaviors.ROBOT_COMFORT:
        #     msg.do_motion = True
        #     msg.do_tts = True
        #     msg.do_sound_playback = False
        #     msg.tts_text =
        #     msg.motion = JiboAction
        #
        # elif command == RobotBehaviors.ROBOT_ASK_HELP:
        #     msg,do_motion = True
        #     msg.do_tts = True
        #     msg.do_sound_playback = False
        #     msg.tts_text =
        #     msg.motion = JiboAction
        #
        # elif command == RobotBehaviors.ROBOT_DISAPPOINTED:
        #     msg.do_motion = True
        #     msg.do_tts = False
        #     msg.do_sound_playback = True
        #     msg.audio_filename =
        #     msg.motion = JiboAction


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
            msg.tts_text = args[0][0].lower()

        elif command == RobotBehaviors.NOVICE_ROLE_KEYWORD:
            msg.do_motion = True
            msg.do_tts = True
            msg.do_sound_playback = False
            msg.motion = JiboAction.SILENT_PUZZLED
            msg.tts_text = jibo_tts_dict["novice_keyword"][args[0][0].lower()]

        ### ========== Jibo Speech for Role Switching Project ========== ###
        elif command == RobotBehaviors.BEFORE_GAME_SPEECH:
            msg.do_motion = False
            msg.do_tts = True
            msg.do_sound_playback = False
            hint_num = random.choice(["1", "3"])
            msg.tts_text = jibo_tts_dict["before_game"][hint_num]

        elif command == RobotBehaviors.VOCAB_EXPLANATION_SPEECH:





        return msg


