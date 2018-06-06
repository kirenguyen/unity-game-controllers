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
    from std_msgs.msg import Header  # standard ROS msg header
    from unity_game_msgs.msg import TapGameCommand
    from unity_game_msgs.msg import TapGameLog
    from r1d1_msgs.msg import TegaAction
    from r1d1_msgs.msg import Vec3
    from jibo_msgs.msg import JiboAction

else:
    TapGameLog = GlobalSettings.TapGameLog  # Mock object, used for testing in non-ROS environments
    TapGameCommand = GlobalSettings.TapGameCommand
    JiboAction = GlobalSettings.JiboAction


# class JiboBehaviors:  # pylint: disable=no-member, too-many-instance-attributes
#     """
#     A Class definition for "cosmetic" robot behavior strings, which get translated by the ROSNodeMgr
#     """
#     @staticmethod
#     def get_msg_from_behavior(command, *args): #pylint: disable=too-many-branches, too-many-statements
#
#         msg = JiboAction()
#         msg.header = Header()
#         msg.header.stamp = rospy.Time.now()
#
#         if command == RobotBehaviors.LOOK_AT_TABLET:
#             msg.do_motion = True
#             msg.do_tts = False
#             msg.do_lookat = False
#             msg.do_sound_playback = False
#             msg.motion = JiboAction.LOOK_DOWN
#
#         elif command == RobotBehaviors.LOOK_CENTER:
#             msg.do_motion = True
#             msg.do_tts = False
#             msg.do_lookat = False
#             msg.do_sound_playback = False
#             msg.motion = JiboAction.DEFAULT
#
#         elif command == RobotBehaviors.WIN_SPEECH:
#             msg.do_motion = False
#             msg.do_tts = True
#             msg.do_lookat = False
#             msg.tts_text = "I win I win I win"
#
#
#         elif command == RobotBehaviors.ROBOT_TURN_SPEECH:
#             msg.do_motion = False
#             msg.do_tts = True
#             msg.do_lookat = False
#             msg.tts_text = "It is my turn now"
#
#         elif command == RobotBehaviors.RING_ANSWER_CORRECT:
#             msg.do_motion = True
#             msg.do_tts = False
#             msg.do_lookat = False
#             msg.do_sound_playback = False
#             msg.motion = JiboAction.RING_IN_ANIM
#
#         # elif command == RobotBehaviors.LATE_RING:
#         #     msg.do_motion = True
#         #     msg.do_tts = False
#         #     msg.do_lookat = False
#         #     msg.do_sound_playback = False
#         #     msg.motion = JiboAction.RING_IN_ANIM
#
#         elif command == RobotBehaviors.PRONOUNCE_CORRECT:
#             msg.do_motion = False
#             msg.do_tts = True
#             msg.do_lookat = False
#             msg.tts_text = args[0][0]
#             print(args[0][0])
#
#         elif command == RobotBehaviors.REACT_CHILD_ANSWER_WRONG:
#             msg.do_motion = False
#             msg.do_tts = False
#             msg.do_lookat = False
#             msg.do_sound_playback = True
#             msg.audio_filename = "SSA_disappointed.m4a"
#
#         # elif command == RobotBehaviors.PRONOUNCE_WRONG_SPEECH:
#         #     msg.do_motion = False
#         #     msg.do_tts = True
#         #     msg.do_lookat = False
#         #     msg.do_sound_playback = False
#         #     msg.tts_text = "I dont know this one"
#
#         elif command == RobotBehaviors.REACT_CHILD_ANSWER_CORRECT:
#             msg.do_motion = True
#             msg.do_tts = False
#             msg.do_lookat = False
#             msg.motion = "Misc/Eye_to_Happy_01.keys"
#
#         # elif command == RobotBehaviors.REACT_ANSWER_WRONG:
#         #     msg.do_motion = False
#         #     msg.do_tts = False
#         #     msg.do_lookat = False
#         #     msg.do_sound_playback = True
#         #     msg.audio_filename = "SSA_disappointed.m4a"
#
#         # elif command == RobotBehaviors.REACT_TO_BEAT_CORRECT:
#         #     msg.do_motion = True
#         #     msg.do_tts = False
#         #     msg.do_lookat = False
#         #     msg.do_sound_playback = False
#         #     msg.motion = "Misc/Frustrated_01_04.keys"
#
#
#         elif command == RobotBehaviors.REACT_GAME_START:
#             msg.do_motion = True
#             msg.do_tts = False
#             msg.do_lookat = False
#             msg.motion = "Misc/Eye_to_Happy_01.keys"
#
#
#         elif command == RobotBehaviors.REACT_GAME_START2:
#             msg.do_motion = False
#             msg.do_tts = False
#             msg.do_lookat = False
#             msg.do_sound_playback = True
#             msg.audio_filename = "SSA_laugh.m4a"
#
#         elif command == RobotBehaviors.WIN_MOTION:
#             msg.do_motion = True
#             msg.do_tts = False
#             msg.do_lookat = False
#             msg.motion = JiboAction.HAPPY_GO_LUCKY_DANCE
#
#
#         # elif command == RobotBehaviors.LOSE_MOTION:
#         #     msg.do_motion = True
#         #     msg.do_tts = False
#         #     msg.do_lookat = False
#         #     msg.motion = JiboAction.EMOJI_RAINCLOUD
#
#         # elif command == RobotBehaviors.LOSE_SPEECH:
#         #     msg.do_motion = False
#         #     msg.do_tts = True
#         #     msg.do_lookat = False
#         #     msg.tts_text = "I lost. Oh well. I'll beat you next time"
#
#         elif command == RobotBehaviors.EYE_FIDGET:
#             msg.do_motion = True
#             msg.do_tts = False
#             msg.do_lookat = False
#             msg.motion = JiboAction.EYE_FIDGET
#
#         # elif command == RobotBehaviors.REACT_TO_BEAT:
#         #     msg.do_motion = True
#         #     msg.do_lookat = False
#         #     msg.motion = "Misc/Frustrated_01_04.keys"
#
#         # elif command == RobotBehaviors.PLAYER_RING_PROMPT:
#         #     msg.do_motion = False
#         #     msg.do_tts = False
#         #     msg.do_lookat = False
#         #     msg.do_sound_playback = True
#         #     msg.audio_filename = "SSA_prompt.m4a"
#
#         return msg


class JiboBehaviors:  # pylint: disable=no-member, too-many-instance-attributes
    """
    A Class definition for "cosmetic" robot behavior strings, which get translated by the ROSNodeMgr
    """

    @staticmethod
    def get_msg_from_behavior(command, *args):  # pylint: disable=too-many-branches, too-many-statements

        msg = JiboAction()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()

        # # Look Ats
        # if command == RobotBehaviors.LOOK_AT_TABLET:
        #     msg.do_motion = True
        #     msg.do_tts = False
        #     msg.do_lookat = False
        #     msg.do_sound_playback = False
        #     msg.motion = JiboAction.LOOK_DOWN
        #
        # elif command == RobotBehaviors.LOOK_CENTER:
        #     msg.do_motion = True
        #     msg.do_tts = False
        #     msg.do_lookat = False
        #     msg.do_sound_playback = False
        #     msg.motion = JiboAction.DEFAULT
        #
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
        # #TODO: decide between TTS and Audio
        # #TODO: Audio filenames + path and TTS need a JSON file
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
        #
        # elif command == RobotBehaviors.ROBOT_SAY_WORD:
        #     msg.do_motion = False
        #     msg.do_tts = True
        #     msg.do_sound_playback = False
        #     msg.tts_text = args[0][0]
        #
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

        return msg