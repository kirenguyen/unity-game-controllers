"""
This is the main class that manages the creation / parsing of ROS Node Communication
"""
# -*- coding: utf-8 -*-
# pylint: disable=import-error

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
    TegaAction = GlobalSettings.TegaAction
    JiboAction = GlobalSettings.JiboAction

ROSCORE_TO_TAP_GAME_TOPIC = '/tap_game_from_ros'
TAP_GAME_TO_ROSCORE_TOPIC = '/tap_game_to_ros'

ROSCORE_TO_JIBO_TOPIC = '/jibo'
ROSCORE_TO_TEGA_TOPIC = '/tega'

class ROSNodeMgr:  # pylint: disable=no-member, too-many-instance-attributes
    """
    A Manager for the ROS Communication in the Tap Game. Contains nodes for interacting
    w the Unity "View" and also the Robot
    """

    game_commander = None
    robot_commander = None
    log_listener = None


    def __init__(self):
        pass


    def init_ros_node(self):
        rospy.init_node('FSM_Listener_Controller', anonymous=True)
        
    def start_log_listener(self, on_log_callback):
        """
        Start up the Game Log Subscriber node
        """
        print('Sub Node started')        
        self.log_listener = rospy.Subscriber(TAP_GAME_TO_ROSCORE_TOPIC, TapGameLog,
                                             on_log_callback)

    def start_cmd_publisher(self):
        """
        Starts up the command publisher node
        """
        print('GameCmd Pub Node started')
        self.game_commander = rospy.Publisher(ROSCORE_TO_TAP_GAME_TOPIC,
                                              TapGameCommand, queue_size=10)
        rate = rospy.Rate(10)  # spin at 10 Hz
        rate.sleep()  # sleep to wait for subscribers
        # rospy.spin()

    def start_robot_publisher(self):
        """
        Starts up the robot publisher node
        """
        print('Robot Pub Node started')

        if GlobalSettings.USE_TEGA:
            msg_type = TegaAction
            msg_topic = ROSCORE_TO_TEGA_TOPIC
        else:
            msg_type = JiboAction
            msg_topic = ROSCORE_TO_JIBO_TOPIC

        self.robot_commander = rospy.Publisher(msg_topic, msg_type, queue_size=10)
        rate = rospy.Rate(10)  # spin at 10 Hz
        rate.sleep()  # sleep to wait for subscribers

    def send_game_cmd(self, command, *args):
        """
        send a TapGameCommand to game
        Args are optional parameters
        """

        # send message to tablet game
        if self.game_commander is None:
            self.start_cmd_publisher()

        msg = TapGameCommand()
        # add header
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()

        # fill in command and any params:
        msg.command = command
        if len(args) > 0:
            msg.params = args[0]        
        self.game_commander.publish(msg)
        rospy.loginfo(msg)

    def send_robot_cmd(self, command, *args): #pylint: disable=too-many-branches, too-many-statements
        """
        send a Command from the ActionSpace to the robot
        This function maps actions from the ActionSpace into actual ROS Msgs
        """

        if self.robot_commander is None:
            self.start_robot_publisher()

        # choose which platform
        if GlobalSettings.USE_TEGA:
            msg = TegaAction()
        else:
            msg = JiboAction()

        # add header
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()

        if not GlobalSettings.USE_TEGA:
            if command == 'LOOK_AT_TABLET':
                msg.do_motion = True
                msg.do_tts = False
                msg.do_lookat = False
                msg.do_sound_playback = False
                msg.motion = JiboAction.LOOK_DOWN

            if command == 'RING_ANSWER_CORRECT':
                msg.do_motion = True
                msg.do_tts = False
                msg.do_lookat = False
                msg.do_sound_playback = False
                msg.motion = JiboAction.RING_IN_ANIM

            elif command == 'PRONOUNCE_CORRECT':
                msg.do_motion = False
                msg.do_tts = True
                msg.do_lookat = False
                msg.tts_text = args[0]

            elif command == 'JIBO_WIN_MOTION':
                msg.do_motion = True
                msg.do_tts = False
                msg.do_lookat = False
                msg.motion = JiboAction.HAPPY_GO_LUCKY_DANCE

            elif command == 'JIBO_WIN_SPEECH':
                msg.do_motion = False
                msg.do_tts = True
                msg.do_lookat = False
                msg.tts_text = "I win I win I win I win I win"

            elif command == 'JIBO_LOSE_MOTION':
                msg.do_motion = True
                msg.do_tts = False
                msg.do_lookat = False
                msg.motion = JiboAction.EMOJI_RAINCLOUD

            elif command == 'JIBO_LOSE_SPEECH':
                msg.do_motion = False
                msg.do_tts = True
                msg.do_lookat = False
                msg.tts_text = "I lost. Oh well. I'll beat you next time"

            elif command == 'EYE_FIDGET':
                msg.do_motion = True
                msg.do_tts = False
                msg.do_lookat = False
                msg.motion = JiboAction.EYE_FIDGET
                               
        else:
            if command == 'LOOK_AT_TABLET':
                msg.do_motion = False
                msg.do_text_to_speech = False
                msg.do_look_at = True
                lookat_pos = Vec3()
                lookat_pos.x = 0
                lookat_pos.y = -30
                lookat_pos.z = 40
                msg.look_at = lookat_pos
            # USE TEGA

        self.robot_commander.publish(msg)
        rospy.loginfo(msg)
