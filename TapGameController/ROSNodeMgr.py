"""
This is the main class that manages the creation / parsing of ROS Node Communication
"""
# -*- coding: utf-8 -*-
# pylint: disable=import-error


import json
from .TapGameUtils import GlobalSettings

if GlobalSettings.USE_ROS:
    import rospy
    from std_msgs.msg import Header  # standard ROS msg header
    from unity_game_msgs.msg import TapGameCommand
    from unity_game_msgs.msg import TapGameLog
    from r1d1_msgs.msg import TegaAction
    from jibo_msgs.msg import JiboAction  # TODO: uncomment when JiboMessage exists
else:
    TapGameLog = GlobalSettings.TapGameLog  # Mock object, used for testing in non-ROS environments
    TapGameCommand = GlobalSettings.TapGameCommand
    TegaAction = GlobalSettings.TegaAction
    JiboAction = GlobalSettings.JiboAction

ROSCORE_TO_TAP_GAME_TOPIC = '/tap_game_from_ros'
TAP_GAME_TO_ROSCORE_TOPIC = '/tap_game_to_ros'

ROSCORE_TO_JIBO_TOPIC = '/jibo'
ROSCORE_TO_TEGA_TOPIC = '/tega'

FSM_LOG_MESSAGES = [TapGameLog.CHECK_IN, TapGameLog.GAME_START_PRESSED, TapGameLog.INIT_ROUND_DONE,
                    TapGameLog.START_ROUND_DONE, TapGameLog.ROBOT_RING_IN,
                    TapGameLog.PLAYER_RING_IN, TapGameLog.END_ROUND_DONE,
                    TapGameLog.RESET_NEXT_ROUND_DONE, TapGameLog.SHOW_GAME_END_DONE,
                    TapGameLog.PLAYER_BEAT_ROBOT]


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



    def on_log_received(self, data):
        """
        Rospy Callback for when we get log messages
        """
        rospy.loginfo(rospy.get_caller_id() + "I heard " + data.message)

        if data.message in FSM_LOG_MESSAGES:

            if data.message == TapGameLog.CHECK_IN:
                print('Game Checked in!')

            if data.message == TapGameLog.GAME_START_PRESSED:
                self.init_first_round()  # makes state transition + calls self.on_init_first_round()

            if data.message == TapGameLog.INIT_ROUND_DONE:
                print('done initializing')
                self.start_round()

            if data.message == TapGameLog.START_ROUND_DONE:
                print('I heard Start Round DONE. Waiting for player input')

            if data.message == TapGameLog.PLAYER_RING_IN:
                print('Player Rang in!')
                self.player_ring_in()

            if data.message == TapGameLog.ROBOT_RING_IN:
                print('Robot Rang in!')
                self.robot_ring_in()

            if data.message == TapGameLog.PLAYER_BEAT_ROBOT:
                self.player_beat_robot()

            if data.message == TapGameLog.RESET_NEXT_ROUND_DONE:
                print('Done Resetting Round!')
                self.send_robot_cmd("LOOK_AT_TABLET")
                self.current_round_word = self.student_model.get_next_best_word()
                self.send_game_cmd(TapGameCommand.INIT_ROUND, json.dumps(self.current_round_word))

            if data.message == TapGameLog.SHOW_GAME_END_DONE:
                print('GAME OVER!')
        else:
            print('NOT A REAL MESSAGE?!?!?!?')

    def start_log_listener(self):
        """
        Start up the Game Log Subscriber node
        """
        print('Sub Node started')
        rospy.init_node('FSM_Listener_Controller', anonymous=True)
        self.log_listener = rospy.Subscriber(TAP_GAME_TO_ROSCORE_TOPIC, TapGameLog,
                                             self.on_log_received)

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

    def send_robot_cmd(self, command, *args):
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
                if len(args) > 0:
                    msg.params = args[0]

            if command == 'RING_ANSWER_CORRECT':
                msg.do_motion = True
                msg.do_tts = False
                msg.do_lookat = False
                msg.do_sound_playback = False
                msg.motion = JiboAction.RING_IN_ANIM
                if len(args) > 0:
                    msg.params = args[0]

            elif command == 'PRONOUNCE_CORRECT':
                msg.do_motion = False
                msg.do_tts = True
                msg.do_lookat = False
                msg.tts_text = self.current_round_word
                if len(args) > 0:
                    msg.params = args[0]

            elif command == 'JIBO_WIN_MOTION':
                msg.do_motion = True
                msg.do_tts = False
                msg.do_lookat = False
                msg.motion = JiboAction.HAPPY_GO_LUCKY_DANCE
                if len(args) > 0:
                    msg.params = args[0]

            elif command == 'JIBO_WIN_SPEECH':
                msg.do_motion = True
                msg.do_tts = True
                msg.do_lookat = False
                msg.tts_text = "I win I win I win I win I win"
                if len(args) > 0:
                    msg.params = args[0]

            elif command == 'JIBO_LOSE_MOTION':
                msg.do_motion = True
                msg.do_tts = False
                msg.do_lookat = False
                msg.motion = JiboAction.EMOJI_RAINCLOUD
                if len(args) > 0:
                    msg.params = args[0]

            elif command == 'JIBO_LOSE_SPEECH':
                msg.do_motion = True
                msg.do_tts = True
                msg.do_lookat = False
                msg.tts_text = "I lost. Oh well. I'll beat you next time"
                if len(args) > 0:
                    msg.params = args[0]

        else:
            pass
            # USE TEGA

        self.robot_commander.publish(msg)
        rospy.loginfo(msg)