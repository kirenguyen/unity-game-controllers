"""
This is the main FSM / Game Logic class for the Tap Game
"""
# -*- coding: utf-8 -*-
# pylint: disable=import-error


import json
import time
from transitions import Machine
from .TapGameUtils import GlobalSettings
from .TapGameUtils.PronunciationUtils import PronunciationHandler
from .StudentModel import StudentModel
from .TapGameAudioRecorder import TapGameAudioRecorder
from .AgentModel import ActionSpace
from .AgentModel import AgentModel


if GlobalSettings.USE_ROS:
    import rospy
    from std_msgs.msg import Header  # standard ROS msg header
    from unity_game_msgs.msg import TapGameCommand
    from unity_game_msgs.msg import TapGameLog
    from r1d1_msgs.msg import TegaAction
    from jibo_msgs.msg import JiboAction #TODO: uncomment when JiboMessage exists
else:
    TapGameLog = GlobalSettings.TapGameLog #Mock object, used for testing in non-ROS environments
    TapGameCommand = GlobalSettings.TapGameCommand
    TegaAction = GlobalSettings.TegaAction
    JiboAction = GlobalSettings.JiboAction

ROSCORE_TO_TAP_GAME_TOPIC = '/tap_game_from_ros'
TAP_GAME_TO_ROSCORE_TOPIC = '/tap_game_to_ros'

ROSCORE_TO_JIBO_TOPIC = '/jibo'
ROSCORE_TO_TEGA_TOPIC = '/tega'

RECORD_TIME_MS = 3500
SHOW_RESULTS_TIME_MS = 3500
WAIT_TO_BUZZ_TIME_MS = 3500 #note, game currently waits 3000ms after receiving message

FSM_LOG_MESSAGES = [TapGameLog.CHECK_IN, TapGameLog.GAME_START_PRESSED, TapGameLog.INIT_ROUND_DONE,
                    TapGameLog.START_ROUND_DONE, TapGameLog.ROBOT_RING_IN,
                    TapGameLog.PLAYER_RING_IN, TapGameLog.END_ROUND_DONE,
                    TapGameLog.RESET_NEXT_ROUND_DONE, TapGameLog.SHOW_GAME_END_DONE,
                    TapGameLog.PLAYER_BEAT_ROBOT]



class TapGameFSM: # pylint: disable=no-member, too-many-instance-attributes
    """
    An FSM for the Tap Game. Contains Game Logic and some nodes for interacting w the Unity "View"
    """

    round_index = 1
    max_rounds = 5

    student_model = StudentModel()
    agent_model = AgentModel()
    recorder = TapGameAudioRecorder()
    pronunciation_handler = PronunciationHandler()
    current_round_word = ""

    game_commander = None
    robot_commander = None
    log_listener = None
    letters = None
    passed = None

    states = ['GAME_START', 'ROUND_START', 'ROUND_ACTIVE',
              'PLAYER_PRONOUNCE', 'ROBOT_PRONOUNCE', 'SHOW_RESULTS',
              'GAME_FINISHED']

    transitions = [
        {'trigger': 'init_first_round',
         'source': 'GAME_START',
         'dest': 'ROUND_START',
         'after': 'on_init_first_round'},

        {'trigger': 'start_round',
         'source': 'ROUND_START',
         'dest': 'ROUND_ACTIVE',
         'after': 'on_start_round'},

        {'trigger': 'robot_ring_in',
         'source': 'ROUND_ACTIVE',
         'dest': 'ROBOT_PRONOUNCE',
         'after': 'on_robot_ring_in'},

        {'trigger': 'player_ring_in',
         'source': 'ROUND_ACTIVE',
         'dest': 'PLAYER_PRONOUNCE',
         'after': 'on_player_ring_in'},

        {'trigger': 'player_pronounce_eval',
         'source': 'PLAYER_PRONOUNCE',
         'dest': 'SHOW_RESULTS',
         'after': 'on_player_pronounce_eval'},

        {'trigger': 'robot_pronounce_eval',
         'source': 'ROBOT_PRONOUNCE',
         'dest': 'SHOW_RESULTS',
         'after': 'on_robot_pronounce_eval'},

        {'trigger': 'handle_round_end',
         'source': 'SHOW_RESULTS',
         'dest': 'ROUND_START',
         'conditions': 'is_not_last_round',
         'after': 'on_round_reset'},

        {'trigger': 'handle_round_end',
         'source': 'SHOW_RESULTS',
         'dest': 'GAME_FINISHED',
         'conditions': 'is_last_round',
         'after': 'on_game_finished'},

        {'trigger': 'replay_game',
         'source': 'GAME_FINISHED',
         'dest': 'ROUND_START',
         'after': 'on_game_replay'},
    ]

    def __init__(self):

        self.state_machine = Machine(self, states=self.states, transitions=self.transitions,
                                     initial='GAME_START')

        print('graphing distribution!')
        self.student_model.plot_curricular_distro()

    def on_init_first_round(self):
        """
        Called when the game registers with the controller
        Should send msg to Unity game telling it the word to load for the first round
        """
        print("got to init_first round!")
        self.current_round_word = self.student_model.get_next_best_word()
        self.send_game_cmd(TapGameCommand.INIT_ROUND, json.dumps(self.current_round_word))

        # #send message every 2s in case it gets dropped
        # while(not self.state == "ROUND_ACTIVE"):
        #     self.send_cmd(TapGameCommand.INIT_ROUND, self.student_model.get_next_best_word() )
        #     print('sent command!')
        #     print(self.state)
        #     time.sleep(2)

    def on_start_round(self):
        """
        Called when the game registers that the round initialization is done
        Should send msg to Unity game telling it to begin countdown and make buzzers active
        """
        print('got to start round cb')
        self.send_game_cmd(TapGameCommand.START_ROUND)

        # get the next robot action
        next_action = self.agent_model.get_next_action()

        if next_action == ActionSpace.RING_ANSWER_CORRECT:
            time.sleep(WAIT_TO_BUZZ_TIME_MS / 1000.0)
            self.send_robot_cmd(next_action)
            self.send_game_cmd(TapGameCommand.ROBOT_RING_IN)

    def on_robot_ring_in(self):
        """
        Called when the game registers that the robot 'buzzed in'
        Should send msg to Unity game telling it to load robot pronunciation screen
        """
        print('got to robot ring in cb')

        # Send message to robot telling it to pronounce

        # Wait a few seconds, pronounce word, then wait again
        time.sleep((RECORD_TIME_MS / 2) / 1000.0)
        self.send_robot_cmd("PRONOUNCE_CORRECT")
        time.sleep((RECORD_TIME_MS / 2) / 1000.0)

        # Move to evaluation phase
        self.letters = list(self.current_round_word)
        self.passed = ['1'] * len(self.letters) #TODO: robot always gets it perfect for now

        #self.send
        self.robot_pronounce_eval()

    def player_beat_robot(self):
        print("PLAYER BEAT ROBOT TO THE PUNCH!")


    def on_player_ring_in(self):
        """
        Called when the human player has tapped their buzzer to ring in
        Should send msg to Unity game telling it to load the pronunciation screen
        And also start recording from the phone for 5 seconds + writing to wav
        """
        print('got to player ring in cb')

        # Initializes a new audio recorder object if one hasn't been created
        if self.recorder is None:
            self.recorder = TapGameAudioRecorder()

        #SEND SHOW_PRONUNCIATION_PAGE MSG
        self.recorder.start_recording()
        time.sleep(RECORD_TIME_MS / 1000.0)
        self.recorder.stop_recording()

        ##Evaluates the action message

        ## If given a word to evaluate and done recording send the information to speechace
        if self.current_round_word and \
           self.recorder.has_recorded % 2 == 0 and\
           self.recorder.has_recorded != 0:

            audio_file = TapGameAudioRecorder.WAV_OUTPUT_FILENAME
            word_score_list = self.recorder.speechace(audio_file, self.current_round_word)
            print("WORD SCORE LIST")
            print(word_score_list)

            # if we didn't record, there will be no word score list
            if word_score_list:
                for word_results in word_score_list:
                    print("Message for ROS")
                    self.letters, self.passed = \
                        self.pronunciation_handler.process_speechace_word_results(word_results)
                    print(self.letters)
                    print(self.passed)
            else:
                self.letters = list(self.current_round_word)
                self.passed = ['1'] * len(self.letters)
                print('NO RECORDING, SO YOU AUTO-PASS!!')

            self.player_pronounce_eval()
        else:
            print('THIS SHOULD NEVER HAPPEN')

    def on_player_pronounce_eval(self):
        """
        Called after the human player has pronounced their buzzer to ring in
        send wav from previous step to speech ace, get results, update model, and
        send message to game to display results
        """
        print('got to player pronounce eval cb')
        # Get the actual results here
        # TODO: the [1] is only for fully correct answers!!!!
        tmp = [int(x) for x in self.passed]
        passed_ratio = (sum(tmp) / len(tmp)) #TODO: do this over phonemes, not letters!
        means, variances = self.student_model.train_and_compute_posterior([self.current_round_word],
                                                                          [passed_ratio])

        print("LATEST MEANS / VARS")
        print(self.student_model.curriculum)
        print(means)
        print(variances)

        # TODO Send message to Game to show results for three seconds, sleep, + handle round_end
        results_params = {}
        results_params['letters'] = self.letters
        results_params['passed'] = self.passed

        self.send_game_cmd(TapGameCommand.SHOW_RESULTS, json.dumps(results_params))
        time.sleep(SHOW_RESULTS_TIME_MS / 1000.0)
        self.handle_round_end()

    def on_robot_pronounce_eval(self):
        """
        Called after the robot has 'pronounced' a word. Should send mesage to Game telling it
        to show results
        handle_round_end() to transition to next round
        """

        #TODO Send message to Game to show results for three seconds, sleep, + handle round_end

        results_params = {}
        results_params['letters'] = self.letters
        results_params['passed'] = self.passed

        self.send_game_cmd(TapGameCommand.SHOW_RESULTS, json.dumps(results_params))
        time.sleep(SHOW_RESULTS_TIME_MS / 1000.0)
        self.handle_round_end()

    def on_round_reset(self):
        """
        Called after finishing a round in the game, but we have not hit
        'max_rounds" yet
        Should increment round index, and send cmd to game to reset for the next round
        """
        print('got to round reset')        
        self.round_index += 1
        self.send_game_cmd(TapGameCommand.RESET_NEXT_ROUND)



    def on_game_finished(self):
        """
        Called when we have completed 'max_rounds' rounds in a game.
        Sends msg to the Unity game to load the game end screen
        """
        print('got to game finished')
        #self.student_model.plot_curricular_distro()
        self.send_game_cmd(TapGameCommand.SHOW_GAME_END)



    def on_game_replay(self):
        """
        Called when the player wants to replay the game after finishing.
        Sends msg to the Unity game to reset the game and start over
        """


    def on_log_received(self, data):
        """
        Rospy Callback for when we get log messages
        """
        rospy.loginfo(rospy.get_caller_id() + "I heard " + data.message)

        if data.message in FSM_LOG_MESSAGES:

            if data.message == TapGameLog.CHECK_IN:
                print('Game Checked in!')

            if data.message == TapGameLog.GAME_START_PRESSED:
                self.init_first_round() #makes state transition + calls self.on_init_first_round()

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
        #rospy.spin()

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

        if GlobalSettings.USE_TEGA:
            msg = TegaAction()
        else:
            msg = JiboAction()

        if self.robot_commander is None:
            self.start_robot_publisher()

        # add header
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()


        if command == 'RING_ANSWER_CORRECT' and not GlobalSettings.USE_TEGA: #this handles the mapping
            msg.do_motion = False
            msg.do_tts = False
            msg.do_lookat = False
            msg.do_sound_playback = True
            msg.audio_filename = JiboAction.RING_IN_SOUND
            if len(args) > 0:
                msg.params = args[0]

        if command == 'PRONOUNCE_CORRECT' and not GlobalSettings.USE_TEGA: #this handles the mapping
            msg.do_motion = False
            msg.do_tts = True
            msg.do_lookat = False
            msg.tts_text = self.current_round_word
            if len(args) > 0:
                msg.params = args[0]


        self.robot_commander.publish(msg)
        rospy.loginfo(msg)

        #elif command == 'REACT_FRUSTRATED':
        #    msg.do_motion = True
        #    msg.motion = "CONFIRM" if GlobalSettings.USE_TEGA else JiboAction.SILENT_CONFIRM



    def is_last_round(self):
        """
        used by FSM to determine whether to start next round or end game
        """
        return self.round_index == self.max_rounds

    def is_not_last_round(self):
        """
        used by FSM to determine whether to start next round or end game
        """
        return not self.is_last_round()
