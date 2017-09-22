"""
This is the main FSM / Game Logic class for the Tap Game
"""
# -*- coding: utf-8 -*-
# pylint: disable=import-error


import json
import time
import _thread as thread
from random import randint

from transitions import Machine

from GameUtils import GlobalSettings
from GameUtils.PronunciationUtils.PronunciationUtils import PronunciationUtils
from GameUtils.AudioRecorder import AudioRecorder

from .ROSNodeMgr import ROSNodeMgr
from GameUtils.Curriculum import Curriculum

if GlobalSettings.USE_ROS:
    from unity_game_msgs.msg import TapGameCommand
    from unity_game_msgs.msg import TapGameLog
else:
    TapGameLog = GlobalSettings.TapGameLog #Mock object, used for testing in non-ROS environments
    TapGameCommand = GlobalSettings.TapGameCommand

RECORD_TIME_MS = 3500
SHOW_RESULTS_TIME_MS = 2500
WAIT_TO_BUZZ_TIME_MS = 1500 #note, game currently waits 3000ms after receiving message
SIMULATED_ROBOT_RESULTS_TIME_MS = 2500 # time to wait while we "process" robot speech (should be close to SpeechAce roundtrip time)

PASSING_RATIO_THRESHOLD = .65

FSM_LOG_MESSAGES = [TapGameLog.CHECK_IN, TapGameLog.GAME_START_PRESSED, TapGameLog.INIT_ROUND_DONE,
                    TapGameLog.START_ROUND_DONE, TapGameLog.PLAYER_RING_IN, TapGameLog.END_ROUND_DONE,
                    TapGameLog.RESET_NEXT_ROUND_DONE, TapGameLog.SHOW_GAME_END_DONE,
                    TapGameLog.RESTART_GAME]


class TapGameFSM: # pylint: disable=no-member, too-many-instance-attributes
    """
    An FSM for the Tap Game. Contains Game Logic and some nodes for interacting w the Unity "View"
    """

    round_index = 1
    max_score = 5 #game ends when someone gets to this score

    player_score = 0
    robot_score = 0

    pronunciation_utils = PronunciationUtils()
    ros_node_mgr = ROSNodeMgr()
    current_round_word = ""

    game_commander = None
    robot_commander = None
    log_listener = None
    letters = None
    passed = None


    states = ['GAME_START', 'ROUND_START', 'ROUND_ACTIVE',
              'PLAYER_PRONOUNCE', 'SHOW_RESULTS',
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

        {'trigger': 'player_ring_in',
         'source': 'ROUND_ACTIVE',
         'dest': 'PLAYER_PRONOUNCE',
         'after': 'on_player_ring_in'},

        {'trigger': 'player_pronounce_eval',
         'source': 'PLAYER_PRONOUNCE',
         'dest': 'SHOW_RESULTS',
         'after': 'on_player_pronounce_eval'},

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
         'dest': 'GAME_START',
         'after': 'on_game_replay'},
    ]

    def __init__(self, participant_id, experimenter_name, experiment_phase):

        self.state_machine = Machine(self, states=self.states, transitions=self.transitions,
                                     initial='GAME_START')

        self.participant_id = participant_id
        self.experimenter_name = experimenter_name
        self.experiment_phase = experiment_phase

        if not  self.experiment_phase == 'practice':
            print(str(self.experiment_phase) + " was not 'practice'")
            exit()

        # Initializes a new audio recorder object if one hasn't been created
        self.recorder = AudioRecorder(self.participant_id, self.experimenter_name, self.experiment_phase)

        # Tell robot to look at Tablet
        self.ros_node_mgr.init_ros_node()
        #self.ros_node_mgr.send_robot_cmd(RobotBehaviors.LOOK_AT_TABLET)

        # fancy python one-liner to read all string attributes off of a class
        self.curriculum = [p for p in dir(Curriculum)
                           if isinstance(getattr(Curriculum, p), str)
                           and not p.startswith('__')]

    def on_init_first_round(self):
        """
        Called when the game registers with the controller
        Should send msg to Unity game telling it the word to load for the first round
        """
        print("got to init_first round!")

        #send message every 2s in case it gets dropped
        def send_msg_til_received():
            while(self.state == "ROUND_START"):
                self.current_round_word = self.curriculum[self.round_index - 1]
                self.ros_node_mgr.send_game_cmd(TapGameCommand.INIT_ROUND,
                                                json.dumps(self.current_round_word))
                print('sent command!')
                print(self.state)
                time.sleep(5)

        thread.start_new_thread(send_msg_til_received, ())


    def on_start_round(self):
        """
        Called when the game registers that the round initialization is done
        Should send msg to Unity game telling it to begin countdown and make buzzers active
        """
        print('got to start round cb')
        self.ros_node_mgr.send_game_cmd(TapGameCommand.START_ROUND)


    def on_player_ring_in(self):
        """
        Called when the human player has tapped their buzzer to ring in
        Should send msg to Unity game telling it to load the pronunciation screen
        And also start recording from the phone for 5 seconds + writing to wav
        """
        print('got to player ring in cb')

        #SEND SHOW_PRONUNCIATION_PAGE MSG
        self.recorder.start_recording(self.current_round_word, RECORD_TIME_MS)
        #time.sleep(RECORD_TIME_MS / 1000.0)
        self.recorder.stop_recording()

        ##Evaluates the action message

        ## If given a word to evaluate and done recording send the information to speechace
        if self.current_round_word and \
                                self.recorder.has_recorded % 2 == 0 and \
                        self.recorder.has_recorded != 0:

            # If you couldn't find the android audio topic, automatically pass
            # instead of using the last audio recording
            if not self.recorder.valid_recording:
                self.letters = list(self.current_round_word)
                self.passed = ['0'] * len(self.letters)
                print ("NO RECORDING SO YOU AUTOMATICALLY FAIL")
            else:
                audio_file = self.recorder.WAV_OUTPUT_FILENAME_PREFIX + self.current_round_word + '.wav'
                print("SENDING TO SPEECHACE")
                word_score_list = self.recorder.speechace(audio_file)
                print("WORD SCORE LIST")
                print(word_score_list)

                # if we didn't record, there will be no word score list
                if word_score_list:
                    for word_results in word_score_list:
                        print("Message for ROS")
                        self.letters, self.passed = \
                            self.pronunciation_utils.process_speechace_word_results(word_results)
                        print(self.letters)
                        print(self.passed)
                else:
                    self.letters = list(self.current_round_word)
                    self.passed = ['0'] * len(self.letters)
                    print('NO RECORDING, SO YOU AUTO-FAIL!!')

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
        tmp = [int(x) for x in self.passed]
        passed_ratio = (sum(tmp) / len(tmp)) #TODO: do this over phonemes, not letters!
        print("PASSED RATIO WAS" + str(passed_ratio))
        means, variances = self.student_model.train_and_compute_posterior([self.current_round_word],
                                                                          [passed_ratio])

        if passed_ratio > PASSING_RATIO_THRESHOLD:
            self.player_score += 1

        print("LATEST MEANS / VARS")
        print(self.student_model.curriculum)
        print(means)
        print(variances)

        results_params = {}
        results_params['letters'] = self.letters
        results_params['passed'] = self.passed

        self.ros_node_mgr.send_game_cmd(TapGameCommand.SHOW_RESULTS, json.dumps(results_params))
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
        self.ros_node_mgr.send_game_cmd(TapGameCommand.RESET_NEXT_ROUND)



    def on_game_finished(self):
        """
        Called when we have completed 'max_rounds' rounds in a game.
        Sends msg to the Unity game to load the game end screen
        """
        print('got to game finished')
        self.ros_node_mgr.send_game_cmd(TapGameCommand.SHOW_GAME_END)



    def on_game_replay(self):
        """
        Called when the player wants to replay the game after finishing.
        Sends msg to the Unity game to reset the game and start over
        """

        # reset all state variables (rounds, score)
        self.player_score = 0
        self.robot_score = 0
        self.init_first_round()

        #reset student model here if needed

        #self.ros_node_mgr.send_game_cmd(TapGameCommand.RESTART_GAME) #START GAME OVER
        #self.ros_node_mgr.send_game_cmd()


    def on_log_received(self, data):
        """
        Rospy Callback for when we get log messages
        """
        if data.message in FSM_LOG_MESSAGES:

            if data.message == TapGameLog.CHECK_IN:
                print('Game Checked in!')

            if data.message == TapGameLog.GAME_START_PRESSED:
                time.sleep(500 / 1000.0)
                self.init_first_round()  # makes state transition + calls self.on_init_first_round()

            if data.message == TapGameLog.INIT_ROUND_DONE:
                print('done initializing')
                self.start_round()

            if data.message == TapGameLog.START_ROUND_DONE:
                print('I heard Start Round DONE. Waiting for player input')


            if data.message == TapGameLog.PLAYER_RING_IN:
                print('Player Rang in!')
                self.player_ring_in()

            if data.message == TapGameLog.RESET_NEXT_ROUND_DONE:
                print('Game Done Resetting Round! Now initing new round')
                self.current_round_word = self.curriculum[self.round_index - 1]
                self.ros_node_mgr.send_game_cmd(TapGameCommand.INIT_ROUND, json.dumps(self.current_round_word))

            if data.message == TapGameLog.SHOW_GAME_END_DONE:
                print('GAME OVER! WAIT FOR RESET SIGNAL')

            if data.message == TapGameLog.RESTART_GAME:
                self.replay_game()
        else:
            print('NOT A REAL MESSAGE?!?!?!?')


    def is_last_round(self):
        """
        used by FSM to determine whether to start next round or end game
        """
        return (self.round_index == len(self.curriculum))

    def is_not_last_round(self):
        """
        used by FSM to determine whether to start next round or end game
        """
        return not self.is_last_round()
