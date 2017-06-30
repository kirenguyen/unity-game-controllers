"""
This is a basic class for the Game Controller
"""
# -*- coding: utf-8 -*-
# pylint: disable=import-error


import json
import time
from transitions import Machine
from .TapGameUtils import GlobalSettings
from .StudentModel import StudentModel
from .TapGameAudioRecorder import TapGameAudioRecorder


if GlobalSettings.USE_ROS:
    import rospy
    from std_msgs.msg import Header  # standard ROS msg header
    from unity_game_msgs.msg import TapGameCommand
    from unity_game_msgs.msg import TapGameLog
else:
    TapGameLog = GlobalSettings.TapGameLog #Mock object, used for testing in non-ROS environments
    TapGameCommand = GlobalSettings.TapGameCommand

ROS_TO_TAP_GAME_TOPIC = '/tap_game_from_ros'
TAP_GAME_TO_ROS_TOPIC = '/tap_game_to_ros'
ROS_TO_ANDROID_MIC_TOPIC = '/android_audio'

FSM_LOG_MESSAGES = [TapGameLog.CHECK_IN, TapGameLog.GAME_START_PRESSED, TapGameLog.INIT_ROUND_DONE,
                    TapGameLog.START_ROUND_DONE, TapGameLog.ROBOT_RING_IN,
                    TapGameLog.PLAYER_RING_IN, TapGameLog.START_PRONUNCIATION_PANEL_DONE,
                    TapGameLog.END_PRONUNCIATION_PANEL_DONE, TapGameLog.SHOW_RESULTS_DONE,
                    TapGameLog.RESET_NEXT_ROUND_DONE, TapGameLog.SHOW_GAME_END_DONE]

class TapGameFSM: # pylint: disable=no-member
    """
    Each class should have a docstring describing what it does
    """

    round_index = 1
    max_rounds = 7

    student_model = StudentModel()
    recorder = TapGameAudioRecorder()
    current_round_word = ""

    game_commander = None
    log_listener = None

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

    def on_init_first_round(self):
        """
        Called when the game registers with the controller
        Should send msg to Unity game telling it the word to load for the first round
        """
        print("got to init_first round!")
        self.current_round_word = self.student_model.get_next_best_word()
        self.send_cmd(TapGameCommand.INIT_ROUND, self.current_round_word)

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
        self.send_cmd(TapGameCommand.START_ROUND)

    def on_robot_ring_in(self):
        """
        Called when the game registers that the robot 'buzzed in'
        Should send msg to Unity game telling it to load robot pronunciation screen
        """
        print('got to robot ring in cb')
        self.handle_SHOW_RESULTS()

    def on_player_ring_in(self):
        """
        Called when the human player has tapped their buzzer to ring in
        Should send msg to Unity game telling it to load the pronunciation screen
        And also start recording from the phone for 5 seconds + writing to wav
        """
        print('got to player ring in cb')

        # Initializes a new audio recorder object if one hasn't been created
        if self.recorder == None:
            self.recorder = TapGameAudioRecorder()

        #SEND SHOW_PRONUNCIATION_PAGE MSG
        self.recorder.startRecording()
        time.sleep(5)
        self.recorder.stopRecording()

        ##Evaluates the action message

        ## If given a word to evaluate and done recording send the information to speechace
        if self.current_round_word and self.recorder.has_recorded % 2 == 0 and self.recorder.has_recorded != 0:
            audioFile = "audioFile.wav"
            word_score_list = self.recorder.speechace(audioFile, self.current_round_word)
            print("WORD SCORE LIST")
            print(word_score_list)
            self.player_pronounce_eval()
        else:
            print('THIS SHOULD NEVER HAPPEN')

        #record

        # where my wav

        #move along

        #self.on_player_pronounce_eval()

    def on_player_pronounce_eval(self):
        """
        Called after the human player has pronounced their buzzer to ring in
        send wav from previous step to speech ace, get results, update model, and
        send message to game to display results
        """
        print('got to player pronounce eval cb')
        # Get the actual results here
        # TODO: the [1] is only for fully correct answers!!!!
        means, variances = self.student_model.train_and_compute_posterior([self.current_round_word],
                                                                          [1])

        print(self.student_model.curriculum)
        print(means)
        print(variances)

        # TODO Send message to Game to show results for three seconds, sleep, + handle round_end
        self.handle_round_end()

    def on_robot_pronounce_eval(self):
        """
        Called after the robot has 'pronounced' a word. Should send mesage to Game telling it
        to show results
        handle_round_end() to transition to next round
        """

        #TODO Send message to Game to show results for three seconds, sleep, + handle round_end
        self.handle_round_end()

    def on_round_reset(self):
        """
        Called after finishing a round in the game, but we have not hit
        'max_rounds" yet
        Should increment round index, and send cmd to game to reset for the next round
        """
        print('got to round reset')
        self.round_index += 1
        self.send_cmd(TapGameCommand.RESET_NEXT_ROUND)



    def on_game_finished(self):
        """
        Called when we have completed 'max_rounds' rounds in a game.
        Sends msg to the Unity game to load the game end screen
        """
        print('got to game finished')
        self.send_cmd(TapGameCommand.SHOW_GAME_END)



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

            if data.message == TapGameLog.RESET_NEXT_ROUND_DONE:
                print('Done Resetting Round!')
                self.current_round_word = self.student_model.get_next_best_word()
                self.send_cmd(TapGameCommand.INIT_ROUND, self.current_round_word)

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
        self.log_listener = rospy.Subscriber(TAP_GAME_TO_ROS_TOPIC, TapGameLog,
                                             self.on_log_received)
        

    def start_cmd_publisher(self):
        """
        Starts up the command publisher node
        """
        print('Pub Node started')
        self.game_commander = rospy.Publisher(ROS_TO_TAP_GAME_TOPIC, TapGameCommand, queue_size=10)
        rate = rospy.Rate(10)  # spin at 10 Hz
        rate.sleep()  # sleep to wait for subscribers
        #rospy.spin()


    def send_cmd(self, command, *args):
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
            msg.params = json.dumps(args[0]) #assume the
        self.game_commander.publish(msg)
        rospy.loginfo(msg)


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
