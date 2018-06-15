"""
This is the main class that manages the creation / parsing of ROS Node Communication
"""
# -*- coding: utf-8 -*-
# pylint: disable=import-error, invalid-names

import time
import json
from GameUtils import GlobalSettings
from .RobotBehaviorList.JiboBehaviors import JiboBehaviors
from .RobotBehaviorList.TegaBehaviors import TegaBehaviors

from affdex_msgs.msg import AffdexFrameInfo # please download affdex ros msg from PRG's git. if you don't have access to this repo, just ask Huili
from affdex_msgs.msg import Vec2 # please download affdex ros msg from PRG's git



if GlobalSettings.USE_ROS:
    import rospy
    from std_msgs.msg import Header  # standard ROS msg header
    from unity_game_msgs.msg import iSpyCommand
    from unity_game_msgs.msg import iSpyAction
    from unity_game_msgs.msg import iSpyChildRobotInteraction
    from r1d1_msgs.msg import TegaAction
    from r1d1_msgs.msg import TegaState
    from r1d1_msgs.msg import Vec3
    from jibo_msgs.msg import JiboAction
    from std_msgs.msg import String
    from asr_google_cloud.msg import AsrResult
    from asr_google_cloud.msg import AsrCommand
    from jibo_msgs.msg import JiboAsrCommand
    from jibo_msgs.msg import JiboAsrResult
    from jibo_msgs.msg import JiboState

else:
    TapGameLog = GlobalSettings.iSpyAction  # Mock object, used for testing in non-ROS environments
    TapGameCommand = GlobalSettings.iSpyCommand
    TegaAction = GlobalSettings.TegaAction
    JiboAction = GlobalSettings.JiboAction

#ROSCORE_TO_ISPY_GAME_TOPIC = '/ispy_game_from_ros'
#ISPY_GAME_TO_ROSCORE_TOPIC = '/ispy_game_to_ros'

#set up ros topics for ispy game
ROS_TO_ISPY_GAME_TOPIC = 'ispy_cmd_topic'
ISPY_GAME_TO_ROS_ACTION_TOPIC = 'ispy_action_topic'
ISPY_GAME_TO_ROS_TRANSITION_TOPIC = 'ispy_transition_state_topic'
ISPY_GAME_TO_ROS_LOG_TOPIC = 'ispy_log_topic'
ROS_TO_ANDROID_MIC_TOPIC = 'android_audio'
DATA_CHILD_ROBOT_INTERACTION = 'data_child_robot_interaction'


ROSCORE_TO_JIBO_TOPIC = '/jibo'
ROSCORE_TO_TEGA_TOPIC = '/tega'

# COMMAND CONSTANTS
RESET = 0
SHOW_PRONOUNCIATION_PANEL = 1
SHOW_OBJECT_DESCR_PANEL = 2
ROBOT_EXPERT_ROLE = 3
SEND_PRONOUNCIATION_ACCURACY_TO_UNITY = 10
SEND_TASKS_TO_UNITY = 20
GAME_FINISHED = 99
ROBOT_VIRTUAL_ACTIONS = 30
BUTTON_DISABLED=31
TASK_COMPLETED=32
WHOSE_TURN = 33
SET_GAME_SCENE = 34
SPEAK = 35
VALID_ISPY_COMMANDS = [SET_GAME_SCENE, WHOSE_TURN,TASK_COMPLETED,
                       ROBOT_VIRTUAL_ACTIONS, RESET, SHOW_PRONOUNCIATION_PANEL, 
                       SHOW_PRONOUNCIATION_PANEL, SEND_PRONOUNCIATION_ACCURACY_TO_UNITY, 
                       SEND_TASKS_TO_UNITY, GAME_FINISHED,BUTTON_DISABLED,
                       SPEAK]



class ROSNodeMgr:  # pylint: disable=no-member, too-many-instance-attributes
    """
    A Manager for the ROS Communication in the Tap Game. Contains nodes for interacting
    w the Unity "View" and also the Robot
    """

    game_commander = None
    robot_commander = None
    log_listener = None

    message_received = None 



    def __init__(self):
        pass

    # check...
    def init_ros_node(self): #pylint: disable=no-self-use
        """
        Start up the game connection ROS node
        """
        print("rospy init node")
        rospy.init_node('ispy_ROS_receiver', anonymous = True)
        

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



    def send_robot_cmd(self, command, *args):
        """
        send a command to the robot (action space or "cosmetic")
        This function maps actions from the ActionSpace into actual ROS Msgs
        """
        
    
        if self.robot_commander is None:
            self.start_robot_publisher()
            time.sleep(.5)

        # choose which platform
        if GlobalSettings.USE_TEGA:
            msg = TegaBehaviors.get_msg_from_behavior(command, args)
        else:
            msg = JiboBehaviors.get_msg_from_behavior(command, args)

        # add header
        self.robot_commander.publish(msg)  # would be nice to guarantee message performance here
        #rospy.loginfo(msg)

    ## for iSpy FSM ################
    def send_ispy_cmd(self, command, *args):
        """
        send a iSpyCommand to unity game
        Args are optional parameters.
        """

        print("in def send_ispy_cmd")
        msg = iSpyCommand()
        # add header
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()

        # Bool for if the word is pronounced completely correctly
        # Used to reuse the origText variable to try pronouncing again
        perfect_word = True

        #TODO: may need to convert args to json before passing it to msg.params
        #if command is SEND_PRONOUNCIATION_ACCURACY_TO_UNITY, then convert accuracy result to JSOn message first
        #use function in Util.py

        if command in VALID_ISPY_COMMANDS:
            # fill in command and any params:
            msg.command = command
            if len(args) > 0:


                if command == SEND_PRONOUNCIATION_ACCURACY_TO_UNITY:
                    msg.properties = json.dumps(args[0])

                elif command == SEND_TASKS_TO_UNITY:
                    converted_result = json.dumps(args[0])
                    msg.properties = converted_result

                elif command == ROBOT_VIRTUAL_ACTIONS:
                    
                    robot_action = json.dumps(args[0])
                    msg.properties = robot_action

                elif command == BUTTON_DISABLED:
                    msg.properties = json.dumps(args[0])

                elif command == WHOSE_TURN:
                    msg.properties = json.dumps(args[0])

                elif command == SET_GAME_SCENE:
                    gameScene = json.dumps(args[0])
                    msg.properties = gameScene

                elif command == SPEAK:
                    msg.properties = json.dumps(args[0])

            # send message to tablet game
            if self.game_commander is None:
                self.start_ispy_cmd_publisher()


            if command == SEND_TASKS_TO_UNITY or command == SET_GAME_SCENE:
                counter = 0
                self.message_received = False
                # Keep sending the message until hearing that it was received
                while self.message_received == False:
                    print("***send ros: "+str(counter)+"**"+str(command))
                    
                    self.game_commander.publish(msg)
                    time.sleep(0.5)
                    counter = counter + 1
            else:
                self.game_commander.publish(msg)
            self.message_received = False

            #rospy.loginfo(msg)

        else:
            print ("Not a valid command")


    def start_ispy_action_listener(self,on_ispy_action_received):
        """
        Start up the ispy action Subscriber node
        """
        self.ispy_to_ros__action_subs = rospy.Subscriber(ISPY_GAME_TO_ROS_ACTION_TOPIC, iSpyAction, on_ispy_action_received)   
        print("action listener")
        rospy.spin()
        

    def start_ispy_transition_listener(self,on_ispy_state_info_received):
        """
        Start up the ispy state Subscriber node
        """
        time.sleep(.2)
        self.ispy_to_ros_trans_subs = rospy.Subscriber(ISPY_GAME_TO_ROS_TRANSITION_TOPIC, String, on_ispy_state_info_received)   
        print("transition listener")
        rospy.spin()
        #rospy.spin()

    def start_ispy_log_listener(self,on_ispy_log_received):
        self.ispy_to_ros_log_subs = rospy.Subscriber(ISPY_GAME_TO_ROS_LOG_TOPIC, String, on_ispy_log_received)
        print("log listener")
        rospy.spin()
    
    def start_ispy_cmd_publisher(self):
        """
        Starts up the ispy command publisher node,
        which allows iSpy controller sends cmd messages over ROS to the unity game
        """
        print('Pub Node started')
        self.game_commander = rospy.Publisher(ROS_TO_ISPY_GAME_TOPIC, iSpyCommand, queue_size=10)
        rate = rospy.Rate(10)  # spin at 10 Hz
        rate.sleep()  # sleep to wait for subscribers
        # rospy.spin()

    def start_tega_state_listener(self,on_tega_state_received):
        print('start tega state listener')
        sub_affdex = rospy.Subscriber('tega_state',TegaState,on_tega_state_received)

    def start_affdex_listener(self,on_affdex_data_received):
        '''
        start affdex listener, which receives affdex data 
        '''
        print('affdex listener starts..')
        sub_affdex = rospy.Subscriber('affdex_data', AffdexFrameInfo, on_affdex_data_received) # affdex data


    def start_tega_asr(self, on_tega_new_asr_result):
        if GlobalSettings.USE_TEGA:
            self.sub_asr_result = rospy.Subscriber('asr_result', AsrResult, on_tega_new_asr_result)
            self.pub_asr_command = rospy.Publisher('asr_command', AsrCommand, queue_size=1)
        else:
            self.sub_asr_result = rospy.Subscriber('asr_result', JiboAsrResult, on_tega_new_asr_result)
            self.pub_asr_command = rospy.Publisher('asr_command', JiboAsrCommand, queue_size=1)

    def start_asr_listening(self):
        '''
        start asr's listening
        '''
        #TODO: implement this properly for Jibo/Tega switch
        if GlobalSettings.USE_TEGA:
            msg = AsrCommand()
            msg.command = AsrCommand.START_FINAL  # start final
        else:
            msg = JiboAsrCommand()
            msg.header - Header()
            msg.header.stamp = rospy.Time.now()
            msg.command = JiboAsrCommand.START
            msg.heyjibo = False
            msg.incremental = True
            msg.continuous = True
            msg.rule = rule


        self.pub_asr_command.publish(msg)

    def stop_asr_listening(self):
        '''
        stop asr's listening and get the results
        '''

        #TODO: implement this properly for Jibo/Tega switch
        if GlobalSettings.USE_TEGA:
            msg = AsrCommand()
            msg.command = AsrCommand.STOP_FINAL # stop final for the asr result
        else:
            msg = JiboAsrCommand()
            msg.header - Header()
            msg.header.stamp = rospy.Time.now()
            msg.command = JiboAsrCommand.STOP
            msg.heyjibo = False
            msg.incremental = True
            msg.continuous = True
            msg.rule = rule


        self.pub_asr_command.publish(msg)
        print("stop asr listening....")


    def start_child_only_interaction_pub_sub(self, on_interaction_data):
        print("start child only interaction publisher/subscriber")
        self.pub_child_only_interaction = rospy.Publisher()
        self.sub_child_only_interaction = rospy.Subscriber()


    def start_child_robot_interaction_pub_sub(self, on_interaction_data):
        print("start child robot interaction publisher/subscriber")
        self.pub_child_robot_interaction = rospy.Publisher(DATA_CHILD_ROBOT_INTERACTION,iSpyChildRobotInteraction,queue_size=1)
        self.sub_child_robot_interacftion = rospy.Subscriber(DATA_CHILD_ROBOT_INTERACTION,iSpyChildRobotInteraction, on_interaction_data)
