##USAGE: python -m start_main_controller.py

from iSpyGameController import iSpyGameFSM
from unity_game_msgs.msg import iSpyCommand
import rospy
import time
import signal
import sys
import _thread as thread # in Python3, the module is called _thread
from multiprocessing import Process

import warnings
import time


# COMMAND CONSTANTS
QUIT_GAME = 67



def fxn():
    warnings.warn("deprecated", DeprecationWarning)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")




def usage():
    print('python3 -m scripts.start_ispy_game_controller [participant id: pXX] [experimenter] [session number: sXX], ["jibo" or "tega" or "none"]')

def main():
    try:

        participant_id = sys.argv[1] # which participant
        experimenter = sys.argv[2] # experimenter
        session_number = sys.argv[3] # session number 
        use_jibo_or_tega = sys.argv[4] # which robot


    except IndexError:
        usage()
        sys.exit()

    global control
    control = iSpyGameFSM.iSpyGameFSM(participant_id, experimenter, session_number, use_jibo_or_tega)
    
    def signal_handler(signal, frame):
        print("closing iSpy Unity Game")
        control.ros_node_mgr.send_ispy_cmd(QUIT_GAME)
        print("closing in one second!")         
        control.kill_received = True
        sys.exit()



    print("FSM Started!")
    thread.start_new_thread(control.ros_node_mgr.start_ispy_transition_listener, (control.on_ispy_state_info_received,))

    thread.start_new_thread(control.ros_node_mgr.start_ispy_log_listener, (control.on_ispy_log_received,))


    signal.signal(signal.SIGINT, signal_handler) # for keyboard interrupt

    # control.ros_node_mgr.send_ispy_cmd(34,game_round) #SET_GAME_SCENE = 34

    control.ros_node_mgr.start_ispy_action_listener(control.on_ispy_action_received)


    
if __name__ == "__main__":
    main()
