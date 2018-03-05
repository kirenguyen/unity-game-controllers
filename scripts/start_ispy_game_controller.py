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

def fxn():
    warnings.warn("deprecated", DeprecationWarning)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")




def usage():
    print('python3 -m scripts.start_ispy_game_controller [game round: practice or experiment] [participant id: pXX] [experimenter] [session number: sXX]')

def main():

    def signal_handler(signal, frame):
        print("closing in one second!")
        control.kill_received = True
        sys.exit()

    try:

        participant_id = sys.argv[1] # which participant
        experimenter = sys.argv[2] # experimenter
        session_number = sys.argv[3] # session number 

    except IndexError:
        usage()
        sys.exit()

    global control
    control = iSpyGameFSM.iSpyGameFSM(participant_id, experimenter, session_number)
    print("FSM Started!")
    thread.start_new_thread(control.ros_node_mgr.start_ispy_transition_listener, (control.on_ispy_state_info_received,))

    thread.start_new_thread(control.ros_node_mgr.start_ispy_log_listener, (control.on_ispy_log_received,))

    signal.signal(signal.SIGINT, signal_handler) # for keyboard interrupt

    # control.ros_node_mgr.send_ispy_cmd(34,game_round) #SET_GAME_SCENE = 34

    control.ros_node_mgr.start_ispy_action_listener(control.on_ispy_action_received)



    
if __name__ == "__main__":
    main()