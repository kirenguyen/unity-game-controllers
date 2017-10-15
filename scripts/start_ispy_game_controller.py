##USAGE: python -m start_main_controller.py

from iSpyGameController import iSpyGameFSM
from unity_game_msgs.msg import iSpyCommand
import rospy
import time
import signal
import sys
import _thread as thread # in Python3, the module is called _thread


import warnings

def fxn():
    warnings.warn("deprecated", DeprecationWarning)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")


def signal_handler(signal, frame):
    print('Closing!')
    sys.exit()

def main():
    control = iSpyGameFSM.iSpyGameFSM()
    print("FSM Started!")
    thread.start_new_thread(control.ros_node_mgr.start_ispy_transition_listener, (control.on_ispy_state_info_received,))
    
    thread.start_new_thread(control.ros_node_mgr.start_ispy_log_listener, (control.on_ispy_log_received,))
    
    control.ros_node_mgr.start_ispy_action_listener((control.on_ispy_action_received,))
    
main()