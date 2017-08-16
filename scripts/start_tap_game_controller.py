##USAGE: python -m start_main_controller.py

from TapGameController import TapGameFSM
from unity_game_msgs.msg import TapGameCommand
import rospy
import time
import signal
import sys


import warnings

def fxn():
    warnings.warn("deprecated", DeprecationWarning)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")


def signal_handler(signal, frame):
    print('Closing!')
    sys.exit()

def main():

    my_FSM = TapGameFSM.TapGameFSM()
    my_FSM.ros_node_mgr.start_log_listener(my_FSM.on_log_received)
    print('nodes started!')
    signal.signal(signal.SIGINT, signal_handler)
    #rospy.spin()


    while(True):
        try:
           my_FSM.student_model.plot_curricular_distro()
           my_FSM.student_model.fig.canvas.flush_events()
           time.sleep(1)
        except KeyboardInterrupt: 
            print('Closing!')
            sys.exit()
    

    ##

fxn()
main()