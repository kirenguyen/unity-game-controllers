##USAGE: python -m start_main_controller.py

from TapGameController import TapGameFSM
from unity_game_msgs.msg import TapGameCommand
import rospy

import warnings

def fxn():
    warnings.warn("deprecated", DeprecationWarning)

with warnings.catch_warnings():
    warnings.simplefilter("ignore")
    fxn()

def main():

    my_FSM = TapGameFSM.TapGameFSM()
    my_FSM.start_log_listener()
    print('nodes started!')
    rospy.spin()

    ##
main()