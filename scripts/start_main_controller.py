##USAGE: python -m start_main_controller.py

from TapGameController import TapGameFSM
from unity_game_msgs.msg import TapGameCommand
import rospy

def main():

    my_FSM = TapGameFSM.TapGameFSM()
    my_FSM.startLogListener()
    print('nodes started!')
    rospy.spin()
    
    ##
main()