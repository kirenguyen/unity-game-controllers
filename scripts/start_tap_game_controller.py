##USAGE: python -m start_main_controller.py

from TapGameController import TapGameFSM
from unity_game_msgs.msg import TapGameCommand
import rospy
import time
import signal
import sys
import _thread as thread


def signal_handler(signal, frame):
    print('Closing!')
    sys.exit()

def main(argv):

    #TegaDemo(argv[0],int(argv[1]),argv[2],argv[3])
    print(argv)
    my_FSM = TapGameFSM.TapGameFSM(argv[1], argv[2], argv[3])
    thread.start_new_thread(my_FSM.ros_node_mgr.start_log_listener, (my_FSM.on_log_received,))
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




if __name__ == "__main__":
    if len(sys.argv) == 4:
        main(sys.argv)
    else:
        print(sys.argv)
        print("Usage: python -m scripts.start_tap_game_controller <p-ID> <experimenter_name> <experiment_phase>")
        exit()


