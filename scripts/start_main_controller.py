##USAGE: python -m start_main_controller.py

from TapGameController import TapGameFSM

def main():

    my_FSM = TapGameFSM.TapGameFSM()
    my_FSM.initRound()
    my_FSM.startRound()
    my_FSM.robotRingIn()
    my_FSM.evaluate_round()
    my_FSM.startRound()
    my_FSM.robotRingIn()
    my_FSM.evaluate_round()


##
main()