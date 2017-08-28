##USAGE: python -m start_main_controller.py

from GameUtils.PronunciationUtils import PronunciationHandler

def signal_handler(signal, frame):
    print('Closing!')
    sys.exit()

def main():

    myPH = PronunciationHandler()    
    myPH.build_substitution_score_matrix()    

main()