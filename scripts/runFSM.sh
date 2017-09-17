#!/bin/bash
echo -e "\033[1m\033[42mCTRL-C here to stop all\033[0m"
echo
echo ">>Quick Troubleshoot<<"
echo
echo "to restart speech rec:"
echo -e "\033[1m\033[36m./restart_speech.sh\033[0m"
echo
echo "to restart backchannel:"
echo -e "\033[1m\033[36m./restart_backchannel.py\033[0m"

#gnome-terminal --geometry 93x31+0+900 --title "Speech Recognition" -e "./tega/cognition/node_pyaudio_google_asr.py" &
#xterm -geometry 45x20+300+0 -T "Speech Recognition" -e bash -c "./tega/cognition/node_pyaudio_google_asr.py" &
xterm -geometry 45x20+300+0 -T "Speech Recognition" -e bash -c "./tega/cognition/node_google_asr.py" &

xterm -geometry 45x20+1200+0 -T "Story Question FSM" -e bash -c ./demo/activity_storyQuestion.py &
xterm -geometry 45x20+900+0 -T "Storytell FSM" -e bash -c "./demo/activity_storytell.py '$1' '$2'" &
sleep 0.5s
xterm -geometry 45x20+600+0 -T "Main FSM" -e bash -c "./demo/storyteller2017.py '$1' '$2' '$3' '$4' '$5'" &
xterm -geometry 45x20+300+350 -T "Affect Recognition" -e bash -c $CATKIN_DIR/devel/lib/affect_processing/affect_processing &
xterm -geometry 45x20+600+350 -T "Affect Result" -e bash -c 'python tega/cognition/affect_interaction.py'
#xterm --title "Face Recognition" -e "python tega/cognition/face_recognition.py train=false"
#xterm --title "Speech Recognition" -e "python tega/cognition/node_google_asr.py"


