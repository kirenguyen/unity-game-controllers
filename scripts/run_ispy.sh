#!/bin/bash
echo -e "\033[1m\033[42mhere to stop all\033[0m"
echo


gnome-terminal --geometry 93x31+0+900 --title "Speech Recognition" -e "python3 ../asr_google_cloud/src/ros_asr.py" 



sleep 0.5s
echo "I AM HERE!"
xterm -geometry 45x20+200+200 -T "USB Cam" -e bash -c "roslaunch usb_cam usb_cam-test.launch" &
xterm -geometry 45x20+300+550 -T "Affect Recognition" -e bash -c $CATKIN_DIR/devel/lib/tega_cam_affect_analysis/tega_cam_affect_analysis 


#./scripts/start_recording_linux.sh $1 $2
python3 -m scripts.start_ispy_game_controller $3 $1 $2
#xterm -geometry 120x40+200+0 -T "Main FSM" -e bash -c "python3 -m scripts.start_ispy_game_controller" $3 &
