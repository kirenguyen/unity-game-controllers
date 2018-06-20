#!/bin/bash
echo -e "\033[1m\033[42mhere to stop all\033[0m"
echo

sleep 0.5s
echo "I AM HERE!"

xterm -bg grey -geometry 45x20+200+200 -T "USB Cam" -e bash -c "roslaunch usb_cam usb_cam-test.launch" &

sleep 0.5s

xterm -bg grey -geometry 45x20+300+550 -T "Affect Recognition" -e bash -c ./../../devel/lib/tega_cam_affect_analysis//tega_cam_affect_analysis & 

sleep 0.5s

if [ "$4" = "tega" ]
then
echo "~~~~~~~~~~~~~~~~~~~~~~WE ARE USING TEGA~~~~~~~~~~~~~~~~~~~~~~~~~~~"
xterm -bg brown -geometry 45x20+300+550 -T "Speech Recognition" -e "python3 ../asr_google_cloud/src/ros_asr.py" &
elif [ "$4" = "jibo" ]
then
echo "~~~~~~~~~~~~~~~~~~~~~~WE ARE USING JIBO~~~~~~~~~~~~~~~~~~~~~~~~~~"
else
echo "~~~~~~~~~~~~~~~~~~~~~WE ARE NOT USING ROBOTS~~~~~~~~~~~~~~~~~~~~~"
fi

sleep 0.5s

#./scripts/start_recording_linux.sh $1 $2
python3 -m scripts.start_ispy_game_controller $1 $2 $3 $4



#xterm -geometry 120x40+200+0 -T "Main FSM" -e bash -c "python3 -m scripts.start_ispy_game_controller" $3