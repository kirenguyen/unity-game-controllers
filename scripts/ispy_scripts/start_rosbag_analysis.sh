xterm -bg grey -geometry 45x20+200+200 -T "ROS Bag" -e bash -c "rosbag play  rosbag_inputs/$1" &

sleep 1s
xterm -bg grey -geometry 45x20+300+550 -T "Affect Recognition" -e bash -c ./tega_cam_affect_analysis &


sleep 1s #check the rostopic publishing rate
gnome-terminal --geometry 40x120+0+0 --title ">>>AFFDEX ROSTOPIC PUBLISHING RATE<<<" -x bash -c "rostopic hz /affdex_data"
sleep 1s
#xterm -bg grey -geometry 45x20+200+200 -T "USB Cam" -e "python3 extract_affdex_data.py"
python3 extract_affdex_data.py $1 $2 $3 # start the analysis

