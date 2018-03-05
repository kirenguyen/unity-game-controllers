xterm -bg grey -geometry 45x20+200+200 -T "ROS Bag" -e bash -c "rosbag play ../../ispy_rosbag/$1" &

sleep 1s
#xterm -bg grey -geometry 45x20+300+550 -T "Affect Recognition" -e bash -c ./tega_cam_affect_analysis &

sleep 1s
#xterm -bg grey -geometry 45x20+200+200 -T "USB Cam" -e "python3 extract_affdex_data.py"
python3 extract_affdex_data.py $1