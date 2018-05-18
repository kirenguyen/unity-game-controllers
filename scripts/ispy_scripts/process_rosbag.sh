#!/bin/bash



gnome-terminal --geometry 40x120+0+0 --title ">>>AFFDEX ROSTOPIC PUBLISHING RATE<<<" -x bash -c "rostopic hz /affdex_data"
sleep 1s #check the rostopic publishing rate

for i in `seq 1 4`;
do
	echo "process the next participant"
	python3 get_next_rosbag.py

	for i in $( ls rosbag_inputs/); do
    	echo item: $i
		rosbagName=$i

		jsonOut=`python3 get_interaction_file_name.py $rosbagName`
		sessNumber=$(echo $jsonOut | cut -f1 -d#)
		interFileName=$(echo $jsonOut | cut -f2 -d#)
		echo "json map outputs:"
		echo "$sessNumber"
		echo "$interFileName"
		echo "~~~"
		
		xterm -bg grey -geometry 45x20+200+200 -T "ROS Bag" -e bash -c "rosbag play  rosbag_inputs/$rosbagName" &

		sleep 1s
		xterm -bg grey -geometry 45x20+300+550 -T "Affect Recognition" -e bash -c "./tega_cam_affect_analysis" &

		sleep 1s
		#xterm -bg grey -geometry 45x20+200+200 -T "Affect Extraction" -e "python3 extract_affdex_data.py"
		gnome-terminal --geometry 40x120+0+0 --title "Affect Extraction" -e "python3 extract_affdex_data.py $rosbagName $sessNumber" &



		###################

		#rosbag play $rosbag_name
		echo "sleep for 3 seconds"
		sleep 3s
		echo "Right before while loop"
		while :
		do
			sleep 1s
			echo "check the status of rosbag play"
			out=`ps aux | grep "rosbag play" | grep $rosbagName`
			if [ -z "$out" ]; then
    			echo "$rosbag_name is done..."
    		break
			fi
		done

	# check affdex existence
	csvFile=`echo $rosbagName | sed  's/bag/csv/g'`
	if [ -f /affdex-outputs/$csvFile ]; then
    	echo "Affect File found!"
    	# delete rosbags
    	numLine=`wc -l < /affdex-outputs/$csvFile`
    	echo "number of lines in affdex" $numLine
    	rm rosbag_inputs/$rosbagName
	fi

done

done