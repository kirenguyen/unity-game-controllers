rosbagName="p106_huili_s02_2018-04-03-09-31-53.bag"
csvFile=`echo $rosbagName | sed  's/bag/csv/g'`
		echo $csvFile
		csvFile=`echo "affdex_$csvFile"`
		echo $csvFile
		if [ -f "affdex-outputs/$csvFile" ]; then
    		echo "Affect File found!"
    		# delete rosbags
    		numLine=`wc -l < affdex-outputs/$csvFile`
    		echo "number of lines in affdex" $numLine
    	else
    		echo "not found.."
    		
		fi