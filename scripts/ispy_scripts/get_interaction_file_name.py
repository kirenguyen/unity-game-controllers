import json
import sys

remaining_rosbags=""
with open('study_data_map.json') as json_data:
	remaining_rosbags = json.load(json_data)
rosbag_name = sys.argv[1]
p_id = rosbag_name.split("_")[0]

s_number = ""
inter_file = ""
for session in remaining_rosbags[p_id]:
	for epi in remaining_rosbags[p_id][session]:
		if epi[0] in rosbag_name:
			s_number = session
			inter_file = epi[1]+".csv"
			break
print(s_number+"#"+inter_file)
