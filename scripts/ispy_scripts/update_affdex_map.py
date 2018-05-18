import json
import sys
import os
# check affdex data files in affdex-outputs
affdex_dir = "affdex-outputs"
interaction_dir = "interaction_outputs/"
aff_map_file = "affdex_data_map.json"
study_map_file = 'study_data_map.json'
aff_json=None

if os.path.exists(aff_map_file):
	aff_json = json.load(open(aff_map_file,'r'))
else:
	aff_json = json.load(open(study_map_file,'r'))


for affdex_file in os.listdir(affdex_dir):
	print("\n affdex file: {}".format(affdex_file))
	p_id = affdex_file.split('_')[1]
	session_num = affdex_file.split('_')[3].replace("0","")
	bag_name = affdex_file.replace(".csv",'').strip("affdex_")
	interaction_name = affdex_file.replace('affdex','interaction_rosbag')
	tmp = aff_json[p_id][session_num]
	for inst in tmp:

		if bag_name == inst[0]:
			if len(inst) == 2:
				print("updating: {}".format(affdex_file))
				if os.path.exists(interaction_dir+interaction_name):
					inst.append({'affdex':affdex_file,'interaction_rosbag':interaction_name})
				else:
					print("cannot find the interaction rosbag log.")
					inst.append({'affdex':affdex_file,'interaction_rosbag':""})
			elif len(inst) == 3:
				if inst[2]['affdex'] != affdex_file:
					print("inconsistency! {}".format(affdex_file))
			break

print("finish updating the map")
json.dump(aff_json,open(aff_map_file,'w'))