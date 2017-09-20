#!/bin/bash
echo $1, $2 $3

#check $1
list="p01 p02 p03 p04 p05 p06 p07 p08 p09 p10"

if ! [[ $list =~ (^| )$1($| ) ]]; then
  echo "error: participant_id [$1] does not exist"
  echo "Usage: ./startStudy.sh <participant_id> <experimenter_name> <study_phase>"
  echo "./startStudy.sh p000 sam experiment"
  exit
fi

#check $2
list='sam'
if ! [[ $list =~ (^| )$2($| ) ]]; then
  echo "error: experimenter [$2] does not exist"
  echo "Usage: ./startStudy.sh <participant_id> <experimenter_name> <study_phase>"
  echo "./startStudy.sh p000 sam experiment"
  exit
fi

#check $3
list='practice experiment posttest'
if ! [[ $list =~ (^| )$3($| ) ]]; then
  echo "error: study phase [$3] does not exist"
  echo "Usage: ./startStudy.sh <participant_id> <experimenter_name> <study_phase>"
  echo "./startStudy.sh p000 sam experiment"
  exit
fi

mkdir -p log
mkdir -p rosbag


gnome-terminal --geometry 240x120+0+0 --title ">>>Tap Game Study MAIN<<<" -e "./scripts/runFSM.sh $1 $2 $3 $pg $5"

#./scripts/rosbag_record.sh $1 $2 $4

