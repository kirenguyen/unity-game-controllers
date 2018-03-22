#!/bin/bash

source ~/.bashrc

echo $1, $2, $3, $4

#check $1
list="p00 p01 p02 p03 p04 p05 p06 p07 p08 p09 p10 p11 p12 p13 p14 p15 p16 p17 p18 p19 p20 p21 p22 p23 p24 p25 p26 p27 p28 p29 p30 p31 p32 p33 p34 p35 p36 p37 p38 p39 p40 p41 p42 p43 p44 p45 p46 p47 p48 p49 p50 p51 p52 p53 p54 p55 p56 p57 p58 p59 p60 p61 p62 p100 p101 p102 p103 p104 p105 p106"

if ! [[ $list =~ (^| )$1($| ) ]]; then
  echo "error: participant_id [$1] does not exist"
  echo "Usage: ./startStudy.sh <participant_id> <experimenter_name> <study_phase> <session number>"
  echo "./startStudy.sh p000 huili [practice, s01, s02]"
  exit
fi

#check $2
list="huili safinah calvin maggie soojung max"
if ! [[ $list =~ (^| )$2($| ) ]]; then
  echo "error: experimenter [$2] does not exist"
  echo "Usage: ./startStudy.sh <participant_id> <experimenter_name> <study_phase> <session number>"
  echo "./startStudy.sh p01 huili [practice, s01, s02]"
  exit
fi

#check $3
list="practice s01 s02"
if ! [[ $list =~ (^| )$3($| ) ]]; then
  echo "error: session number [$3] does not exist"
  echo "Usage: ./startStudy.sh <participant_id> <experimenter_name> <study_phase> <session number>"
  echo "./startStudy.sh p01 huili [practice, s01, s02]"
  exit
fi


mkdir -p log
mkdir -p rosbag


gnome-terminal --geometry 40x120+0+0 --title ">>>iSpy Game Study MAIN<<<" -e "./scripts/run_ispy.sh $1 $2 $3" 

echo $3
if [ $3 != 'practice' ]; then
  echo "start rosbag recording...."
  ./scripts/rosbag_record.sh $1 $2 $3
fi