echo $1, $2, $3, $4

#check $1
list="p000 p100 p001 p002 p003 p004 p005 p006 p007 p008 p009 p010 p011 p012 p013 p014 p015 p016 p101 p102 p103 p104 p105 p106 p107 p108 p109 p110 p111 p112 p113 p114 p115 p116 p117 p118 p119 p120 p121 p122 p123 p124 p125 p126 p127 p128 p201 p202 p203 p204 p205 p206 p207 p208 p209 p210 p211 p212 p213 p214 p215 p216 p217 p218 p219 p220 p221 p222 p223 p224"

if ! [[ $list =~ (^| )$1($| ) ]]; then
  echo "error: participant_id [$1] does not exist"
  echo "Usage: ./startStudy.sh <participant_id> <session_#> <experimenter_name> <opening/story1/listen1/story2/listen2/closing>"
  echo "./startStudy.sh p000 1 haewon opening"
  exit
fi

#check $2
list='1 2 3 4 5 6 7 8'
if ! [[ $list =~ (^| )$2($| ) ]]; then
  echo "error: session [$2] does not exist"
  echo "Usage: ./startStudy.sh <participant_id> <session_#> <experimenter_name> <opening/story1/listen1/story2/listen2/closing>"
  echo "./startStudy.sh p000 1 haewon opening"
  exit
fi

#check $3
list='randi sooyeon haewon david sam ishaan nikhita pedro huili'
if ! [[ $list =~ (^| )$3($| ) ]]; then
  echo "error: experimenter [$3] does not exist"
  echo "Usage: ./startStudy.sh <participant_id> <session_#> <experimenter_name> <opening/story1/listen1/story2/listen2/closing>"
  echo "./startStudy.sh p000 1 haewon opening"
  exit
fi

#check #4
list='opening story1 story2 listen1 listen2 closing'
if ! [[ $list =~ (^| )$4($| ) ]]; then
  echo "error: entry point [$4] does not exist"
  echo "Usage: ./startStudy.sh <participant_id> <session_#> <experimenter_name> <opening/story1/listen1/story2/listen2/closing>"
  echo "./startStudy.sh p000 1 haewon opening"
  exit
fi

pg=$4
if [ "$4" == "story1" ] || [ "$4" == "story2" ]; then 
  LOG="./log/$1_s$2_$4.txt"
  if [[ -f $LOG ]]; then
    echo
    echo -n "Previous log file found:$LOG. Do you wish to continue [y/n]?"
    read -n 1 cont
    echo

    if [[ "$cont" == "y" ]]; then
      #read last line of log and append to $4
      pg=`tail -1 $LOG | head -1`
      pg=$4:$pg
      echo $pg
    fi
  fi
fi

#check #5
if [ "$5" != "" ] && [ "$5" != "last" ]; then
  echo "error: [$5] is not the right format. If it's the last session, input \"last\""
  echo "Usage: ./startStudy.sh <participant_id> <session_#> <experimenter_name> <opening/story1/listen1/story2/listen2/closing> <last>"
  echo "./startStudy.sh p000 7 haewon opening last"
  exit
fi
#if [[ $5 == "last" ]]; then

mkdir -p log
mkdir -p rosbag


gnome-terminal --geometry 24x12+0+0 --title ">>>Storyteller Study MAIN<<<" -e "./scripts/runFSM.sh $1 $2 $3 $pg $5"

./scripts/rosbag_record.sh $1 $2 $4

