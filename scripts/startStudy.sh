echo $1, $2, $3, $4

#check $1
list="p01 p02 p03 p04 p05 p06 p07 p08 p09 p10"

if ! [[ $list =~ (^| )$1($| ) ]]; then
  echo "error: participant_id [$1] does not exist"
  echo "Usage: ./startStudy.sh <participant_id> <experimenter_name>"
  echo "./startStudy.sh p000 sam"
  exit
fi

#check $2
list='1 2 3 4 5 6 7 8'
if ! [[ $list =~ (^| )$2($| ) ]]; then
  echo "error: session [$2] does not exist"
  echo "Usage: ./startStudy.sh <participant_id> <experimenter_name>"
  echo "./startStudy.sh p000 sam"
  exit
fi

mkdir -p log
mkdir -p rosbag


gnome-terminal --geometry 24x12+0+0 --title ">>>Tap Game Study MAIN<<<" -e "./scripts/runFSM.sh $1 $2 $3 $pg $5"

#./scripts/rosbag_record.sh $1 $2 $4

