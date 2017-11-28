ffmpeg -y -f video4linux2 -s 640x480 -r 30 -i "" \
   -ac 2   -vf drawtext="fontfile=/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf: \
text='%{frame_num}  %{localtime}  %{pts}': fontcolor=white: fontsize=24: box=1: boxcolor=black@0.5: \
boxborderw=5: x=(w-text_w)/2: y=0" \
  videos/p000_huili_vid1_2017-11-28-11-35-21.mp4
