ffmpeg -y -f video4linux2 -s 640x480 -r 30 -i "/dev/video0" \
    -ac 2   -vf drawtext="fontfile=/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf: \
text='%{frame_num}  %{localtime}  %{pts}': fontcolor=white: fontsize=24: box=1: boxcolor=black@0.5: \
boxborderw=5: x=(w-text_w)/2: y=0" \
  videos/p00_1_vid2_2017-11-24-20-29-46.mp4
