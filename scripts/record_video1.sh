ffmpeg -y -f avfoundation -s 640x480 -r 30 -i "0:0" \
	-vf "drawbox=y=0: color=black@1.0: width=iw:height=40: t=20, \
  drawtext=fontfile=/Library/Fonts/Arial.ttf:  \
	text='%{frame_num}  %{localtime}  %{pts}': fontcolor=white: x=7: y=15: box=1: boxcolor=black@1.0" \
  videos/p01_huili_vid1_2017-11-28-12-27-50.mp4
