#!/bin/bash

#rosrun nsf_data_processing extract_mp3.py $1 $2 $3

MP3="$1/$1.mp3"
AAC="$1/$1.aac"
AVI="$1/$1.mp4"
AVI_NOSOUND="./$1/$1_nosound.mp4"


#avconv -r 41040/1369 -b 65536k -i "$1/images/annotated/frame%010d.jpg" "$AVI_NOSOUND"
#avconv -r 20520/1369 -i "$1/images/raw/frame%010d.jpg" -b 32768k -vsync cfr "$AVI_NOSOUND"
avconv -r 41040/1369 -i "$1/images/raw/frame%010d.jpg" -b 65536k -vsync cfr "$AVI_NOSOUND"


#avconv -i "$MP3" -i "$AVI_NOSOUND" -acodec copy -map 0:a -vcodec copy -map 1:v "$AVI"
avconv -i "$MP3" -strict experimental "$AAC"
avconv -i "$AVI_NOSOUND" -i "$AAC" -strict experimental -c:v copy -c:a aac "$AVI"
