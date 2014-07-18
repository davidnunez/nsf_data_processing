#!/bin/bash

rosrun nsf_data_processing extract_mp3.py $1 $2 $3

MP3="$1/$1.mp3"
AVI="$1/$1.avi"
AVI_NOSOUND="./$1/$1_nosound.avi"

avconv -r 41040/1369 -i "$1/images/annotated/frame%010d.jpg" "$AVI_NOSOUND"
avconv -i "$MP3" -i "$AVI_NOSOUND" -acodec copy -vcodec copy -map 0 -map 1:v "$AVI"
