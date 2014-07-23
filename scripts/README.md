# Bag to AVI hack

_Note: this is a hack to help export segments of video (including sound) from ros bags from the command line.  It was a 30-minute hack to support my thesis / nsf study and hasn't been tested for robustness, yet._

## Dependencies

- you will probably need to install avconv (**I don't remember exactly what the package is for this... I'm recalling lib-avtools from memory**)
 
        sudo apt-get install lib-avtools

- python imports:

        import json
        import sys
        import rospy
        from subprocess import call
        from ros import rosbag
        from PIL import Image, ImageDraw, ImageFont
        import os, errno


## installation

This should be installed somewhere in your active ros workspace. (you will probably need to do a catkin source setup to include this in your rospath)... 

    # assuming workspace is in ~/projects/catkin_ws/
    cd ~/projects 
    git clone https://github.com/davidnunez/nsf_data_processing.git
    source ~/projects/catkin_ws/devel/setup.bash

## Usage:

    # - processing bag file: test.bag
    # - creates a subdirectory called "test" with:
    # - images/raw images/annotated	test.avi  test.mp3  test_nosound.avi
    
    # export complete movie:
    rosrun nsf_data_processing export.sh test
    
    # export movie from timestamp 1404056566 to end
    rosrun nsf_data_processing export.sh test 1404056566

    # export movie from timestamp 1404056566 to 1404056595
    rosrun nsf_data_processing export.sh test 1404056566 1404056595



## Usage Notes

- there is no ".bag" in the filename parameter
- the audio and video topic are hardcoded in extract_mp3.py -- you'll probably want to change this:

	- audio: `audio_common_msgs/AudioData`
	- video: `/usb_cam2/image_raw/compressed`
	
- export.sh will spit out an _annotated_ avi (i.e. timestamp and intent messages superimposed on the frames). The fastest way to change to an export w/o the annotation is by changing the word "annotated" to "raw" in the export.sh script
- timestamps are absolute as recorded by the ROS clock and are only the "whole seconds" portion of the timestamp (i.e. ROS stamp.secs)
- this may be a useful snippet

        # convert an avi to mp4 starting from time 00:30 for 25 seconds 
        # (the order of the parameters is important)
        avconv -ss 00:00:30 -i test.avi -t 25 -c:v libx264 -crf 23 test-clipped.mp4

