#!/usr/bin/env python

# From a rosbag, this script outputs an mp3 file from an audio topic 
# and a directory of images from image topic. It also outputs another
# directory of the same images with time stamp and annotation information
# overlaid.
#
# Code inspired by https://github.com/ros-drivers/audio_common/issues/1
# Usage:
# $ rosrun data_processing extract_mp3.py sample.bag
#
# Run from the directory of the bag file.
#
# # This generated mp3 has issues with its vbr header.  This can be fixed using utilities:
# $ vbrfix -always -makevbr sample.mp3
# $ mp3val sample.mp3 -f -t

import json
import sys
import rospy
from subprocess import call
from ros import rosbag
from PIL import Image, ImageDraw, ImageFont
import os, errno

audio_topic = "/audio"
image_topic = '/usb_robotcam/image_raw/compressed'

def mkdir_p(path):
        try:
                os.makedirs(path)
        except OSError as exc:
                if exc.errno == errno.EEXIST and os.path.isdir(path):
                        pass
                else:
                        raise


def extract_audio(bag_path, topic_name, mp3_path, start=None, stop=None):
	print 'Opening bag:' +  bag_path
	# print 'start: %d, stop: %d'%(start, stop)
	bag = rosbag.Bag(bag_path + ".bag")
	mp3_file = open(bag_path + "/" + bag_path + ".mp3", 'w')
	print 'Reading audio messages and saving to mp3 file'
	audio_msg_count = 0
        video_msg_count = 0
        display_message_1 = "..."
        display_message_2 = "..."
        display_message_3 = "..."
        fingerDown = False
	for topic, msg, stamp in bag.read_messages():
		if (stop != None and stop < stamp.secs):
			break
		if (start != None and start > stamp.secs):
			continue
		#print topic
                if msg._type == 'audio_common_msgs/AudioData':
			audio_msg_count += 1
			mp3_file.write(''.join(msg.data))
                if topic == image_topic:
                        img_filename = 'frame%010d.jpg'%(video_msg_count)
                        # print img_filename
                        image_file = open(bag_path + "/images/raw/" + img_filename, 'w')
                        image_file.write(''.join(msg.data))
                        image_file.close()
                        image = Image.open(bag_path + "/images/raw/" + img_filename)
                        draw = ImageDraw.Draw(image)
                        # font = ImageFont.truetype("arial.ttf", 20, encoding = "unic")
                        draw.text( (10,10), '%d.%d'%(stamp.secs, stamp.nsecs) + ": " + display_message_1, fill = '#ffffff') #, font=font)
                        draw.text( (10,20), display_message_2, fill = '#ffffff')
                        if (fingerDown):
                                draw.text ((10, 30), "fingerDown", fill = "#ff0000")
                        image.save(bag_path + "/images/annotated/" + img_filename)

                        video_msg_count +=1
                if topic == '/intent_to_ros':
                        if not 'finger' in msg.data:
                                try:
                                        if 'scene' in msg.data:
                                                j = json.loads(msg.data)
                                                display_message_1 = j['value']
                                        else:
                                                j = json.loads(msg.data)
                                                display_message_2 = j['value']
                                except:
                                        display_message_2 = msg.data
                        if 'fingerDown' in msg.data:
                                fingerDown = True
                        if 'fingerUp' in msg.data:
                                fingerDown = False
        bag.close()
	mp3_file.close()
	print 'Done.'
        print '%d audio messages written to %s'%(audio_msg_count, mp3_path)
        print '%d images saved'%(video_msg_count)


if __name__ == '__main__':
        mkdir_p(sys.argv[1])
        mkdir_p(sys.argv[1]+'/images/')
        mkdir_p(sys.argv[1]+'/images/raw/')
        mkdir_p(sys.argv[1]+'/images/annotated/')
	print sys.argv        
        bag_name = sys.argv[1]
	if len(sys.argv) > 2:
		start = int(sys.argv[2])
	else:
		start = None
	if len(sys.argv) > 3:
		stop = int(sys.argv[3])
	else:
		stop = None
	extract_audio(bag_name, audio_topic, bag_name + ".mp3", start, stop)









