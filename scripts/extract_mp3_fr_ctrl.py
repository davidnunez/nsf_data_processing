#!/usr/bin/env python

# From a rosbag, this script outputs an mp3 file from an audio topic 
# and a directory of images from image topic. It also outputs another
# directory of the same images with time stamp and annotation information
# overlaid.
#
# Code inspired by https://github.com/ros-drivers/audio_common/issues/1
# Usage:
# $ rosrun data_processing extract_mp3.py bag_name_without_.bag_extension
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
import math

audio_topic = "/audio"
image_topic = '/usb_cam/image_raw/compressed'
frames_per_sec = 15 # forces this framerate in the images output, regardless of original framerate; avconv script should use this parameter after -r
annotate = False
vid_delay_from_audio = 0.2 # can be negative in theory, but code doesn't yet support that


def mkdir_p(path):
        try:
                os.makedirs(path)
        except OSError as exc:
                if exc.errno == errno.EEXIST and os.path.isdir(path):
                        pass
                else:
                        raise

def to_video_time(ros_time):
        vid_time = (ros_time.secs%1000000000) + (ros_time.nsecs / 1000000000.0) # remove some leading digits from the seconds-based time 
	return vid_time

def extract_audio(bag_path, topic_name, mp3_path, start=None, stop=None):
	global frames_per_sec
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

        last_frame_time = float('nan')
	frames_written = 0
        secs_per_frame = 1.0 / frames_per_sec
	print "secs per frame ", secs_per_frame
	zero_frames_cnt = 0
	audio_start_time = float('nan')
	video_start_time = float('nan')

	for topic, msg, stamp in bag.read_messages():
		if (stop != None and stop < stamp.secs):
			break
		if (start != None and start > stamp.secs):
			continue




		# if audio topic, write to mp3 file
                if msg._type == 'audio_common_msgs/AudioData':
			if math.isnan(audio_start_time):
				audio_start_time = to_video_time(stamp)
			#if audio_msg_count < 10:
			#	print "audio time stamp",audio_msg_count,": ", to_video_time(stamp)
			audio_msg_count += 1
			mp3_file.write(''.join(msg.data))





                # if image topic, write as many as are needed to maintain framerate (0-n)
                if topic == image_topic:
			if video_msg_count == 0:
				video_start_time = to_video_time(stamp)
			if math.isnan(last_frame_time):
				#last_frame_time = (audio_start_time-0.1) - secs_per_frame # pretend that one frame back was written to keep logic below simpler
				last_frame_time = to_video_time(stamp) - (secs_per_frame + vid_delay_from_audio) # pretend that one frame back was written to keep logic below simpler
				print "time since audio start at first video msg: ", str(to_video_time(stamp) - audio_start_time)

                        # find how many frames should be saved of this image
			time_since_last_frame = to_video_time(stamp) - last_frame_time
			frames_to_write = int((time_since_last_frame / secs_per_frame) + 0.5)
			"""if video_msg_count < 20:
				print "video time stamp",video_msg_count,": ", to_video_time(stamp)
				print "time ", to_video_time(stamp)
				print "writing ", frames_to_write, " frames"
				print "time_since_last_frame ", time_since_last_frame, "\n" """
			last_frame_time += frames_to_write * secs_per_frame
			if frames_to_write == 0:
				zero_frames_cnt += 1
			
			# TODO keep track of max and min frames per sec and report to user
                   
			for i in range(0,frames_to_write):
				img_filename = 'frame%010d.jpg'%(frames_written)
				#if video_msg_count < 20:
				#	print img_filename
				image_file = open(bag_path + "/images/raw/" + img_filename, 'w')
				image_file.write(''.join(msg.data))
				image_file.close()
				if annotate:
					image = Image.open(bag_path + "/images/raw/" + img_filename)
					draw = ImageDraw.Draw(image)
        				### font = ImageFont.truetype("arial.ttf", 20, encoding = "unic")
					draw.text( (10,10), '%d.%d'%(stamp.secs, stamp.nsecs) + ": " + display_message_1, fill = '#ffffff') #, font=font)
					draw.text( (10,20), display_message_2, fill = '#ffffff')
					if (fingerDown):
						draw.text ((10, 30), "fingerDown", fill = "#ff0000")
					image.save(bag_path + "/images/annotated/" + img_filename)
				frames_written += 1


                        video_msg_count +=1
                        #print "image"
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
        print '%d image messages seen'%(video_msg_count)
        print '%d image frames written'%(frames_written)
        print '%d image messages unwritten'%(zero_frames_cnt)
	print '%f first video time stamp: '%(video_start_time)
	print '%f first audio time stamp: '%(audio_start_time)



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









