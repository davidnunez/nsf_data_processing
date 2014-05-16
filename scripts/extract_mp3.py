#!/usr/bin/env python

# Code inspired by https://github.com/ros-drivers/audio_common/issues/1

import sys
import rospy
from ros import rosbag

def extract_audio(bag_path, topic_name, mp3_path):
	print 'Opening bag:' +  bag_path
	bag = rosbag.Bag(bag_path)
	mp3_file = open(mp3_path, 'w')
	print 'Reading audio messages and saving to mp3 file'
	msg_count = 0
	for topic, msg, stamp in bag.read_messages(topics=[topic_name]):
		if msg._type == 'audio_common_msgs/AudioData':
			msg_count += 1
			mp3_file.write(''.join(msg.data))
	bag.close()
	mp3_file.close()
	print 'Done. %d audio messages written to %s'%(msg_count, mp3_path)

if __name__ == '__main__':
	extract_audio(sys.argv[1], "/audio", sys.argv[1] + ".mp3")
