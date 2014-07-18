#!/bin/bash

rostopic pub  -1 /ros_to_intent std_msgs/String '"{\"tablet_id\":1, \"key\":\"start_prompt\" }"'

