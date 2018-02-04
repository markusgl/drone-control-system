#!/bin/bash
source ~/bebop_ws/devel/setup.bash
rosrun topic_tools drop /bebop/image_raw 9 10 classifyTopic
