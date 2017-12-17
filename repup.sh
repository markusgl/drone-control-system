#!/bin/bash
source ~/bebop_ws/devel/setup.bash
rosrun topic_tools drop /bebop/image_raw 1 10 classifyTopic
