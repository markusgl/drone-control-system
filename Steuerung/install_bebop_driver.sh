#!/bin/bash

sudo apt-get install build-essential python-rosdep python-catkin-tools
mkdir -p ~/bebop_ws/src && cd ~/bebop_ws
catkin init
git clone https://github.com/AutonomyLab/bebop_autonomy.git src/bebop_autonomy
git clone https://github.com/AutonomyLab/parrot_arsdk.git src/parrot_arsdk
rosdep update
rosdep install --from-paths src -i
catkin build
