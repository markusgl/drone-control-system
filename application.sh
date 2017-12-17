#!/bin/bash
chmod +x repup.sh
gnome-terminal -e "./repup.sh"
source ~/bebop_ws/devel/setup.bash
python main.py
