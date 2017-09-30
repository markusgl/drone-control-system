#!/bin/bash
chmod +x application.sh
chmod +x bebop_driver.sh
gnome-terminal -e "./bebop_driver.sh"
sleep 2
gnome-terminal -e "./application.sh"
