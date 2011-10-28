#!/bin/bash

DEVICE=`ls /dev/ttyUSB* -1 | head -1`
echo "Setting USB Device as $DEVICE"
rosparam set /brown/irobot_create_2_1/port $DEVICE
