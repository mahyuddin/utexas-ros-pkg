#!/bin/sh
file="00-bwi-camera.rules"
target="/etc/udev/rules.d/$file"
source=$(rospack find bwi_camera)/$file
sudo cp $source $target
