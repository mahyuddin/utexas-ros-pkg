#!/bin/sh
path=$1
name=$2
file="00-bwi-camera.rules"
source=$(rospack find bwi_camera)/$file
target="/etc/udev/rules.d/$file"
camlist="$(rospack find bwi_camera)/cameras.yaml"

if [ ! -e "/dev/$path" ]; then
  echo "No device at $path"
  exit 1
fi
if [ -z "$path" ]; then
  echo "No path specified."
  exit 1
fi
if [ -z "$name" ]; then
  echo "No name specified."
  exit 1
fi


# grab serial info
serial=$(udevadm info -a -p `udevadm info -q path -n /dev/$path` | grep ATTRS{serial}== -m 1| awk -F"==" '{print $2}')
vendor=$(udevadm info -a -p `udevadm info -q path -n /dev/$path` | grep ATTRS{idVendor}== -m 1| awk -F"==" '{print $2}')
product=$(udevadm info -a -p `udevadm info -q path -n /dev/$path` | grep ATTRS{idProduct}== -m 1| awk -F"==" '{print $2}')
if [ -f $source ]; then
  matches=$(cat $source | grep $serial)
fi
if [ -z "$matches" ]; then # make sure the serial isn't already mapped
  #add rule to the source rules file
  echo "KERNEL==\"video[0-9]*\", ATTRS{idVendor}==$vendor, ATTRS{idProduct}==$product, ATTRS{serial}==$serial, SYMLINK+=\"$name\"" >> $source
fi

# refresh rules in /etc/udev/rules.d
$(rospack find bwi_camera)/scripts/refreshRules.sh

# set up yaml list for nodes to read from
$(rospack find bwi_camera)/scripts/cameras.py add "$name" "$camlist"
