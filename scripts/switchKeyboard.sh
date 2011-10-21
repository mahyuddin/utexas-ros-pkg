#!/bin/bash

setxkbmap -query | grep dvorak > /dev/null
if [ $? -eq 0 ]
then
  # currently in dvorak
  setxkbmap us
else
  # currently in us
  setxkbmap dvorak
fi
