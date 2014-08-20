#!/bin/sh
for var in "$@"
do
echo $var
    if [ "$var" -lt 60 ]; then
        expect sync_waypoints.sh $USER $var ut longhorn
    else
        expect sync_waypoints.sh $USER $var siavash 123456
    fi
done
