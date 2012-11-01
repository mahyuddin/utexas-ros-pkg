bwi_multimaster_location=`rospack find bwi_multimaster`
export ROS_IP=`${bwi_multimaster_location}/scripts/get_addr.py wlan0`
export ROS_MASTER_URI=http://${ROS_IP}:11311

