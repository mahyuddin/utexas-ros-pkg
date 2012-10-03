clear
read -p "Step 1: Select the name you want to use for the camera: " camname
echo
read -p "Step 2 (optional): You can now create a mapping for this camera if it hasn't been registered before. Enter the device name for the new camera, or just press [Enter] to skip this step: " device
if [ -n $device ]; then
    $(rospack find bwi_camera)/scripts/addUdevRule.sh $device $camname
    echo
    read -s -p "The mapping has been created. Unplug the camera and then plug it back in to ensure that the mapping runs correctly. Press [Enter] after you have done this."
else
  echo
  echo "Skipping mapping step."
fi
echo
echo "Step 3: Add optional data points to $(rospack find map_camera_pose)/data/example_points.yaml. These points will appear on the displayed map for simplifying correspondence selections."

read -s -n 1 -p "Open this file in vim for editing? (y/n)" result
echo
if [ $result == "y" ]; then
  vim $(rospack find map_camera_pose)/data/example_points.yaml
  echo "Finished editing example points."
else
  echo "Skipped example points."
fi
echo
read -s -p "Step 4: Run the transform producer and create correspondences between the camera frame and the map by clicking points in each sequentially. When four correspondences have been chosen, the producer will output a valid transform file with each response. The file will be written to $(rospack find bwi_camera)/launch/$camname-tf.launch. You may stop the transform producer at any point after the transform file has been created. Press [Enter] to continue."
cat $(rospack find bwi_camera)/launch/camera-template.launch | sed s/##CAMERA_NAME##/$camname/ > $(rospack find bwi_camera)/launch/$camname.launch
roslaunch bwi_camera tfSetup.launch camname:=$camname
