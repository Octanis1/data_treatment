# Octanis 1 Mission Data Treatment

This repository is home to a catkin workspace containing various packages used to extract and/or process information collected during the Octanis 1 rover mission to Antarctica. The source format is rosbag which stores all topics that run on the rover's single board computer.

## ROS Nodes
  * data_extraction
  * mavlink_conversion
  * stereocamera_export - saves "/stereo/[right,left]_cam/image_raw" to a folder as JPG
