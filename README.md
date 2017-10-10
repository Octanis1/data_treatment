# Octanis 1 Mission Data Treatment

This repository is home to a catkin workspace containing various packages used to extract and/or process information collected during the Octanis 1 rover mission to Antarctica. The source format is rosbag which stores all topics that run on the rover's single board computer.

## ROS Nodes
  * data_extraction
  * mavlink_conversion
  * stereocamera_export
    * saves "/stereo/[right,left]_cam/image_raw" to a folder as JPG
    * run "rosrun stereocamera_export exporter.py destination_folder rosbag_filename"


## ROS Bags

Some rosbags had to be reindexed for replay. All mission rosbags were prepended a letter for quick reference, the original pseudo-date in the filename was kept. (Pseudo-date due to the linux system time not being in-sync with GPS time).

  * A_2016-09-19-13-24-36_0.bag / size=1.94GB,duration=1446s
  * B_2016-09-19-08-28-16_0.bag / size=1.56GB,duration=1364s
  * C_2016-09-19-14-23-54_1.bag / size=854MB,duration=657s
  * D_2016-09-19-13-53-54_0.bag / size=2.53GB,duration=1794s
    * image extract (jpg): https://drive.google.com/file/d/0Bwa66ZQR4ocHV2xpajVDNDNuUVE/view?usp=sharing
  * E_2016-09-19-13-02-22_0.bag / size=1.5GB,duration=1102s
  * F_2016-09-19-12-52-19_0.bag / size=402MB,duration=301s
  * G_2016-09-19-08-56-13_0.bag / size=408MB,duration=334s
  * H_2016-09-19-08-02-25_0.bag / size=931MB,duration=795s
  * I_2016-09-19-07-18-47_0.bag / size=50MB,duration=26s 

Total recorded time: 7829s


## For ROS catkin workspace: Quickstart
  1. Clone this repository to the ROS Indigo VM
  2. Run catkin_make
     * If a package is not compiling, ignore it with: catkin_make -DCATKIN_BLACKLIST_PACKAGES="mavlink_conversion"

## ROS troubleshooting 
  * "CMake Error at ... Could not find a package..." -> library is missing, look for it via "sudo apt-cache search ros-indigo yourMissingLibrary"
