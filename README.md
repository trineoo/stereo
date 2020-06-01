# Stereo Vision for Autonomous ferry - code
The project is a part of TTK4900 - Engineering Cybernetics, Master's Thesis at Norwegian University of Science and Technology. "Stereo Vision for Autonomous ferry" goes into detail of the calibration, the chosen stereo setup and the implementation for testing in dynamic scenes. The ownship has adequate data to sense, process and understand its surroundings.

## Contents
* [Getting Started](#getting-started)
* [System overviw](#system-overview)
* [Launch](#launch)
* [Connect to Network MilliAmpere](#connect-to-network-on-milliampere)
* [Application and Scrips](application-and-scrips)
* [Authors and License](#authors-and-license)


## Getting Started
Hello from two soon-to-be well educated grown-ups.

### Prerequisites
 * Setup a computer with GPU and Ubuntu 16.04. This is tested on a Dell something something.
 * Install Spinnaker and verify that you can run your cameras with SpinViw (prefer Windows operating system). Setup a stereo system with either hardware og software triggering. Modify the yaml files in spinnaked-sdk-driver replacing the cam-ids and master cam serial number to match your camera's serial number. Also include the calibration parameters in.... This repo is tested with [Blackfly S GigE](https://www.flir.com/products/blackfly-s-gige/?model=BFS-PGE-50S5C-C), using PoE, master software triggering and GPIO pins for external triggering the slave. 
 * Optionally: Set up the  [LiDAR driver](http://wiki.ros.org/velodyne/Tutorials/Getting%20Started%20with%20the%20Velodyne%20VLP16). The LiDAR needs to be calibrated agaisnt the stereo camera to be used as a Ground Truth or as basis for comparison. The code is tested using [Velodyne LiDAR Puck-16](http://www.isaza.co/VELODYNE/63-9243%20Rev%20B%20User%20Manual%20and%20Programming%20Guide,VLP-16.pdf)

### Installing
* [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
* [CUDA 10.2](https://developer.nvidia.com/cuda-downloads) for runnning darknet ros (YOLOv3) on GPU.
* [Python](https://www.python.org/downloads/). The scripts is tested with Python 2.7.12 
  1. [Matplotlib](https://matplotlib.org/users/installing.html) and [OpenCV](https://opencv.org/)
```Bash
  python -mpip install -U pip  #Installs Python pip
  python -mpip install -U matplotlib #Plot the results by installing Matplotlib
  python -mpip install -U opencv-python #Show animation by installing 
```

## System overview
Bilde av systemet/systemflow kommer :))


## Launch 
```bash
cd Master
catkin build
roslaunch launch clustering_cnn file:= "your bag file"  #launch bagfile, stereo_image_proc, yolo and clustering_cnn
roslaunch launch clustering_ptcloud file:= "your bag file"  #launch bagfile, stereo_image_proc, clustering_ptcloud
```

### Run individual packages
```bash
cd Master
source devel/setup.bash
```
#### Camera driver
* `roslaunch spinnaker_sdk_camera_driver acquisition.launch`

  1. Rosbag: `rosbag play "filename" --clock`
  2. And rosbag include redirecting topics: `rosbag play "filename" /camera_array/cam0/image_raw:=/camera_array/left/image_raw /camera_array/cam1/image_raw:=/camera_array/right/image_raw /camera_array/cam0/camera_info:=/camera_array/left/camera_info /camera_array/cam1/camera_info:=/camera_array/right/camera_info --clock`

#### Stereo image proc
* `ROS_NAMESPACE=camera_array rosrun stereo_image_proc stereo_image_proc`
##### display images
* `rosrun image_view stereo_view stereo:=/camera_array image:=image_rect_color #rectified images and disparity map`
* `rosrun image_view stereo_view stereo:=/camera_array image:=image_raw #raw images`
* `rosrun image_view image_view image:=/camera_arr/left/image_rect_color  #left rectified image`
#### Clustering
* `roslaunch clustering pcl_obstacle_detector.launch #Need the bagfile to be run with the "--clock"`
#### Darknet_ros (YOLOv3)
```bash
roslaunch darknet_ros yolo_v3.launch  #subscribes on camera_array/left/image_rect_color
```

## Connect to Network on MilliAmpere
If the code is to be run with the master core on the ferry Milliampere a local network need to be setup. The best solution is to set up a separate static network on the computer through usb3. 

```bash
ifconfig #find your network
sudo nano /etc/network/interfaces #add/make the network static
nano /etc/hosts #add milliAmpere as host on machine, further do the same at milliAmpere
sudo ifdown "your network" && sudo ifup "your network" #restarting interface
ping milliAmpere #check if connection established
```
Now that you have created a static network, one have to export the roscore. Telling your machine the roscore is running on anoher computer. Be sure to do this in every terminal window (can be advanatges to add in nano .bashrc) 
```bash
export ROS_MASTER_URI=http://milliAmpere:11311
echo $ROS_MASTER_URI
rostopic echo /topic #test that connection is established
```
## Application and Scrips
### stereoTuner
Application for tuning the disaprity map. Using the stereoBM object and WLS filter from openCV. The GUI is a modified version of the repository [stereo-tuner](https://github.com/guimeira/stereo-tuner). Remember to rectify the images in beforehand, you're welcome. 
It is a simple little GTK application that can be used to tune parameters for the OpenCV Stereo Vision algorithms.

```bash
mkdir build
cd build
cmake ..
make
./main
```


### Label YOLO images
GUI for marking bounded boxes of objects in images for training Yolo v3. 

* Put your `.jpg`-images to the directory `x64/Release/data/img`. 
* Change numer of classes (objects for detection) in file `x64/Release/data/obj.data
* Put names of objects, one for each line in file `x64/Release/data/obj.names
If the images is used for training a custom dataset visit [AlexBA](https://github.com/AlexeyAB/Yolo_mark) for a more detailed explanation.  
```bash
cmake .
make
./linux_mark.sh
```


### Precision-recall curve for YOLOv3
Plot and calculate the precision-recall curve from ground thruth images. It iterates through two for-loops, one with IoU-threshold and the second with the YOLO-threshold. Make sure to input detection images from the network with threshold less than the ones in the for-loop in main.py. The mAP script is a modifed version of the code in the github repository [mAp](https://github.com/Cartucho/mAP). It outputs a precision-recall curve for each threshold and IoU-threshold in main.py

* Create the ground truth files using [Label YOLO images](#Label-YOLO-images)
  1. Insert images into the folder input/ground-truth/images 
  2. Insert ground-truth files into ground-truth/
  3. Add class list to the file scripts/extra/class_list.txt
  4. Run the python script: python convert_gt_yolo.py to convert the txt-files to the correct format
* Create the detection-results files by running darknet_ros
  1. Copy the detection-results files into the folder input/detection-results/
* Run the code: python main.py

note: Be consistent with using rectifiec/not-rectified images. 
```bash
python main.py
```
Feel free to edit the main file with your preferred values in the for-loops. 

<img src="/applications_scripts/precision-recall/scripts/extra/thresChange0.1.png"/>

### Plot and calculate ground truth depth and stereo depth
Match handhold-GPS csv file with the GPS from MA by satellite time. Match the stereo ros-time and plot a beautiful graph. 



## Authors and License

* **Trine Ødegård Olsen** - [trineoo](https://github.com/trineoo)
* **Lina Theimann** - [Linact](https://github.com/linact)

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details


