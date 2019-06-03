# CarND-Capstone-Project
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

## Perception Subsystem

There are two nodes in this subsystem, the `Obstacle Detection Node` and the `Traffic Light Detection Node`, in this project, only the `Traffic Light Detection Node` needs to be completed.

The `Traffic Light Detection Node` subscribes to four topics:

* `/base_waypoints` provides the complete list of waypoints for the course.
* `/current_pose` can be used to determine the vehicle's location.
* `/image_color` provides an image stream from the car's camera. These images are used to determine the color of upcoming traffic lights.
* `/vehicle/traffic_lights` provides the (x, y, z) coordinates of all traffic lights

The Traffic Light Detection Node is divided into two parts, the first part is used to detect the state of the traffic lights in the picture captured by the camera, which is completed in `tl_classifier.py`, the second part find the nearest visible traffic light ahead of the vehicle, which is completed in `tl_detector.py`

### Traffic Light Classification

We use the TensorFlow Objection Detection API to detect and classify the traffic light. We do the detection and classification in one stage. We test four models:

* SSD-inception
* SSD-mobile
* Faster-rcnn-resnet50
* Faster-rcnn-inception-resnet-v2-atrous

`Detection speed` SSD-mobile > SSD-inception > Faster-rcnn-resnet50 > Faster-rcnn-inception-resnet-v2-atrous

`Detection accuracy` Faster-rcnn-inception-resnet-v2-atrous > Faster-rcnn-resnet50 > SSD-inception > SSD-mobile


### Traffic Light Detection

Since the exact location of the traffic lights and parking lines is deterministic, we don't need to be constantly detecting traffic lights. When the traffic signal is far away from us, there is no need to detect the traffic signal. Only when the traffic signal is within the distance we set (200 waypoints), the detection model starts to detect. When the red light is detected, the car will decelerate， and stop before the stop line. When the green light is detected, the car continues to move forward.

## Plan Subsystem


Waypoint Updater Node


## Control Subsystem


DBW Node

## Installation

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation

[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container

```bash
docker build . -t capstone
```

Run the docker file

```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Port Forwarding

To set up port forwarding, please refer to the [instructions from term 2](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77)

### Usage

1. Clone the project repository

```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies

```bash
cd CarND-Capstone
pip install -r requirements.txt
```

3. Make and run styx

```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```

4. Run the simulator

### Real world testing

1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file

```bash
unzip traffic_light_bag_file.zip
```

3. Play the bag file

```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```

4. Launch your project in site mode

```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```

5. Confirm that traffic light detection works on real life images

