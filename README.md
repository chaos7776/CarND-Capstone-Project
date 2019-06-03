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



