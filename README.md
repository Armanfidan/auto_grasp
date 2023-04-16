# Auto Grasp
1. [Overview](#overview)
2. [Getting Started](#getting-started)
3. [How to Run](#how-to-run)

## Overview
A ROS library for dynamic autonomous grasping with the BostonDynamic Spot quadruped robot and the Kinova Gen 3 Manipulator.
Contains ROS nodes for manipulation, robot movement, movement compensation and computer vision.

## Getting Started
As a pre-requisite, build the Docker images in https://github.com/Armanfidan/PRL-Dockerfiles.git.
Keep in mind that you will need a BostonDynamics Spot robot, Kinova Gen 3 Manipulator and an NVIDIA Jetson NX to be able to run this project.

This package contains ROS nodes ot be run on two devices: NVIDIA Jetson NX and BostonDynamic Spot.

For the latter, build the `arman_spok` image and spin up a container from this image:

```
git clone https://github.com/Armanfidan/PRL-Dockerfiles.git
cd PRL-Dockerfiles
docker build arman_spok
docker start arman_spok
```

This image contains all dependencies of this repository, as well as the repository itself and the correct directory mounts and configurations.

The ROS nodes in this package have different dependencies, based on the device they are run on.

Pre-requisites for the Jetson NX: JetPack SDK 4.2 flashed onto an SD card

1. Download the image on https://developer.nvidia.com/downloads/embedded/l4t/r35_release_v3.1/sd_card_b49/jp511-xnx-sd-card-image.zip/
2. Follow the instructions on https://developer.nvidia.com/embedded/learn/get-started-jetson-xavier-nx-devkit#write to flash an SD card with this image.

This will set up the JetPack SDK, with Ubuntu 18.04.
Then install ROS Melodic and the `ros_deep_learning` package from https://github.com/dusty-nv/ros_deep_learning on this device.

Nodes to be run on the Jetson NX, with ROS Melodic on Ubuntu 18.04:

* DetectNet Switcher
* DetectNet (From `ros_deep_learning`)

Nodes to be run on Spot (On the `arman_spok`  Ubuntu 20.04 with Ros Noetic and Ubuntu 18.04 with Ros Melodic.

* MovingPlanner
* ImageRotator
* ObjectLocaliser
* SpotMover

## How to Run
A specific sequence should be followed to run the project.
Each step will be annotated with the device it should be run on.
The devices will be referred to as follows:
* NVIDIA Xavier NX: "Xavier"
* BostonDynamics Spot: "Spot"

### Steps to run
1. Spot: Turn on the robot, make sure the Robot Body is on and engaged.
Turn on the Kinova arm and wait for it to power up and configure.
2. Spot: Build and run `arman_spok` in interactive mode using the CMake script `make arman_spok`.
All steps after now will be run in this conainer. This will start ROS Core automatically.
3. Spot: Start the Kinova (and Kinova Vision Module) driver with `make kinova-driver`.
4. Spot: Start the Spot camera image rotation node with `rosrun auto_grasp image_rotator`.
5. Xavier: Start ROS with `roscore`.
6. Xavier: In a new terminal window, run `roslaunch ros_deep_learning detectnet.ros1.launch`.
7. Xavier: In a new terminal window, run `rosrun auto_grasp detectnet_switcher`.
8. Spot: In a new terminal window, run `rosrun auto_grasp spot_mover`.
9. Spot: In a new terminal window, run `rosrun auto_grasp moving_planner`.

Optionally, you can start RViz to see the detected objects through the camera and in three-dimensional space.
You can now simply a button in the MovingPlanner window to grasp the object, move Spot to its initial position and move it around.
