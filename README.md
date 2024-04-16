# Unitree Z1 Robotic Arm Control for ROS Noetic

This repository is based on the [Unitree Z1 SDK](https://github.com/unitreerobotics/z1_sdk) and extends its functionalities for control within ROS Noetic environment. It includes calibration for both eye-in-hand and eye-to-hand camera setups.

## Hardware and Software Environment

- **Compute Platform:** Nvidia Jetson Orin NX
- **Operating System:** Jetpack 5.1.2 (Ubuntu 20.04)
- **ROS Distribution:** ROS Noetic
- **Cameras:** Realsense D455/D455f
- **Robotic Arm:** Unitree Z1

## Example Programs

The `examples` folder contains official sample programs provided by Unitree, as detailed on the [official wiki](https://dev-z1.cn.unitree.com/%E6%9C%BA%E6%A2%B0%E8%87%82%E4%BD%BF%E7%94%A8/SDK%E4%BB%8B%E7%BB%8D.html#22-examples). Our scripts are adaptations of the `highcmd_basic` example.

### Running the Examples

To run an example program, first start the controller by opening a terminal and running:

```shell
./z1_ctrl
```

Then, execute the program, for example:

```shell
rosrun z1_ros highcmd_development
```

For convenience, our launch files include the above steps, allowing you to start with:

```shell
roslaunch z1_ros highcmd_development.launch
```

## ROS Programs

The `src` folder contains our ROS programs. `highcmd_basic_ros.cpp` is our modified script demonstrating `moveJ`, `moveL`, and `moveC` controls. These can also be run using roslaunch.

## Calibration Programs

Calibration is conducted using Apriltag with the 41h12 tag family. Please download and place it in your workspace from [Apriltag ROS](https://github.com/AprilRobotics/apriltag_ros).

My other repository, [my_tf_transformer](https://github.com/4399chen/my_tf_transformer), can rotate the `tag_1` TF frame to align with the Unitree Z1 coordinate frame. While not necessary for the calibration process, it is essential for testing the calibration results.

`test.launch` includes commands to start the Realsense D455 camera, Apriltag_ros, and my_tf_transformer:

```shell
roslaunch z1_ros test.launch
```

`eye_in_hand_calibration.cpp` and `eye_to_hand_calibration.cpp` are the programs for eye-in-hand and eye-to-hand camera calibration, respectively, using the `cv::calibrateHandEye` function.

Calibration involves moving the robotic arm to points specified in `./config/1.csv` and `2.csv`, with each line containing Euler angles followed by XYZ coordinates.

`test_eye_in_hand_calibration.cpp` and `test_eye_to_hand_calibration.cpp` are programs to test the calibration results. Ideally, the 3D printed tip of the robotic arm should touch the center of the Apriltag.

A video of the calibration process can be viewed [here](https://www.bilibili.com/video/BV1Px4y1a7cZ/).

The 3D printed tip model used for calibration is available on [MakerWorld](https://makerworld.com/zh/models/430201#profileId-334619).

> Note: The calibration results were suboptimal, with a 6-degree pitch deviation in the eye-in-hand results and approximately 25 cm deviation along the X-axis in the eye-to-hand results.
