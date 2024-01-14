aist_handeye_calibration: ROS package for estimating camera poses w.r.t. the robot or the world
==================================================

## Overview
This package provides a set of software for estimating a relative transformation between a depth camera and a robotic arm. You can do two things with this software:

- **Calibrating cameras** -- Estimate a relative transformation between the camera and the robot by showing an AR marker to the camera while moving the robot to several known key poses.
- **Checking calibration result** -- Validate the estimated transformation by showing a marker to the camera and then commanding the robot to move its tooltip to the center of the marker.

The calibration is possible under the following two situations:

- **eye_on_hand** -- Camera is mounted on the end-effector of the robot.
- **eye_on_base** -- Camera is fixed to the stationary environment.

## Step 0: Preparing an AR marker

A square AR marker, [ArUco](https://www.uco.es/investiga/grupos/ava/portfolio/aruco/), is required for calibration. Default marker used in this package is the one of ID=26, size=0.07 and margin=0.005 which can be found in [aist_handeye_calibration/markers/aruco-26-70-5.pdf](markers/aruco-26-70-5.pdf). Please print and paste it to a square plate of 80mm x 80mm size. Then, you should fix the marker plate to the environment(`eye_on_hand`) or to the end-effector(`eye_on_base`).

If you want to use a marker with different ID or size, please visit [marker generator site](https://chev.me/arucogen). You should select `Original ArUco` dictionary and specify marker ID as well as its size you want.
The configuration parameters also should be modified in accordance with the marker properties. Please see the section [Parameters for configuring calibration](#parameters-for-configuring-calibration) for details.

## Step 1: Calibrating camera

First, you bring up robots and cameras with the following command:
```bash
$ roslaunch aist_bringup <config>_bringup.launch [sim:=true]
```
where `<config>` is the name of the hardware configuration of the total robot system, e.g. *aist*. If `sim:=true` is specified, the gazebo simulator is launched instead of the drivers for real robots and cameras.

Then, you start the calibrator which acts as a *calibration server* in another terminal with the following command:
```bash
$ roslaunch aist_handeye_calibration handeye_calibration.launch camera_name:=<camera name>
```
where `<camera name>` is the name of the camera to be calibrated, e.g. *a_bot_inside_camera*.

Finally, a *calibration client* should be launched in the third terminal with the following command:
```bash
$ roslaunch aist_handeye_calibration run_calibration.launch config:=<config> camera_name:=<camera name>
```
Here, `<config>` and `<camera name>` must be same as the ones specified above.

After the client comes up, type **calib** command and hit **return** key to start the calibration process. The robot moves to several key poses at each of which the client takes an image of the marker and estimates its pose w.r.t. the camera. After visiting all key poses, a 3D transformation from the camera to the robot is computed and saved in `aist_handeye_calibration/calib/<camera_name>.yaml` and `${HOME}/.ros/aist_handeye_calibration/<camera_name>.yaml`. The former contains only the camera-to-robot transformation, while the latter holds also the marker-to-robot transformation as well. The estimated transformation is also displayed in the terminal which has launched the client together with the residual errors.

You can terminate the client by hitting **q** and **return**.

### Important note
The calibration result stored in `aist_handeye_calibration/calib/<camera_name>.yaml` is loaded into the ROS parameter `/robot_description` when bringing up robots and cameras. The parameter `/robot_description` contains all information about geometric configuration of robots, cameras and other devices as well as their surrounding environment. This means the following two things:

1. Before the first calibration, you do not have `<camera_name>.yaml` and bringing up robots will fail. To avoid this, you have to use nominal calibration values as temporary settings by:
```
$ cd aist_handeye_calibration/calib
$ cp <camera_name>-nominal.yaml <camera_name>.yaml
```
2. After performing calibration, you have to terminate all ROS nodes and bring up the robots again so that the new calibration values are made effective by loading them into `/robot_description`.

## Step 2: Checking calibration result

You can validate the calibration result by checking whether the robot can move its tooltip to the target position specified by the marker placed at an arbitrary position.

First, you should bring up robots and cameras again to load new calibration values:
```bash
$ roslaunch aist_bringup <config>_bringup.launch
```
Then, you start the ArUco marker detector in another terminal with the following command:
```bash
$ roslaunch aist_handeye_calibration handeye_calibration.launch camera_name:=<camera name>
```
Here, `<config>` and `<camera name>` should be same as the ones specified in Step 1, respectively.

Next, the *checking client* is launched in another terminal with the following command:
```bash
$ roslaunch aist_handeye_calibration check_calibration.launch config:=<config> camera_name:=<camera name>
```
Here, `<config>` and `<camera name>` should be same as the above.

After the client coming up, simply hit **i** and **return** keys to move the robot to the initial position. Then place the marker at an arbitrary position with an arbitrary orientation within the camera's field of view and hit **return**. The robot will go to the approach position 50mm above the marker and descend to the center along the normal direction. You can repeat this process varying the marker positions and orientations.

You can terminate the client by hitting **q** and **return**.


## Parameters for configuring calibration

The calibration process can be customized by adjusting the parameters listed below. They are stored in `aist_handeye_calibration/config/<camera_name>.yaml` where `<camera_name>` is the name of the camera to be calibrated.
- **camera_name** -- Name of the camera, e.g. *a_bot_inside_camera*
- **robot_name** -- Name of the robot used for calibration, e.g. *a_bot*. In the **eye_on_hand** configurations, the name of the robot on which the camera is mounted should be given. In the **eye_on_base** configurations, name of the robot to which the marker attached should be specified.
- **eye_on_hand** -- *true* for the **eye_on_hand** configurations while *false* for **eye_on_base** configurations.
- **robot_base_frame** -- ID of the frame with respect to which the cartesian robot poses are specified, e.g. *workspace_center*.
- **robot_effector_frame** -- ID of the frame attached to the end-effector of the robot,e.g. *a_bot_ee_link*. When calibrating a camera, robot motions are given as poses of this frame with respect to the *robot_base_frame*.
- **camera_frame** -- ID of the frame associated with the depth and intensity/color images, e.g. *a_bot_inside_camera_color_optical_frame*. This value must be identical to that specified when launching tha camera. In addition, depth images must be aligned to intensity/color images so that they have a common projection center.
- **marker_map_dir** -- Directory containing configuration files of marker maps
- **marker_map** -- Name of marker map
- **marker_size** -- Side length of the square marker (in meters). Margins are not included (valid only when **marker_map** is not specified).
- **planarity_tolerance** -- Thresholding value for filtering out outliers when fitting a plane to the 3D points within a detected marker region in the depth image (in meters).
- **initpose** -- Initial robot pose when checking calibration results. The pose is given by a 6-dimensional array with x/y/z coordinates (in meters) and roll/pitch/yaw angles (in degrees) with respect to the *robot_base_frame*.
- **keyposes** -- An array of robot poses where marker images are captured when calibrating the camera. Each pose is given by a 6-dimensional array with x/y/z coordinates (in meters) and roll/pitch/yaw angles (in degrees) with respect to the *robot_base_frame*.

## Calibration algorithm

The calibrator implements the following two algorithms for estimating a camera pose from a set of transformation pairs composed of the one from the end-effector to the robot base and the one from the marker to the camera.
- **Classical method** -- This algorithm first optimizes camera orientation and then optimizes camera rotation while the orientation is fixed to the value estimated before.
- **Dual quaternion method** -- This algorithm optimizes both camera position and orientation simultaneously, which gives more accurate results compared with the classical method. See
```
K.Daniilidis and E. Bayro-Corrochano, The dual quaternion approach to hand-eye calibration, Int. J. Robotics Research, 18: 286-298, 1999
```
If `use_dual_quaternion:=false` is specified when launching the calibrator, the former algorithm is used. The latter is adopted otherwise.