# VIO_SLAM
## I. Basics of Camera Calibration:
Camera calibration is a process to find the camera intrinsic, extrinsic, and distortion parameters of a specific camera. It is very important in the multiple-camera and camera-IMU (visual inertial) system.  

#### a. Camera Teminology:
 - **Intrinsic Matrix**: The matrix that transform 3D points from camera 3D coordinate frame to 2D image plane. Parameters (focal length, offset): fx, fy, px, py.
 - **Extrinsic Matrix**: The matrix that transform 3D points from other coordinate frame to camera 3D coodinate frame. Note: Usually this refers to the transform between camera and camera (in multiple-camera system) or transform between camera and IMU (in visual inertial system). This is represented by the transformation matrix (4x4, rotation and translation).
 - **Distortion Parameters**: Camera lens can cause distortion in image. More details on parameters about [camera distortion](https://ori.codes/artificial-intelligence/camera-calibration/camera-distortions/).

#### b. Calibration Target:
 - **Checkerboard**:
   - *Target row/column*: Number of INNER boxes in row/column.
   - *Size*: Size of each boxes.
 
     <img src="https://user-images.githubusercontent.com/55560905/162593666-e2a997c7-1dc2-4203-aaf4-72dbb8b5e61b.png" width="300">
 
 - **Aprilgrid**:
   - *Target row/colum*: Number of INNER boxes in row/column.
   - *Size*: Size of each boxes.
   - *Spacing*: Distance between two boxes in mm. (sometimes it is represented by a ratio).
   
     <img src="https://user-images.githubusercontent.com/55560905/162593770-031a92b6-2994-4da0-9823-dd569ca4de69.png" width="300">
   
#### c. Types of Camera Calibrations:
 - **Single camera calibration**: To find intrinsic matrix and distortion parameters.
 - **Multiple-camera calibration**: In addition to instrinsic and distortion, we also want to obtain extrinsic matrix.
 - **Camera-IMU calibration**: Find the transformation between camera and IMU.


## II. Calibration Guide (Camera-IMU calibration)

#### 1. Install Kalibr from ETH Autonomous System Lab (ASL).
  [Kalibr](https://github.com/ethz-asl/kalibr) is a very useful calibration tool for multiple-camera and camera-IMU calibration. Note: Follow their instructions and build the packages in your Ubuntu 18.04 (ROS Melodic) or Ubuntu 16.04 (ROS Kinetic) system.

#### 2. Record rosbag for camera intrinstic and extrinsic calibration at 4-5 Hz.
Take [Intel Realsense D435i](https://www.intelrealsense.com/depth-camera-d435i/) as an example:
If you want to calibrate its stereo camera: 
```
roslaunch realsense2_camera rs_d435i.launch # check the launch file in this repo, you will need to turn off the projector and also ajust the image resolution.

# we need to throttle message frequency to 4-5 Hz
rosrun topic_tools throttle messages /camera/infra1/image_rect_raw 5.0 /camera1 # throttle image1 frequency and republish in /camera1
rosrun topic_tools throttle messages /camera/infra2/image_rect_raw 5.0 /camera1 # throttle image2 frequency and republish in /camera2

# start recording
rosbag record -O stereo_camera_calibration.bag /camera1 /camera2
```
Then, follow the video in [Kalibr](https://github.com/ethz-asl/kalibr) for how to collect data.

Camera calibration:
```
rosrun kalibr kalibr_calibrate_cameras --bag camera_calibration.bag --topics /camera1 /camera2 --models pinhole-radtan pinhole-radtan --target apriltag.yaml --bag-from-to 0 75  --approx-sync 0.1
```

Camera-IMU calibration:
```
rosrun kalibr kalibr_calibrate_imu_camera --bag calibration.bag --cam camera_calibration_result/camchain-camera_calibration.yaml --imu imu_calibration_result/imu.yaml --target apriltag.yaml --bag-from-to 5 105
```
