# Camera-IMU Calibration Guide
## I. Basics of Camera Calibration:
Camera calibration is a process to find the camera intrinsic, extrinsic, and distortion parameters of a specific camera or cameras. It is very important in the multiple-camera and camera-IMU (visual inertial) system.  

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
 - **Multiple-camera calibration**: In addition tNow, we need to estimate noise model for IMU. You can find some explanation of the parameters here. Use the repo imu_utils which depends on code_utils to obtain the IMU noise model.o instriNow, we need to estimate noise model for IMU. You can find some explanation of the parameters here. Use the repo imu_utils which depends on code_utils to obtain the IMU noise model.nsic and distortion, we also want to obtain extrinsic matrix.
 - **Camera-IMU calibration**: Find the transformation between camera and IMU.

#### d. Camera Model:
 TODO

## II. Calibration Guide (Camera-IMU calibration)

#### 1. Install Kalibr from ETH Autonomous System Lab (ASL).
  [Kalibr](https://github.com/ethz-asl/kalibr) is a very useful calibration tool for multiple-camera and camera-IMU calibration. Note: Follow their instructions and build the packages in your Ubuntu 18.04 (ROS Melodic) or Ubuntu 16.04 (ROS Kinetic) system.
Camera Model: your camera lens model.
#### 2. Generate Calibration Target:
You can download calibration target anywhere or generate it from [Kalibr](https://github.com/ethz-asl/kalibr) by running
```
rosrun kalibr kalibr_create_target_pdf --type apriltag --nx 6 --ny 6 --tsize 0.02 --tspace -0.3 # 6x6 apriltag with tag size 0.02m and spacing ratio 0.3.
```
Print this pdf with its actual size and write its parameters into this [format](https://github.com/Zhefan-Xu/camera-imu-calibration-guide/blob/main/apriltag.yaml).

#### 3. Record rosbag for camera intrinstic and extrinsic calibration at 4-5 Hz.
Take [Intel Realsense D435i](https://www.intelrealsense.com/depth-camera-d435i/) as an example:

If you want to calibrate its stereo camera, you need to launch the camera with infrared stereo images (but please turn off the Infrared Projector). Also remember to resize the image size to 640x480. Below is the image with IR projector on:

![Screenshot from 2022-04-10 15-50-39](https://user-images.githubusercontent.com/55560905/162637326-473fc871-ff19-471d-bea2-fe2bda9d858b.png)

Below is the image with IR projector off. Please check yourself in Rviz:

![Screenshot from 2022-04-10 15-51-20](https://user-images.githubusercontent.com/55560905/162637380-ecc20f1b-034a-4c60-84a5-9c5196aa7c1c.png)

You can find the modified launch file [here](https://github.com/Zhefan-Xu/camera-imu-calibration-guide/blob/main/rs_d435i.launch): 
```Now, we need to estimate noise model for IMU. You can find some explanation of the parameters here. Use the repo imu_utils which depends on code_utils to obtain the IMU noise model.
roslaunch realsense2_camera rs_d435i.launch # check the launch file in this repo, you will need to turn off the projector and also ajust the image resolution.
```
Since the frequency of published images is 30Hz, we need to throttle it into 4-5 Hz. Othersize it will be very slow for calibration.
```
# we need to throttle message frequency to 4-5 Hz
rosrun topic_tools throttle messages /camera/infra1/image_rect_raw 5.0 /camera1 # throttle image1 frequency and republish in /camera1
rosrun topic_tools throttle messages /camera/infra2/image_rect_raw 5.0 /camera1 # throttle image2 frequency and republish in /camera2
```
Record rosbag by running the following command:
```
# start recording
rosbag record -O stereo_camera_calibration.bag /camera1 /camera2
```

Or if you want to calibrate its monocular camera:
```
roslaunch realsense2_camera rs_d435i.launch # check the launch file in this repo, you will need to turn off the projector and also ajust the image resolution.

# we need to throttle message frequency to 4-5 Hz
rosrun topic_tools throttle messages /camera/color/image_raw 5.0 /camera_color # throttle color image frequency and republish in /camera_color

# start recording
rosbag record -O color_camera_calibration.bag /camera_color
```
Then, follow the video in [Kalibr](https://github.com/ethz-asl/kalibr) for how to collect data.

#### 4. Calibrate camera with collected rosbag.
Input/Requirements:
  - Rosbag: the image data you collected in the previous step.
  - Data Topic: image message topic names.
  - Camera Model: your camera lens model.
  - Calibration Target YAML file: your calibration target parameters. Please check this repo for examples.
  - Rosbag time interval.
  - Synchronize time: if too big, it causes confusions in stereo image matching. if two small, it causes no image matching.
```
rosrun kalibr kalibr_calibrate_cameras --bag camera_calibration.bag --topics /camera1 /camera2 --models pinhole-radtan pinhole-radtan --target apriltag.yaml --bag-from-to 0 75  --approx-sync 0.1
```
After this step, you should get the camera intrinsic. Please save the file in your desired location for future usage.

#### 5. IMU Calibration.
If you are using D435i's IMU, you have an extra step to do: calibrate IMU and write the data into the camera device. It's very easy and important. You just need to follow the official instruction [PDF](https://www.intelrealsense.com/wp-content/uploads/2019/07/Intel_RealSense_Depth_D435i_IMU_Calibration.pdf).
Necessary tools and code can be found on [librealsense github](https://github.com/IntelRealSense/librealsense).

An esay way to verify your calibration quality is to check the realsense viewer. If the value in x and z axis is close to 0 and y axis is close to 9.8, that means your calibration is okay.

![Screenshot from 2022-04-10 16-00-03](https://user-images.githubusercontent.com/55560905/162637582-218046d8-2697-4444-96e6-0f6fa66beb7f.png)

#### 6. Estimate IMU Noise Model.
Now, we need to estimate noise model for IMU. You can find some explanation of the parameters [here](https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model).
Use the repo [imu_utils](https://github.com/gaowenliang/imu_utils) which depends on [code_utils](https://github.com/gaowenliang/code_utils) to obtain the IMU noise model.

First, record the IMU data for more than 10 minutes while making it stationary. Then, Please refer to the launch file [here](https://github.com/Zhefan-Xu/camera-imu-calibration-guide/blob/main/d435i_imu_calibration.launch) to run the esimation process. Finally, play the rosbag file and it will start estimation.

Save the obtained data into this [format](https://github.com/Zhefan-Xu/camera-imu-calibration-guide/blob/main/imu.yaml).
 

#### 6. Camera-IMU Calibration.
Finally, we can perform the calibration (This step will take several hours).
Camera-IMU calibration (stereo):
```
rosrun kalibr kalibr_calibrate_imu_camera --bag steoreo_calibration.bag --cam camera_calibration_result/camchain-camera_calibration.yaml --imu imu_calibration_result/imu.yaml --target apriltag.yaml --bag-from-to 5 105
```

Camera-IMU calibration (monocular/color):
```
rosrun kalibr kalibr_calibrate_imu_camera --bag color_calibration.bag --cam camera_calibration_result/camchain-camera_calibration.yaml --imu imu_calibration_result/imu.yaml --target apriltag.yaml --bag-from-to 5 105
```
You should see a very small reprojection error (<1.0):

![Screenshot from 2022-04-10 16-05-02](https://user-images.githubusercontent.com/55560905/162637675-5df47c85-e63f-486e-89a4-3f4bba91fe26.png)
