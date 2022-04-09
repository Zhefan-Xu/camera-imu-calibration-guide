# VIO_SLAM


#### Camera Teminology:
 - **Intrinsic Matrix**: The matrix that transform 3D points from camera 3D coordinate frame to 2D image plane. Parameters (focal length, offset): fx, fy, px, py.
 - **Extrinsic Matrix**: The matrix that transform 3D points from other coordinate frame to camera 3D coodinate frame. Note: Usually this refers to the transform between camera and camera (in multiple-camera system) or transform between camera and IMU (in visual inertial system). This is represented by the transformation matrix (4x4, rotation and translation).
 - **Distortion Parameters**: Camera lens can cause distortion in image. More details on parameters about [camera distortion](https://ori.codes/artificial-intelligence/camera-calibration/camera-distortions/).

#### Calibration Target:
 - **Checkerboard**:
   - *Target row/column*: Number of INNER boxes in row/column.
   - *Size*: Size of each boxes.
 
     <img src="https://user-images.githubusercontent.com/55560905/162593666-e2a997c7-1dc2-4203-aaf4-72dbb8b5e61b.png" width="300">
 
 - **Aprilgrid**:
   - *Target row/colum*: Number of INNER boxes in row/column.
   - *Size*: Size of each boxes.
   - *Spacing*: Distance between two boxes in mm. (sometimes it is represented by a ratio).
   
     <img src="https://user-images.githubusercontent.com/55560905/162593770-031a92b6-2994-4da0-9823-dd569ca4de69.png" width="300">
   
#### Camera Calibration:
 - **Single camera calibration**: To find intrinsic matrix and distortion parameters.
 - **Multiple-camera calibration**: In addition to instrinsic and distortion, we also want to obtain extrinsic matrix.
 - **Camera-IMU calibration**: Find the transformation between camera and IMU.


Camera calibration:
```
rosrun kalibr kalibr_calibrate_cameras --bag camera_calibration.bag --topics /camera1 /camera2 --models pinhole-radtan pinhole-radtan --target apriltag.yaml --bag-from-to 0 75  --approx-sync 0.1
```

Camera-IMU calibration:
```
rosrun kalibr kalibr_calibrate_imu_camera --bag calibration.bag --cam camera_calibration_result/camchain-camera_calibration.yaml --imu imu_calibration_result/imu.yaml --target apriltag.yaml --bag-from-to 5 105
```
