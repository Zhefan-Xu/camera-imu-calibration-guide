# VIO_SLAM


#### Camera Teminology:
 - **Intrinsic Matrix**: The matrix that transform 3D points from camera 3D coordinate frame to 2D image plane. Parameters (focal length, offset): fx, fy, px, py.
 - **Extrinsic Matrix**: The matrix that transform 3D points from other coordinate frame to camera 3D coodinate frame. 
 - **Distortion Parameters**: TODO

#### Camera Calibration:
 - **Single camera calibration**: To find intrinsic matrix and distortion parameters.
 - **Multiple-camera calibration**: In addition to instrinsic and distortion, we also want to obtain extrinsic matrix.
 - **Camera-IMU calibration**: Find the transformation between camera and IMU.
