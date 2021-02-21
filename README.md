## Tutorial Codes for SLAM Building Blocks
This is a tutorial code for studying the SLAM(Simultaneous Localization and Mapping).
It aims to make beginners understand basic blocks to build SLAM.
I'm still trying to understand and write a simple code for a practice.
The code is not written from the ground, referenced the code[An Invitation to 3D Vision: A Tutorial for Everyone](https://github.com/sunglok/3dv_tutorial).

### Dependencies
* [OpenCV][] 
  * I used _OpenCV_ for easier implementation and visualization. 

### Examples
* __Calibration__
    * Camera Calibration: To compute the intrinsic parameter _K_, OpenCV provides the function [__calibrateCamera__](https://docs.opencv.org/master/d9/d0c/group__calib3d.html#ga3207604e4b1a1758aa66acb6ed5aa65d). The function is based on _Zhang's Camera Calibration_ to use known real world object(chessboard). It has same approach with DLT(Direct Linear Transformation), but to reduce the complexity, we fix the points' _z_ coordinate to Zero. To perform DLT algorithm, we need at least 4 points per image and 3 image pairs. The correspondance of the points should be known. In this example, we used 8x6 chessboard with size 2.5. By performing Camera Calibration with input images(or video), we can get intrinsic parameter _K_ and distortion factor _q_ as an output.
    * Stereo Camera Calibration: __Epipolar Geometry__ is the geometry of stereo vision. Epipolar Geometry is essential to understandestimating 3D points in the _World Coordinate_ from the image. If you have a stereo camera, you can get __Fundamental matrix F__ or __Essential matrix E(Calibrated)__ relates corresponding points in stereo images. From the Essential matrix, _R_, _t_ can be drived, thus stereo image can be rectified. Image Rectification is neccessary for 3D point estimation by __Triangulation__ using similarity of Triangles. This code is incomplete because the stereo camera should have consistent R, t. In this example, we use only single image pair as an input and result E, F, R, t, and rectified image as an output. 

### Usage
``` 
./camera_calibration [Inputfile path]
./stereo_camera_calibration [Inputfile path]
```

