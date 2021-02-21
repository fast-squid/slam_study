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
    * Camera Calibration: To compute the intrinsic parameter _K_, OpenCV provides the function [__calibrateCamera__](https://docs.opencv.org/master/d9/d0c/group__calib3d.html#ga3207604e4b1a1758aa66acb6ed5aa65d). The function is based on _Zhang's Camera Calibration_ to use known real world object(chessboard). It has same approach with DLT(Direct Linear Transformation), but to reduce the complexity, we fix the points' _z_ coordinate to Zero. To perform DLT algorithm, we need at least 4 points per image and 3 image pairs. The correspondance of the points should be known. In this example, we used 8x6 chessboard with size 2.5.
    * Stereo Camera Calibration: 

'''
./camera\_calibrate [Inputfile path]
'''

