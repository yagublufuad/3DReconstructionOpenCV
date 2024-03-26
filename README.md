# 3D Reconstruction using OpenCV 
This repository is designed to build single-view point clouds from a well-calibrated stereo camera system using the OpenCV library. Single and stereo camera calibration which is an important step of 3D reconstruction is also documented here. We are using OpenCV to perform calibration of single and stereo cameras. Camera calibration is already a well-documented process in different resources. However, the scripts in this repository enable working with the RAW image files alongside the standard JPG images.

## Usage
The scripts have been optimized for the Sony A7 cameras. However, they could be used for the images retrieved with other Sony cameras, as long as, the RAW images are saved as '.ARW' files. '.ARW' is the image format that Sony uses to write RAW image files. Therefore in case of using RAW images that are saved with different extension the scripts should be edited. 

Both `python/single_camera_calibration_chessboard_OpenCV.py` and `python/stereo_calibration_chessboard_OpenCV.py` scripts could be run using the command line. To get help and the short description of input parameters, use `python/single_camera_calibration_chessboard_OpenCV.py -h` or `python/stereo_calibration_chessboard_OpenCV.py -h`. The code reads `python/single_camera_calibration_chessboard_OpenCV.py` all the images under a given directory, performs camera calibration, and writes the camera matrix and distortion coefficients to an OpenCV XML file. The code `python/stereo_calibration_chessboard_OpenCV.py` reads all the images under the given two directories and performs stereo calibration and rectification. The results of stereo calibration and rectification are saved to two different OpenCV XML files.

In order to run the scripts saved under this repository, the following libraries should be installed:
numpy, cv2, rawpy, jupyterlab, matplotlib.

### Examples

Example call from the command line.

#### Running the `python/single_camera_calibration_chessboard_OpenCV.py` script.
```bash
python python/single_camera_calibration_chessboard_OpenCV.py --image_fn "data/SonyA7/Chessboard/Single/left/DSC*.ARW" \
--width 28 --height 17 --checkerboard_scaling 2 --savexml_file \
"data/OpenCV_Calibrations/SonyA7_cam_A_left_OpenCV_CC_fine_checkerboard_portrait_ARW_20Oct2023.xml" \
--square_size 0.01 -w_output 5304 --h_output 7952
```
#### Running the `python/stereo_calibration_chessboard_OpenCV.py` script.
```bash
python python/stereo_calibration_chessboard_OpenCV.py --left_xml_file \ "data/OpenCV_Calibrations/SonyA7_cam_A_left_OpenCV_CC_fine_checkerboard_portrait_ARW_20Oct2023.xml" \
--right_xml_file "data/OpenCV_Calibrations/SonyA7_cam_B_right_OpenCV_CC_fine_checkerboard_portrait_ARW_17Apr2023.xml" \
--left_fn "data/SonyA7/Chessboard/Stereo/left/DSC*.ARW" --right_fn "data/SonyA7/Chessboard/Stereo/right/DSC*.ARW" \
--save_xml_file "data/OpenCV_Calibrations/SonyA7_camAB_stereo_OpenCV_CC_fine_checkerboard_portrait_ARW_14Jun2023.xml" \
--stereo_map_file "data/OpenCV_Calibrations/stereoMap_Sony_A7_14.06.2023.xml" --checkerboard_scaling 2 \
--square_size 0.01 --width 28 --height 17 --w_output 5304 --h_output 7952
```
