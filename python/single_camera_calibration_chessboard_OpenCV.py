import cv2
import numpy as np
import os
import glob
import argparse
import matplotlib.pyplot as plt
import datetime
import rawpy

'''
This is a modified version of OpenCV code that enable camera calibration
Link to the opencv code: https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
This version allow to work with RAW images without converting them into JPG
'''
def camera_calibration_checkerboard(image_files, width, height, checkerboard_scaling, square_size_m, savexml_file, w_clipped, h_clipped):
    CHECKERBOARD = (width, height)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 1000, 1e-9)
    
    # Defining arrays to store 3D objects points and 2D image points
    objpoints = []
    imgpoints = []

    #preparing 3D object points 
    objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3) , np.float32)
    objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
    objp = objp * square_size_m 
    prev_img_shape = None

    image_fn = glob.glob(image_files)
    image_fn.sort()
    print('image_fn',image_fn)

    if len(image_fn) == 0:
        print('no image files found in %s'%image_fn)

    # Reading the chessboard images
    for fname in image_fn:
        img_format = fname[-3:]
        if img_format == 'JPG':
            img = cv2.imread(fname)
	# OpenCV cannot handle RAW images. Therefore, we use rawpy to be able to use RAW images for camera calibration without saving them.
	# This saves space and time
        elif img_format == 'ARW':
            with rawpy.imread(fname) as raw:
                img = raw.postprocess(use_camera_wb=True)
                h_temp, w_temp = img.shape[:2]
                
                img = img[int((h_temp-h_clipped)/2):int(h_temp-(h_temp-h_clipped)/2),int((w_temp-w_clipped)/2):int(w_temp-(w_temp-w_clipped)/2)]
        else:
            print('unknown image format:',img_format)

        # The next steps requires a grayscaled image.         
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        iheight, iwidth = gray.shape
        # Downsampling the images if the images are too big and processing takes too long
        # The images are resampled if a value for the scaling factor is passed by the user
        if checkerboard_scaling:
            height_resize, width_resize = int(iheight/checkerboard_scaling), int(iwidth/checkerboard_scaling)
            gray = cv2.resize(gray, (width_resize, height_resize))               
            ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
            if ret==True:
                corners = corners * checkerboard_scaling
            else:
                print('no corners found.')
        else:
            ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
            
        if ret == True:
            print('found Chessboard Corners, ', end='', flush=True)
            objpoints.append(objp)
	     # Refining the 2D points
            corners2 = cv2.cornerSubPix(gray, corners, (7,7), (-1,-1), criteria)
            print('found SubPixel corner.',flush=True)
            imgpoints.append(corners2)

    print('Calibrating Camera and saving to XML... ',  flush=True, end='')
    ret, intrinsic_matrix, distortion_coefficient, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    # Saving results of camera calibration to a XML file
    cv_file = cv2.FileStorage(savexml_file, cv2.FILE_STORAGE_WRITE)
    cv_file.write("calibration_Time", str(datetime.datetime.now()))
    cv_file.write("image_Height", iheight)
    cv_file.write("image_Width", iwidth)
    cv_file.write("Camera_Matrix", intrinsic_matrix)
    cv_file.write("Distortion_Coefficients", distortion_coefficient)
    cv_file.release()
    print('done.')
    
    # Calculating reprojection error
    mean_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i],
        tvecs[i], intrinsic_matrix, distortion_coefficient)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
        mean_error += error
    print( "total error: {}".format(mean_error/len(objpoints)) )



if __name__ == '__main__':
    # Help parameters explains the arguments briefly
    parser = argparse.ArgumentParser(description= 'Camera calibration with Chessboard pattern')
    parser.add_argument('--image_fn', type=str, required=True, help="image fn in glob format")
    parser.add_argument('--width', type=int, default=17, required=False, help='chessboard width size, default is 7')
    parser.add_argument('--height', type=int, default=28, required=False, help='chessboard height size, default is 9')
    parser.add_argument('--checkerboard_scaling', type=int, required=False, help='factor to resample the images')
    parser.add_argument('--square_size_m', type=float, default=0.01, required=False, help='Size of squares in m')
    parser.add_argument('--savexml_file', type=str, required=True, help='XML file to save camera calibration results')
    parser.add_argument('--w_output', type=int, required=False, help='image width for calibration, if the format is RAW')
    parser.add_argument('--h_output', type=int, required=False, help='image height for calibration, if the format is RAW ')

    args = parser.parse_args()
    camera_calibration_checkerboard(args.image_fn, args.width, args.height, args.checkerboard_scaling, args.square_size_m, args.savexml_file, args.w_output, args.h_output)