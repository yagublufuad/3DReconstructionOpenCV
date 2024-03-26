import numpy as np
import cv2
import glob
import argparse
import rawpy
from R_to_OPK import rotationMatrixToOPK 

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-9)
# Defining a function to detect image and object points
def detect_img_points(left_fn, right_fn, checkerboard_scaling, square_size_m, width, height, w_output, h_output):
    chessboardSize = (width,height)
    objp = np.zeros((chessboardSize[0] * chessboardSize[1], 3), np.float32)
    objp[:,:2] = np.mgrid[0:chessboardSize[0],0:chessboardSize[1]].T.reshape(-1,2)

    objp = objp * square_size_m

    objpoints = [] 
    imgpointsL = [] 
    imgpointsR = []
    
    imagesLeft = sorted(glob.glob(left_fn))
    imagesRight = sorted(glob.glob(right_fn))

    for imgLeft, imgRight in zip(imagesLeft, imagesRight): 
        img_format = imgLeft[-3:]
        if img_format == 'JPG':
            imgL = cv2.imread(imgLeft)
            imgR = cv2.imread(imgRight)
	# OpenCV cannot handle RAW images. Therefore, we use rawpy to be able to use RAW images for camera calibration without saving them.
	# This saves space and time
        elif img_format == 'ARW':
            with rawpy.imread(imgLeft) as raw1:
                imgL = raw1.postprocess(use_camera_wb=True)
                h_temp, w_temp = imgL.shape[:2]
            with rawpy.imread(imgRight) as raw2:
                imgR = raw2.postprocess(use_camera_wb=True)
                h_temp, w_temp = imgR.shape[:2]   
            imgL = imgL[int((h_temp-h_output)/2):int(h_temp-(h_temp-h_output)/2),int((w_temp-w_output)/2):int(w_temp-(w_temp-w_output)/2)]
            imgR = imgR[int((h_temp-h_output)/2):int(h_temp-(h_temp-h_output)/2),int((w_temp-w_output)/2):int(w_temp-(w_temp-w_output)/2)]
        print('imgR shape',imgL.shape)
        grayL = cv2.cvtColor(imgL, cv2.COLOR_BGR2GRAY)
        grayR = cv2.cvtColor(imgR, cv2.COLOR_BGR2GRAY)
        
        iheight, iwidth = grayL.shape[:2] 
        # Resizing the images if the scaling factor is initialized
        if checkerboard_scaling:
            iheight_resize, iwidth_resize = int(iheight/checkerboard_scaling), int(iwidth/checkerboard_scaling)
            grayR = cv2.resize(grayR, (iwidth_resize, iheight_resize))                    
            retR, cornersR = cv2.findChessboardCorners(grayR, chessboardSize, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
            if retR:
                cornersR = cornersR * checkerboard_scaling
            grayL = cv2.resize(grayL, (iwidth_resize, iheight_resize))                    
            retL, cornersL = cv2.findChessboardCorners(grayL, chessboardSize, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
            if retL:
                cornersL = cornersL * checkerboard_scaling
        else:
            retL, cornersL = cv2.findChessboardCorners(grayL, chessboardSize, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
            retR, cornersR = cv2.findChessboardCorners(grayR, chessboardSize, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)

      
        # Here, we considering the image and object point only if the corners are detected on both images
        if retL and retR == True:
            objpoints.append(objp)
            cornersL = cv2.cornerSubPix(grayL, cornersL, (7,7), (-1,-1), criteria)
            imgpointsL.append(cornersL)
            cornersR = cv2.cornerSubPix(grayR, cornersR, (7,7), (-1,-1), criteria)
            imgpointsR.append(cornersR)
    print('imgpoints done...')
    print('image format:',img_format)
    return [objpoints, imgpointsL, imgpointsR, iheight, iwidth]

# A function to perform stereo-calibration
def stereo_calibration(left_xml_file, right_xml_file, left_fn, right_fn, save_xml_file, stereo_map_file, checkerboard_scaling, square_size_m, width, height, w_output, h_output):
    objpoints, imgpointsL, imgpointsR, iheight, iwidth = detect_img_points(left_fn, right_fn, checkerboard_scaling, square_size_m, width=width, height=height, w_output = w_output, h_output = h_output )
    image_size = (iheight, iwidth)
    # Reading camera matrix and distortion coefficients from XML files
    cv_file = cv2.FileStorage(right_xml_file, cv2.FILE_STORAGE_READ)
    cv_file.open(right_xml_file, cv2.FILE_STORAGE_READ)
    K1 = cv_file.getNode('Camera_Matrix').mat()
    D1 = cv_file.getNode('Distortion_Coefficients').mat()
    cv_file.release()
    
    cv_file = cv2.FileStorage(left_xml_file, cv2.FILE_STORAGE_READ)
    cv_file.open(left_xml_file, cv2.FILE_STORAGE_READ)
    K2 = cv_file.getNode('Camera_Matrix').mat()
    D2 = cv_file.getNode('Distortion_Coefficients').mat()
    cv_file.release()

    # Calculating new camera matrices
    newK1, roi_L = cv2.getOptimalNewCameraMatrix(K1, D1, (iwidth, iheight), 1, (iwidth, iheight))
    newK2, roi_R = cv2.getOptimalNewCameraMatrix(K2, D2, (iwidth, iheight), 1, (iwidth, iheight))

    # Defining flags that help to fix the camera intrinsics and only estimate rotation, translation, fundamental, and essential matrices
    flags = 0
    flags |= cv2.CALIB_FIX_INTRINSIC

    criteria_stereo = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 1000, 1e-9)

# Stereo calibration
    retStereo, newK1, D1, newK2, D2, R, T, E, F = cv2.stereoCalibrate(objpoints, imgpointsL, imgpointsR, newK1, D1, newK2, D2, (iwidth, iheight), criteria_stereo, flags)
    omega,phi,kappa = rotationMatrixToOPK(R)
    print('omega',omega,'phi',phi,'kappa',kappa)
    print("Stereo calibration rms: ", retStereo)

# Stereo rectification
    rectifyScale= 1
    R1, R2, P1, P2, Q, roi_L, roi_R= cv2.stereoRectify(newK1, D1, newK2, D2, (iwidth, iheight), R, T, rectifyScale,(0,0), flags=cv2.CALIB_ZERO_DISPARITY)

    cv_file = cv2.FileStorage(save_xml_file, cv2.FILE_STORAGE_WRITE)
    cv_file.write("image_Height", iheight)
    cv_file.write("image_Width", iwidth)
    cv_file.write("Left_Camera_Matrix", K1)
    cv_file.write("Left_Distortion_Coefficients", D1)
    cv_file.write("Right_Camera_Matrix", K2)
    cv_file.write("Right_Distortion_Coefficients", D2)
    cv_file.write("Rotation_Matrix", R)
    cv_file.write("Translation_Matrix", T)
    cv_file.write("Essential_Matrix", E)
    cv_file.write("Fundamental_Matrix", F)
    cv_file.write("R1", R1)
    cv_file.write("R2", R2)
    cv_file.write("ncmtx1", newK1)
    cv_file.write("ncmtx2", newK2)    
    cv_file.write("P1", P1)
    cv_file.write("P2", P2)
    cv_file.write("Q",Q)
    cv_file.release()

    print('Distance between cameras: ',np.linalg.norm(np.asarray(T)))
    # Calculating undistortion and rectification maps
    stereoMapL = cv2.initUndistortRectifyMap(newK1, D1, R1, P1, (iwidth, iheight), cv2.CV_16SC2)
    stereoMapR = cv2.initUndistortRectifyMap(newK2, D2, R2, P2, (iwidth, iheight), cv2.CV_16SC2)

    print("Saving parameters!")
    cv_file = cv2.FileStorage(stereo_map_file, cv2.FILE_STORAGE_WRITE)

    cv_file.write('stereoMapL_x',stereoMapL[0])
    cv_file.write('stereoMapL_y',stereoMapL[1])
    cv_file.write('stereoMapR_x',stereoMapR[0])
    cv_file.write('stereoMapR_y',stereoMapR[1])
    cv_file.write('q', Q)

    cv_file.release()


if __name__ == '__main__':
    
    parser = argparse.ArgumentParser(description='Camera calibration')
    parser.add_argument('--left_xml_file', type=str, required=True, help='path to the XML file of left camera')
    parser.add_argument('--right_xml_file', type=str, required=True, help='path to the XML file of right camera')
    parser.add_argument('--left_fn', type=str, required=True, help='path to the images of left camera')
    parser.add_argument('--right_fn', type=str, required=True, help='path to the images of right camera')
    parser.add_argument('--save_xml_file', type=str, required=True, help='path to save results of stereo-calibration')
    parser.add_argument('--stereo_map_file', type = str, required = True, help = 'path to save stereo map')
    parser.add_argument('--checkerboard_scaling', type=int, required=False, help='factor to resample the images')
    parser.add_argument('--square_size', type=float, required=False, help='chessboard square size')
    parser.add_argument('--width', type=int, required=False, help='width of chessboard')
    parser.add_argument('--height', type=int, required=False, help='height of chessboard height size')
    parser.add_argument('--w_output', type=int, required=False, help='image width for calibration, if the format is RAW')
    parser.add_argument('--h_output', type=int, required=False, help='image height for calibration, if the format is RAW')

    args = parser.parse_args()
    stereo_calibration(args.left_xml_file, args.right_xml_file, args.left_fn, args.right_fn, args.save_xml_file,
                                    args.stereo_map_file, args.checkerboard_scaling, args.square_size, args.width, args.height, args.w_output, args.h_output)