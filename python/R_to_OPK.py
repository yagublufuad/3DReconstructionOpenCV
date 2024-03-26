import numpy as np
import cv2
import math



def rotationMatrixToOPK(R) :


    omega = math.atan2(-R[1,2] , R[2,2])
    phi = math.asin(R[0,2])
    kappa = math.atan2(-R[0,1], R[0,0])

 
    return np.array([omega, phi, kappa])

right_xml_file = "data/OpenCV_Calibrations/SonyA7_camAB_stereo_OpenCV_CC_fine_checkerboard_portrait_ARW_14Jun2023.xml"
cv_file = cv2.FileStorage(right_xml_file, cv2.FILE_STORAGE_READ)
cv_file.open(right_xml_file, cv2.FILE_STORAGE_READ)
R = cv_file.getNode('Rotation_Matrix').mat()

cv_file.release()

omega,phi,kappa = rotationMatrixToEulerAngles(R)
print('omega',omega,'phi',phi,'kappa',kappa)
