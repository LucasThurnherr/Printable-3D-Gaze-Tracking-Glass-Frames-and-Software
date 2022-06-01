from pathlib import Path
import glob
from PIL import Image
import numpy as np
import cv2


# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    
# prepare object points
nx = 9 #number of inside corners in x
ny = 6 #number of inside corners in y

# Prepare obj points, like (0, 0, 0), (1, 0, 0), (2, 0, 0)....., (8, 5, 0)
objp = np.zeros((nx*ny,3), np.float32)
objp[:,:2] =  np.mgrid[0:nx,0:ny].T.reshape(-1,2) # x,y coordinates 

# Arrays to store object points and image points from all the images 
objpoints = [] # 3D points in real world space
imgpoints = [] # 2D points in image plane

# Make a list of calibration images
fname = '/home/pi/Calibration-Pics/tesImage7.jpg' #Taking the best picture resulting in the best mean error, which results in the best distortion coeff. and intrinsic matrix
img = cv2.imread(fname)
# Convert to grayscale
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# Find the chessboard corners
ret, corners = cv2.findChessboardCorners(gray, (nx, ny), None)
# If found, draw corners
if ret == True:
    imgpoints.append(corners)
    objpoints.append(objp)
    # Draw and display the corners
    cv2.drawChessboardCorners(img, (nx, ny), corners, ret)
    cv2.imshow("Corners",img)
    cv2.waitKey(0)
cv2.destroyAllWindows()

h,w = img.shape[:2]
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
# undistort
dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
# crop the image
x, y, w, h = roi
dst = dst[y:y+h, x:x+w]
cv2.imwrite('calibratedImage.jpg', dst)

print("Camera matrix : \n")
print(mtx)
print("dist : \n")
print(dist)
print("rvecs : \n")
print(rvecs)
print("tvecs : \n")
print(tvecs)

mean_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
    mean_error += error
print( "total error: {}".format(mean_error/len(objpoints)) )

#Sources for the camera calibration code:
# https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
# https://automaticaddison.com/how-to-perform-camera-calibration-using-opencv/
# https://learnopencv.com/camera-calibration-using-opencv/
# https://opencv24-python-tutorials.readthedocs.io/en/latest/py_tutorials/py_calib3d/py_calibration/py_calibration.html
