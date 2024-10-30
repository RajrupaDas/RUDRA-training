import cv2
import numpy as np
import glob

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
pattern_size = (9, 6)
objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)

objpoints = []
imgpoints_left = []
imgpoints_right = []

images_left = glob.glob('left/*.jpg')
images_right = glob.glob('right/*.jpg')

for img_left, img_right in zip(images_left, images_right):
    imgL = cv2.imread(img_left)
    imgR = cv2.imread(img_right)
    grayL = cv2.cvtColor(imgL, cv2.COLOR_BGR2GRAY)
    grayR = cv2.cvtColor(imgR, cv2.COLOR_BGR2GRAY)
    
    retL, cornersL = cv2.findChessboardCorners(grayL, pattern_size, None)
    retR, cornersR = cv2.findChessboardCorners(grayR, pattern_size, None)
    
    if retL and retR:
        objpoints.append(objp)
        imgpoints_left.append(cornersL)
        imgpoints_right.append(cornersR)

ret, mtx_left, dist_left, mtx_right, dist_right, R, T, _, _ = cv2.stereoCalibrate(objpoints, imgpoints_left, imgpoints_right, None, None, None, None, grayL.shape[::-1], criteria=criteria)

focal_length = mtx_left[0, 0]
baseline = T[0, 0]

np.savez("stereo_calibration_data.npz", mtx_left=mtx_left, dist_left=dist_left, mtx_right=mtx_right, dist_right=dist_right, R=R, T=T, focal_length=focal_length, baseline=baseline)
