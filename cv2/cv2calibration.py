import numpy as np
import cv2 as cv
import glob

chessboardSize = (9, 6)
frameSize = (640, 480)
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

objp = np.zeros((chessboardSize[0] * chessboardSize[1], 3), np.float32)
objp[:, :2] = np.mgrid[0 : chessboardSize[0], 0 : chessboardSize[1]].T.reshape(-1, 2)

size_of_chessboard_squares_mm = 23
objp = objp * size_of_chessboard_squares_mm

objpoints = []  # 3d point in real world space
imgpointsL = []  # 2d points in image plane.
imgpointsR = []  # 2d points in image plane.


imagesLeft = sorted(glob.glob(r"C:\Users\ed700\Downloads\cv2-20240403T074759Z-001\cv2\stereoImage\left\*.png"))
imagesRight = sorted(glob.glob(r"C:\Users\ed700\Downloads\cv2-20240403T074759Z-001\cv2\stereoImage\right\*.png"))

for imgLeft, imgRight in zip(imagesLeft, imagesRight):
    imgL = cv.imread(imgLeft)
    imgR = cv.imread(imgRight)
    grayL = cv.cvtColor(imgL, cv.COLOR_BGR2GRAY)
    grayR = cv.cvtColor(imgR, cv.COLOR_BGR2GRAY)

    retL, corner_left = cv.findChessboardCorners(grayL, chessboardSize, None)
    retR, corner_right = cv.findChessboardCorners(grayR, chessboardSize, None)
    
    if retL and retR:
        objpoints.append(objp)

        corner_left = cv.cornerSubPix(grayL, corner_left, (11, 11), (-1, -1), criteria)
        imgpointsL.append(corner_left)
        corner_right = cv.cornerSubPix(grayR, corner_right, (11, 11), (-1, -1), criteria)
        imgpointsR.append(corner_right)

        cv.drawChessboardCorners(imgL, chessboardSize, corner_left, retL)
        cv.imshow("img left", imgL)
        cv.drawChessboardCorners(imgR, chessboardSize, corner_right, retR)
        cv.imshow("img right", imgR)
        cv.waitKey(1000)


cv.destroyAllWindows()


# Calibration
retL, cameraMatrixL, distL, rvecsL, tvecsL = cv.calibrateCamera(
    objpoints, imgpointsL, frameSize, None, None
)
heightL, widthL, channelsL = imgL.shape
newCameraMatrixL, roi_L = cv.getOptimalNewCameraMatrix(
    cameraMatrixL, distL, (widthL, heightL), 1, (widthL, heightL)
)

retR, cameraMatrixR, distR, rvecsR, tvecsR = cv.calibrateCamera(
    objpoints, imgpointsR, frameSize, None, None
)
heightR, widthR, channelsR = imgR.shape
newCameraMatrixR, roi_R = cv.getOptimalNewCameraMatrix(
    cameraMatrixR, distR, (widthR, heightR), 1, (widthR, heightR)
)


flags = 0
flags |= cv.CALIB_FIX_INTRINSIC
criteria_stereo = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

(
    retStereo,
    newCameraMatrixL,
    distL,
    newCameraMatrixR,
    distR,
    rot,
    trans,
    essentialMatrix,
    fundamentalMatrix,
) = cv.stereoCalibrate(
    objpoints,
    imgpointsL,
    imgpointsR,
    newCameraMatrixL,
    distL,
    newCameraMatrixR,
    distR,
    grayL.shape[::-1],
    criteria_stereo,
    flags,
)

rectifyScale = 1
rectL, rectR, projMatrixL, projMatrixR, Q, roi_L, roi_R = cv.stereoRectify(
    newCameraMatrixL,
    distL,
    newCameraMatrixR,
    distR,
    grayL.shape[::-1],
    rot,
    trans,
    rectifyScale,
    (0, 0),
)

stereoMapL = cv.initUndistortRectifyMap(
    newCameraMatrixL, distL, rectL, projMatrixL, grayL.shape[::-1], cv.CV_16SC2
)
stereoMapR = cv.initUndistortRectifyMap(
    newCameraMatrixR, distR, rectR, projMatrixR, grayR.shape[::-1], cv.CV_16SC2
)

print("Parameters saved")
cv_file = cv.FileStorage("stereoMap.xml", cv.FILE_STORAGE_WRITE)

cv_file.write("stereoMapL_x", stereoMapL[0])
cv_file.write("stereoMapL_y", stereoMapL[1])
cv_file.write("stereoMapR_x", stereoMapR[0])
cv_file.write("stereoMapR_y", stereoMapR[1])

cv_file.release()
