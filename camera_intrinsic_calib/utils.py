import numpy as np
import cv2

def gen_circle_control_points(rows,cols,d):
    control_points=np.zeros((rows*cols,3),np.float32)
    for i in range(cols):
        for j in range(rows):
            control_points[i*rows+j,0]  = d*i
            control_points[i*rows+j,1]  = d*j
    return control_points
        
def gen_Acircle_control_points(rows,cols,d,first_col_zero = True):
    control_points=np.zeros((rows*cols,3),np.float32)
    for i in range(cols):
        for j in range(rows):
            control_points[i*rows+j,0]  = d*i
            if i%2==0:
                control_points[i*rows+j,1] = d*2*j+(1-first_col_zero)*d
            else:
                control_points[i*rows+j,1] = d*first_col_zero+d*2*j
    return control_points

def gen_chessboard_control_points(rows,cols,square_length):
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:rows, 0:cols].T.reshape(-1, 2)*square_length
    return objp

def reproject_control_points(img,rows,cols,mtx, dist, rvec, tvec,d,method):
    if method.startswith("A"):
        control_points = gen_Acircle_control_points(rows,cols,d)
    if method=="chess_board":
        control_points = gen_chessboard_control_points(rows,cols,d)
    elif method in ["circle","ellipse"]:
        control_points = gen_circle_control_points(rows,cols,d)
    reproj_points, _ = cv2.projectPoints(control_points, rvec, tvec, mtx, np.array([]))            
    out = cv2.undistort(img,mtx,dist)
    cv2.drawChessboardCorners(out, (rows, cols), reproj_points, True)
    return out

def calibrate_charuco(path,rows,cols,square_length,marker_length):
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
    board = aruco.CharucoBoard_create(cols, rows, square_length, 
                                          marker_length, aruco_dict)
    arucoParams = aruco.DetectorParameters_create()
    corners_list = []
    ids_list = []
    images = glob.glob(path+"/*")
    for name in images:
        img = cv2.imread(name)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict,parameters=arucoParams)
        resp, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
                    markerCorners=corners,
                    markerIds=ids,
                    image=gray,
                    board=board
                )
        if resp>=20:
            corners_list.append(charuco_corners)
            ids_list.append(charuco_ids)
    ret, mtx, dist, rvecs, tvecs = aruco.calibrateCameraCharuco(
        charucoCorners=corners_list, 
        charucoIds=ids_list, 
        board=board, 
        imageSize=gray.shape, 
        cameraMatrix=None, 
        distCoeffs=None)
    return ret,mtx, dist, rvecs, tvecs