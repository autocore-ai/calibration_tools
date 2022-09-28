import numpy as np
import cv2

def findEllipses(contours,min_area=5,max_area=1000,sim_thres=1e-6):
    # contours, _ = cv2.findContours(edges.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    # ellipseMask = np.zeros(edges.shape, dtype=np.uint8)
    # contourMask = np.zeros(edges.shape, dtype=np.uint8)
    centers = []
    pi_4 = np.pi * 4
    inds = []
    for i, contour in enumerate(contours):
        if len(contour) < 4:
            continue

        area = cv2.contourArea(contour)
        if area <= min_area or area>=max_area:  # skip ellipses smaller then 10x10
            continue

        arclen = cv2.arcLength(contour, True)
        circularity = (pi_4 * area) / (arclen * arclen)
        ellipse = cv2.fitEllipse(contour)
        poly = cv2.ellipse2Poly((int(ellipse[0][0]), int(ellipse[0][1])), (int(ellipse[1][0] / 2), int(ellipse[1][1] / 2)), int(ellipse[2]), 0, 360, 5)

        # if contour is circular enough
        if circularity > 0.6:
            # continue
            similarity = cv2.matchShapes(poly.reshape((poly.shape[0], 1, poly.shape[1])), contour, cv2.CONTOURS_MATCH_I2, 0)
            if similarity >=sim_thres:
                inds.append(i)
                centers.append(ellipse[0])
    return inds,centers

def get_ellipse_corners(gray,rows,cols,symmetry):
    thresh = cv2.threshold(gray,100, 255,cv2.THRESH_BINARY_INV)[1]
    contours, hierarchy = cv2.findContours(thresh,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)  
    if len(contours)<rows*cols:
        return False, []
    einds,centers = findEllipses(contours)
    if len(einds)<rows*cols:
        return False, []
    centers = np.array(centers)
    ellipses = []
    for e in einds:
        ellipses.append(contours[e])
    out = cv2.drawContours(gray,ellipses,-1,(0,0,255),1)
    ret, corners = cv2.findCirclesGrid(out, (rows,cols),centers, flags = symmetry)
    return ret,corners

def get_circle_corners(img,gray,rows,cols,symmetry):
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6)
    blobParams = cv2.SimpleBlobDetector_Params()
    blobParams.minThreshold = 8
    blobParams.maxThreshold = 255

    # Filter by Area.
    blobParams.filterByArea = True
    blobParams.minArea = 100 # minArea may be adjusted to suit for your experiment
    blobParams.maxArea = 2500    # maxArea may be adjusted to suit for your experiment

    # Filter by Circularity
    blobParams.filterByCircularity = True
    blobParams.minCircularity = 0.75

    # Filter by Convexity
    blobParams.filterByConvexity = True
    blobParams.minConvexity = 0.87

    # Filter by Inertia
    blobParams.filterByInertia = True
    blobParams.minInertiaRatio = 0.3

    # Create a detector with the parameters
    blobDetector = cv2.SimpleBlobDetector_create(blobParams)
    keypoints = blobDetector.detect(gray)
    im_with_keypoints = cv2.drawKeypoints(img, keypoints, np.array([]), (0,255,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    im_with_keypoints_gray = cv2.cvtColor(im_with_keypoints, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findCirclesGrid(im_with_keypoints, (rows,cols), None, flags = symmetry)   
    if ret:
        corners2 = cv2.cornerSubPix(im_with_keypoints_gray, corners, (2,2), (-1,-1), criteria)
        if [corners2]:
            corners = corners2
    return ret,corners

def get_chessboard_corners(gray,rows,cols):
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6)
    ret, corners = cv2.findChessboardCorners(gray, (rows, cols), None)
    if ret:
        corners2 = cv2.cornerSubPix(gray, corners, (3, 3), (-1, -1), criteria)
        if [corners2]:
            corners = corners2
    return ret, corners

def add_keypoints(gray,statistics,kps,center_thr,theta_thr,v_thr):
    meanv = 0
    for t in kps:
        meanv += gray[int(t[1]),int(t[0])]
    meanv = meanv/len(kps)
    delta = kps[0]-kps[-1]
    theta = np.rad2deg(np.arctan(delta[1]/delta[0]))
    meanc = kps.mean(axis=0)

    mind = np.inf
    minv = np.inf
    mint = np.inf
    if len(statistics)==0:
        statistics.append([meanc,theta,meanv])
        return True
    for s in statistics:
        dist = np.linalg.norm(s[0]-meanc)
        if dist<mind:
            mind = dist
        dt = np.abs(theta-s[1])
        if dt<mint:
            mint = dt
        dv = np.abs(s[2]-meanv)
        if dv<minv:
            minv = dv
    if mind>center_thr or mint>theta_thr or minv > v_thr:
        statistics.append([meanc,theta,meanv])
        return True
    return False