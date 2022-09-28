import numpy as np
import cv2
import cv2.aruco as aruco
import glob
import os 
import argparse
import yaml
import matplotlib.pyplot as plt
from feature_extraction import *
from utils import *

# CRITERIA=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 200, 1e-6)
# CENTER_THR = 200
# THETA_THR = 8
# VTHR = 15
# NUMCALIB = 15

class IntrinisicCalib:
    def __init__(self,c_thr=200,t_thr=8,v_thr=15,calib_num=15):
#         self.rows = rows
#         self.cols = cols
#         self.spacing = spacing
        self.centerdist_thres = c_thr
        self.theta_thres = t_thr
        self.brightness_thres = v_thr
        self.calib_num = calib_num
        self.statistics = []
        self.key_points = []
        self.control_points = []
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 200, 1e-6)
        
    def calib_offline(self,path,rows,cols,spacing,method,interval=1):
        self.statistics = []
        if method.startswith("A"):
            symmetry = 2
            control_points = gen_Acircle_control_points(rows,cols,spacing)
        elif method =="chess_board":
            control_points = gen_chessboard_control_points(rows,cols,spacing)
            symmetry = 0
        elif method in ["circle","ellipse"]:
            control_points = gen_circle_control_points(rows,cols,spacing)
            symmetry = 1
        else:
            print("method not implement!")
            return 
        success = []
        idx = 0
        key_points = []
        statistics = []
        images = sorted(glob.glob(path+"/*"),key=lambda x:float(x.split(".png")[0].split("/")[-1]))[::interval]
        for name in images:
            img = cv2.imread(name)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            ret,corners = self.get_keypoints(img,gray,method,symmetry)
            if ret:
                if add_keypoints(gray,self.statistics ,corners.reshape(rows*cols,2),
                                 self.centerdist_thres,self.theta_thres,self.brightness_thres):
                    key_points.append(corners)
                    success.append(idx)
            idx+=1
            if len(key_points)>=self.calib_num:
                break
        ref_points = [control_points]*len(key_points)
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(ref_points, key_points, gray.shape[::-1], 
                                                           None, None,criteria=self.criteria)
        errors = []
        for i in range(len(key_points)):
            reproj_points, _ = cv2.projectPoints(control_points, rvecs[i], tvecs[i], mtx,dist)
            meanerror = np.linalg.norm(key_points[i]-reproj_points,axis=1).mean()
            maxerror = np.linalg.norm(key_points[i]-reproj_points,axis=1).max()
            stderror =  np.linalg.norm(key_points[i]-reproj_points,axis=1).std()
            errors.append([meanerror,maxerror,stderror])
        return ret,mtx, dist, rvecs, tvecs,success,np.array(errors)
    
    
    def get_keypoints(self,img,gray,method,symmetry):
        ret = False
        corners = []
        if method.__contains__("circle"):
            ret,corners = get_circle_corners(img,gray,rows,cols,symmetry)
        elif method.__contains__("ellipse"):
            ret,corners = get_ellipse_corners(gray,rows,cols,symmetry)
        elif method=="chess_board":
            ret,corners = get_chessboard_corners(gray,rows,cols)
        return ret,corners

if __name__=='__main__':
    parser = argparse.ArgumentParser(description=" does params fit your calibration method?")
    parser.add_argument('-R','--rows', type=int,default=0) #rows for control points
    parser.add_argument('-C','--cols', type=int,default=0) #cols for control points
    parser.add_argument('-M','--method', default="Acircle") #calibration method: Acircle,Aellipse,chess_board
    parser.add_argument('-d','--spacing', type=float,default=0) #spacing for marker distance.
#     parser.add_argument('-m','--marker_length', type=float,default=0) # marker_length for aruco markers
    parser.add_argument('--save_result',type=int, default=1) # save calibration result.
    parser.add_argument('--save_folder', default="") # result save folder.
    parser.add_argument('--calib_folder',default="") # calibration image folder.
    parser.add_argument('-i','--interval',type=int,default=1) # images sample interval
    parser.add_argument('-n','--calib_num',type=int,default=15) #number of images for calibration
    parser.add_argument('--centerdist_thr',type=float,default=200)
    parser.add_argument('--theta_thr',type=float,default=10) 
    parser.add_argument('--brightness_thr',type=float,default=12)
                         
    args = parser.parse_args() 
    print("start----------------!")
    path = args.calib_folder
    save = args.save_result
    rows = args.rows
    cols = args.cols
    spacing = args.spacing
    method = args.method
    save_folder = args.save_folder
    interval = args.interval
    num = args.calib_num
    c_thr = args.centerdist_thr
    t_thr = args.theta_thr
    v_thr = args.brightness_thr
    if save and not os.path.exists(save_folder):
         os.mkdir(save_folder) 
    model = IntrinisicCalib(c_thr,t_thr,v_thr,num)
    ret,mtx, dist, rvecs, tvecs,success,errors = model.calib_offline(path,rows,cols,spacing,method,interval)
    resiudal = np.round(errors[:,0].mean(),3)
    max_error = np.round(errors[:,1].mean(),3)
    std_error = np.round(errors[:,2].mean(),3)
    print("calibration result:")
    print("resiudal stats: mean:{:.3f}, max:{:.3f}, std:{:.3f} ".format(resiudal,max_error,std_error))
    print("total images: "+str(len(success)))
    print("mtx :",mtx)
    print("distortion: ",dist)
    if save_folder=="":
       save_folder = "."
    with open(save_folder+"/intrinisic.yaml","w") as f:
            data = {"camera_matrix":mtx.tolist(),"dist_coeff":dist.tolist()}
            yaml.dump(data,f)
    if save:
        images = sorted(glob.glob(path+"/*"),key=lambda x:float(x.split(".png")[0].split("/")[-1]))[::interval]
        j = 0
        roi = []
        for i in range(len(images)):
            img = cv2.imread(images[i])
            out = cv2.undistort(img,mtx,dist)
            save_path = save_folder+"/undistort_"+str(i)+".jpg"
            cv2.imwrite(save_path,out)
            if i in success:
                save_path = save_folder+"/reproject_"+str(i)+".jpg"
                out = reproject_control_points(img,rows,cols,mtx, dist, rvecs[j], tvecs[j],spacing,method)
                cv2.imwrite(save_path,out)
                j+=1
