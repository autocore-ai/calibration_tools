#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <ros/ros.h>
#include <pcl/common/io.h>
#include <string>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <unistd.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/boundary.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/impl/project_inliers.hpp>
#include <pcl/common/intersections.h>
#include <iostream>
#include <fstream>
#include "yaml-cpp/yaml.h"
#include <math.h>
#include <ceres/ceres.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>

using namespace cv;
using namespace std;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pass_ptr(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

cv::Mat image_;
cv::Size imgsize;
cv::Mat rvec_l_c(3, 1, cv::DataType<double>::type);
cv::Mat tvec_l_c(3, 1, cv::DataType<double>::type);
cv::Mat camera_instrinsics_ = cv::Mat(3, 3, CV_64F);
cv::Mat distortion_coefficients_ = cv::Mat(1, 5, CV_64F);
std::vector<cv::Point3f> board_corner_ps;
std::vector<cv::Point2f> img_corners;

std::string pcd_file; 
std::string img_file;
std::string calibration_file;

double filter_y_left;
double filter_y_right;
double filter_x_forward;
double filter_x_backward;

void param_init(void);
void visual_check(void);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "feature_extract");
    ros::Time::init();
	param_init();
    visual_check();

}

void param_init(void)
{
    ros::NodeHandle nh;
    ros::param::get("img_file", img_file);
    ros::param::get("pcd_file", pcd_file);
    ros::param::get("filter_y_left", filter_y_left);
    ros::param::get("filter_y_right", filter_y_right);
    ros::param::get("calibration_file", calibration_file);
    ros::param::get("filter_x_forward", filter_x_forward);
    ros::param::get("filter_x_backward", filter_x_backward);

    cout<<img_file<<endl;
    cout<<pcd_file<<endl;

     XmlRpc::XmlRpcValue intrin;
    if(!nh.getParam("camera_intrinsic", intrin))
    {
        ROS_ERROR("Failed to get camera intrinsic param, please configurate it first!");
    }
    vector<double> inner_list;
    vector<double> dist_list;
    for(size_t i = 0; i < intrin["innner"].size(); ++i)
    {
        XmlRpc::XmlRpcValue tmp_value = intrin["innner"][i];
        if(tmp_value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
        {
            inner_list.push_back(double(tmp_value));
        }
    }
    for(size_t i = 0; i < intrin["dist"].size(); ++i)
    {
        XmlRpc::XmlRpcValue tmp_value = intrin["dist"][i];
        if(tmp_value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
        {
            dist_list.push_back(double(tmp_value));
        }
    }
    Eigen::Matrix3d inner_transpose;
    Eigen::Matrix<double,1,5> distor;
    inner_transpose = Eigen::Map<Eigen::MatrixXd>(inner_list.data(), 3, 3);
    distor = Eigen::Map<Eigen::Matrix<double,1,5>>(dist_list.data(), 1, 5);

    // Matrix directly read from yaml config is actually the transpose,
    // So additional transpose should be done and then map it.
    Eigen::Matrix3d inner = inner_transpose.transpose(); 
    cv::eigen2cv(inner,camera_instrinsics_);
    cv::eigen2cv(distor,distortion_coefficients_);

    cv::FileStorage fs_read(calibration_file,cv::FileStorage::READ);
    fs_read["rvec"]>>rvec_l_c;
    fs_read["tvec"]>>tvec_l_c;
    cout<<"......extrinsic matrix......"<<endl;
    cout<<"......rotation vector......."<<endl;
    cout<< rvec_l_c <<endl;
    cout<<"......translation vector...."<<endl;
    cout<< tvec_l_c <<endl;
    fs_read.release();

}

void visual_check(void)
{        
    //可视化显示验证
    std::vector<cv::Point3f> pts_3d;
    std::vector<cv::Point2f> pts_2d;
    std::vector<cv::Point2f> pts_2d_vis;
	pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_cloud_raw(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pass_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_cloud_no_ground(new pcl::PointCloud<pcl::PointXYZ>);

    image_ = cv::imread(img_file, IMREAD_UNCHANGED);
   
    if(!image_.data)
    {
        std::cout<< "read over ...."<<std::endl;
    }

    if (!pcl::io::loadPCDFile(pcd_file, *lidar_cloud_raw))
    {
        std::string msg = "Sucessfully load pcd, pointcloud size: " +
                      std::to_string(lidar_cloud_raw->size());
        ROS_INFO_STREAM(msg.c_str());
    }

    pcl::PassThrough<pcl::PointXYZ> pass_x;
    pass_x.setInputCloud(lidar_cloud_raw);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(filter_x_backward,filter_x_forward);
    pass_x.filter(*cloud_pass_ptr);

    pcl::PassThrough<pcl::PointXYZ> pass_y;
    pass_y.setInputCloud(cloud_pass_ptr);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(filter_y_right,filter_y_left);//左正右负
    pass_y.filter(*cloud_pass_ptr);

    pcl::PassThrough<pcl::PointXYZ> pass_z;
    pass_z.setInputCloud(cloud_pass_ptr);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(-2.0,2.0);
    pass_z.filter(*cloud_pass_ptr);

    // remove ground
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    // distance threshold [m]
    seg.setDistanceThreshold (0.1);
    // seg.setInputCloud (cloud_ptr);
    seg.setInputCloud (cloud_pass_ptr);    
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        return ;
    }
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud_pass_ptr);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*lidar_cloud_no_ground);

    cout<<"visual"<<endl;
    for (size_t i = 0; i < lidar_cloud_no_ground->size(); i += 1) 
    {
        pcl::PointXYZ point = lidar_cloud_no_ground->points[i];
        float depth = sqrt(pow(point.x, 2) + pow(point.y, 2) + pow(point.z, 2));
        if (depth > 0.5 ) 
        {
            pts_3d.emplace_back(cv::Point3f(point.x, point.y, point.z));
        }
    }
    cv::projectPoints(pts_3d, rvec_l_c, tvec_l_c, camera_instrinsics_, distortion_coefficients_, pts_2d);
 
    int image_rows = image_.rows;
    int image_cols = image_.cols;

    for (size_t i = 0; i < pts_2d.size(); i++) 
    {
        if (pts_2d[i].x >= 0 && pts_2d[i].x < image_cols && pts_2d[i].y >= 0 && pts_2d[i].y < image_rows) 
        {
            // pts_2d_vis.push_back(pts_2d[i]);
            int color = sqrt(pow(pts_3d[i].x, 2) + pow(pts_3d[i].y, 2) + pow(pts_3d[i].z, 2));
            cv::circle(image_, pts_2d[i], 2, CV_RGB(0, 0, color), -1);
        }
    }

    cv::imshow("visual no ground points result", image_);
    cv::waitKey();
    pcl::visualization::CloudViewer viewer_no_ground("ground filter");
	viewer_no_ground.showCloud(lidar_cloud_no_ground);
    pause();
}
