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
#include <pcl/filters/voxel_grid.h>
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
#include <opencv2/core/types.hpp>

using namespace cv;
using namespace std;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pass_ptr(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

cv::Mat image_;
cv::Size imgsize;
cv::Mat rvec_c_l(3, 1, cv::DataType<double>::type);
cv::Mat tvec_c_l(3, 1, cv::DataType<double>::type);
cv::Mat camera_instrinsics_ = cv::Mat(3, 3, CV_64F);
cv::Mat distortion_coefficients_ = cv::Mat(1, 5, CV_64F);
std::vector<cv::Point3f> board_corner_ps;
std::vector<cv::Point2f> img_corners;

std::string pcd_file;
std::string img_file;
std::string feature_file;

double filter_x_forward;
double filter_x_backward;
double filter_y_left;
double filter_y_right;

int pair_num_c;
string pair_num;
YAML::Node config;
ofstream fout;

void lidar_feature_extract(void);
void camera_feature_extracrt(void);
void param_init(void);
void calcul_r_t(void);
void visual_pcs(void);
void save_yaml(void);
void re_project_error(void);
void visual_check(void);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "feature_extract");
    ros::Time::init();
	param_init();
    camera_feature_extracrt();
    lidar_feature_extract();
}

void param_init(void)
{
    ros::NodeHandle nh;
    ros::param::get("img_file", img_file);
    ros::param::get("pcd_file", pcd_file);
    ros::param::get("feature_file", feature_file);
    ros::param::get("filter_y_left", filter_y_left);
    ros::param::get("filter_y_right", filter_y_right);
    ros::param::get("filter_x_forward", filter_x_forward);
    ros::param::get("filter_x_backward", filter_x_backward);
    ros::param::get("pair_num_c", pair_num_c);
    
    pair_num = "pair"+to_string(pair_num_c);
    cout<<img_file<<endl;
    cout<<pcd_file<<endl;
    cout<<feature_file<<endl;
    cout<<filter_y_left<<endl;
    cout<<filter_y_right<<endl;
    fout.open(feature_file,ios::app);

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

    config.IsNull();
    fout<< endl;
	
}
void save_yaml(void)
{
    fout << config;
    fout.close();
}

float getSimilarity(const cv::Mat& first,const cv::Mat& second)
{
    double dotSum=first.dot(second);
    double normFirst=cv::norm(first);
    double normSecond=cv::norm(second); 
    if(normFirst!=0 && normSecond!=0){
        return dotSum/(normFirst*normSecond);
    }
}

void lidar_feature_extract(void)
{
    if (!pcl::io::loadPCDFile(pcd_file, *cloud_ptr))
    {
        std::string msg = "Sucessfully load pcd, pointcloud size: " +
                      std::to_string(cloud_ptr->size());
        cout<<msg<<endl;
    }
	else 
    {
        std::string msg = "Unable to load " + pcd_file;
        cout<<msg<<endl;
        exit(-1);
    } 

    pcl::PassThrough<pcl::PointXYZ> pass_x;
    pass_x.setInputCloud(cloud_ptr);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(filter_x_backward,filter_x_forward);
    pass_x.filter(*cloud_pass_ptr);

    pcl::PassThrough<pcl::PointXYZ> pass_y;
    pass_y.setInputCloud(cloud_pass_ptr);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(filter_y_right,filter_y_left); //left + / right -
    pass_y.filter(*cloud_pass_ptr);

    pcl::PassThrough<pcl::PointXYZ> pass_z;
    pass_z.setInputCloud(cloud_pass_ptr);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(-3.0,2.0);
    pass_z.filter(*cloud_pass_ptr);
    
    // ground remove
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    // distance threshold [m]
    seg.setDistanceThreshold (0.5);
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
    extract.filter (*cloud_filtered);

    //chessboard detection
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_chess_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ModelCoefficients::Ptr coefficients_chess (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_chess (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg_chess;
    // Optional
    seg_chess.setOptimizeCoefficients (true);
    // Mandatory
    seg_chess.setModelType (pcl::SACMODEL_PLANE);
    seg_chess.setMethodType (pcl::SAC_RANSAC);
    // distance threshold [m]
    seg_chess.setDistanceThreshold (0.06);
    // seg.setInputCloud (cloud_ptr);
    seg_chess.setInputCloud (cloud_filtered);
    seg_chess.segment (*inliers_chess, *coefficients_chess);
    if (inliers->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        return ;
    }
    
    pcl::ExtractIndices<pcl::PointXYZ> extract_chess;
    extract_chess.setInputCloud (cloud_filtered);
    extract_chess.setIndices (inliers_chess);
    extract_chess.setNegative (false);
    extract_chess.filter (*cloud_chess_ptr);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_chess_ds_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_chess_ds_ptr = cloud_chess_ptr;

    // fit plane
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_chess_projected(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(cloud_chess_ds_ptr);
    proj.setModelCoefficients(coefficients_chess);
    proj.filter(*cloud_chess_projected);

    // chessboard edge extract
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud<pcl::Normal>);

    cloud = cloud_chess_projected;
    auto t1 = ros::Time::now();

    ROS_INFO("NormalEstimation...");
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normEst;
    normEst.setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr(cloud)); 
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    normEst.setSearchMethod(kdtree);
    normEst.setKSearch(30);
    normEst.compute(*normal); // save normal estimation results
    auto t2 = ros::Time::now();
    std::cout <<"NormalEstimation time: " <<(t2 - t1).toSec() << "[sec]" << std::endl;

    ROS_INFO("BoundaryEstimation...");
    pcl::PointCloud<pcl::Boundary>::Ptr boundaries(new pcl::PointCloud<pcl::Boundary>); // init boundary ptr
    boundaries->resize(cloud->size()); // init boundary size
    pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundary_estimation; // init BoundaryEstimation
    boundary_estimation.setInputCloud(cloud);
    boundary_estimation.setInputNormals(normal);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree_ptr(new pcl::search::KdTree<pcl::PointXYZ>); 
    boundary_estimation.setSearchMethod(kdtree_ptr); 
    boundary_estimation.setKSearch(30);
    boundary_estimation.setAngleThreshold(M_PI * 0.6);
    boundary_estimation.compute(*boundaries); // compute cloud edge, save results in boundaries
    
    auto t3 = ros::Time::now();
    std::cout <<"BoundaryEstimation time: " <<(t3 - t1).toSec() << "[sec]" << std::endl;

    // visulization
    ROS_INFO("cloud_visual...");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_visual(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_chess_board_edge(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_visual->resize(cloud->size());
    int ii = 0;
    for(size_t i = 0; i < cloud->size(); i++)
    {
        cloud_visual->points[i].x = cloud->points[i].x;
        cloud_visual->points[i].y = cloud->points[i].y;
        cloud_visual->points[i].z = cloud->points[i].z;
        if(boundaries->points[i].boundary_point != 0)
        {
            cloud_visual->points[i].r = 0;
            cloud_visual->points[i].g = 255;
            cloud_visual->points[i].b = 0;
            pcl::PointXYZ point;
            point.x = cloud->points[i].x;
            point.y = cloud->points[i].y;
            point.z = cloud->points[i].z;
            cloud_chess_board_edge->push_back(point);
        }
        else
        {
            cloud_visual->points[i].r = 150;
            cloud_visual->points[i].g = 150;
            cloud_visual->points[i].b = 150;
        }
    }
    // 4 edges of chess board
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr chess_board_edge_1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr chess_board_edge_2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr chess_board_edge_3(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr chess_board_edge_4(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr chess_board_edge_1_ots(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr chess_board_edge_2_ots(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr chess_board_edge_3_ots(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::ModelCoefficients::Ptr coefficients_1 (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_1 (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_2 (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_2 (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_3 (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_3 (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_4 (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_4 (new pcl::PointIndices);

    pcl::SACSegmentation<pcl::PointXYZ> seg_line;
    pcl::ExtractIndices<pcl::PointXYZ> extract_edge;

    seg_line.setOptimizeCoefficients (true);
    // Mandatory
    seg_line.setModelType (pcl::SACMODEL_LINE);
    seg_line.setMethodType (pcl::SAC_RANSAC);
    // distance threshold [m]
    seg_line.setDistanceThreshold (0.05);
    
    // 1st edge
    
    seg_line.setInputCloud (cloud_chess_board_edge);    
    seg_line.segment (*inliers_1, *coefficients_1);
    
    if (inliers_1->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        return ;
    }
    extract_edge.setInputCloud (cloud_chess_board_edge);
    extract_edge.setIndices (inliers_1);
    extract_edge.setNegative (false);
    extract_edge.filter (*chess_board_edge_1);

    extract_edge.setNegative (true);
    extract_edge.filter (*chess_board_edge_1_ots);
    
    // 2nd edge
    seg_line.setInputCloud (chess_board_edge_1_ots);    
    seg_line.segment (*inliers_2, *coefficients_2);
    
    if (inliers_2->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        return ;
    }
    extract_edge.setInputCloud (chess_board_edge_1_ots);
    extract_edge.setIndices (inliers_2);
    extract_edge.setNegative (false);
    extract_edge.filter (*chess_board_edge_2);

    extract_edge.setNegative (true);
    extract_edge.filter (*chess_board_edge_2_ots);

    // 3rd edge
    seg_line.setInputCloud (chess_board_edge_2_ots);    
    seg_line.segment (*inliers_3, *coefficients_3);
    
    if (inliers_3->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        return ;
    }
    extract_edge.setInputCloud (chess_board_edge_2_ots);
    extract_edge.setIndices (inliers_3);
    extract_edge.setNegative (false);
    extract_edge.filter (*chess_board_edge_3);

    extract_edge.setNegative (true);
    extract_edge.filter (*chess_board_edge_3_ots);

    // 4rd edge
    seg_line.setInputCloud (chess_board_edge_3_ots);    
    seg_line.segment (*inliers_4, *coefficients_4);
    
    if (inliers_4->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        return ;
    }
    extract_edge.setInputCloud (chess_board_edge_3_ots);
    extract_edge.setIndices (inliers_4);
    extract_edge.setNegative (false);
    extract_edge.filter (*chess_board_edge_4);

    cout<<"coefficients_chess:"<<*coefficients_chess<<endl;

    cout<<"coefficient_1:"<<*coefficients_1<<endl;
    cout<<"coefficient_2:"<<*coefficients_2<<endl;
    cout<<"coefficient_3:"<<*coefficients_3<<endl;
    cout<<"coefficient_4:"<<*coefficients_4<<endl;
    
    Eigen::Vector4f Point_l;
    pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ basic_point; // intersection points stored here

    vector<double> vec_c_1  = {coefficients_1->values[3],coefficients_1->values[4],coefficients_1->values[5]};
    cv::Mat vec_1(vec_c_1);
    
    vector<double> vec_c_2  = {coefficients_2->values[3],coefficients_2->values[4],coefficients_2->values[5]};
    cv::Mat vec_2(vec_c_2);
    
    vector<double> vec_c_3  = {coefficients_3->values[3],coefficients_3->values[4],coefficients_3->values[5]};
    cv::Mat vec_3(vec_c_3);
    
    vector<double> vec_c_4  = {coefficients_4->values[3],coefficients_4->values[4],coefficients_4->values[5]};
    cv::Mat vec_4(vec_c_4);

    cout<< "vec_1 :"<<vec_1<<endl; 
    cout<< "vec_2 :"<<vec_2<<endl; 
    cout<< "vec_3 :"<<vec_3<<endl; 
    cout<< "vec_4 :"<<vec_4<<endl; 
 
    float sim_12 = getSimilarity(vec_1,vec_2);
    float sim_13 = getSimilarity(vec_1,vec_3);
    float sim_14 = getSimilarity(vec_1,vec_4);

    float sim_23 = getSimilarity(vec_2,vec_3);
    float sim_24 = getSimilarity(vec_2,vec_4);
    float sim_34 = getSimilarity(vec_3,vec_4);
    
    float sim_21 = getSimilarity(vec_2,vec_1);
    float sim_41 = getSimilarity(vec_4,vec_1);
    

    cout<<"sim_12 :"<<sim_12<<endl;
    cout<<"sim_13 :"<<sim_13<<endl;
    cout<<"sim_14 :"<<sim_14<<endl;
    
    cout<<"sim_23 :"<<sim_23<<endl;
    cout<<"sim_24 :"<<sim_24<<endl;

    cout<<"sim_34 :"<<sim_34<<endl;
    cout<<"sim_21 :"<<sim_21<<endl;
    cout<<"sim_41 :"<<sim_41<<endl;
    
    if(fabs(sim_12) > 0.9)// 1 parallel to 2, 3 parallel to 4; 13 14 23 24
    {
        if (pcl::lineWithLineIntersection(*coefficients_1, *coefficients_3, Point_l)) 
        {
            basic_point.x = Point_l[0];
            basic_point.y = Point_l[1];
            basic_point.z = Point_l[2];
            basic_cloud_ptr->points.push_back(basic_point);
            cout<<"basic_point: "<<endl;
            cout<< basic_point<< endl;
        }
        if (pcl::lineWithLineIntersection(*coefficients_1, *coefficients_4, Point_l)) 
        {
            basic_point.x = Point_l[0];
            basic_point.y = Point_l[1];
            basic_point.z = Point_l[2];
            basic_cloud_ptr->points.push_back(basic_point);
            cout<<"basic_point: "<<endl;
            cout<< basic_point<< endl;
        }
        if (pcl::lineWithLineIntersection(*coefficients_2, *coefficients_3, Point_l)) 
        {
            basic_point.x = Point_l[0];
            basic_point.y = Point_l[1];
            basic_point.z = Point_l[2];
            basic_cloud_ptr->points.push_back(basic_point);
            cout<<"basic_point: "<<endl;
            cout<< basic_point<< endl;
        }
        if (pcl::lineWithLineIntersection(*coefficients_2, *coefficients_4, Point_l)) 
        {
            basic_point.x = Point_l[0];
            basic_point.y = Point_l[1];
            basic_point.z = Point_l[2];
            basic_cloud_ptr->points.push_back(basic_point);
            cout<<"basic_point: "<<endl;
            cout<< basic_point<< endl;
        }
    }
    else if(fabs(sim_13) > 0.9)// 1 parallel to 3, 2 parallel to 4; 12 14 32 34
    {
        if (pcl::lineWithLineIntersection(*coefficients_1, *coefficients_2, Point_l)) 
        {
            basic_point.x = Point_l[0];
            basic_point.y = Point_l[1];
            basic_point.z = Point_l[2];
            basic_cloud_ptr->points.push_back(basic_point);
            cout<<"basic_point: "<<endl;
            cout<< basic_point<< endl;
        }
        if (pcl::lineWithLineIntersection(*coefficients_1, *coefficients_4, Point_l)) 
        {
            basic_point.x = Point_l[0];
            basic_point.y = Point_l[1];
            basic_point.z = Point_l[2];
            basic_cloud_ptr->points.push_back(basic_point);
            cout<<"basic_point: "<<endl;
            cout<< basic_point<< endl;
        }
        if (pcl::lineWithLineIntersection(*coefficients_3, *coefficients_2, Point_l)) 
        {
            basic_point.x = Point_l[0];
            basic_point.y = Point_l[1];
            basic_point.z = Point_l[2];
            basic_cloud_ptr->points.push_back(basic_point);
            cout<<"basic_point: "<<endl;
            cout<< basic_point<< endl;
        }
        if (pcl::lineWithLineIntersection(*coefficients_3, *coefficients_4, Point_l)) 
        {
            basic_point.x = Point_l[0];
            basic_point.y = Point_l[1];
            basic_point.z = Point_l[2];
            basic_cloud_ptr->points.push_back(basic_point);
            cout<<"basic_point: "<<endl;
            cout<< basic_point<< endl;
        }
    }
    else if(fabs(sim_14) > 0.9)// 1 paraller to 4, 2 parallel to 3; 12 13 42 43
    {
        if (pcl::lineWithLineIntersection(*coefficients_1, *coefficients_2, Point_l)) 
        {
            basic_point.x = Point_l[0];
            basic_point.y = Point_l[1];
            basic_point.z = Point_l[2];
            basic_cloud_ptr->points.push_back(basic_point);
            cout<<"basic_point: "<<endl;
            cout<< basic_point<< endl;
        }
        if (pcl::lineWithLineIntersection(*coefficients_1, *coefficients_3, Point_l)) 
        {
            basic_point.x = Point_l[0];
            basic_point.y = Point_l[1];
            basic_point.z = Point_l[2];
            basic_cloud_ptr->points.push_back(basic_point);
            cout<<"basic_point: "<<endl;
            cout<< basic_point<< endl;
        }
        if (pcl::lineWithLineIntersection(*coefficients_4, *coefficients_2, Point_l)) 
        {
            basic_point.x = Point_l[0];
            basic_point.y = Point_l[1];
            basic_point.z = Point_l[2];
            basic_cloud_ptr->points.push_back(basic_point);
            cout<<"basic_point: "<<endl;
            cout<< basic_point<< endl;
        }
        if (pcl::lineWithLineIntersection(*coefficients_4, *coefficients_3, Point_l)) 
        {
            basic_point.x = Point_l[0];
            basic_point.y = Point_l[1];
            basic_point.z = Point_l[2];
            basic_cloud_ptr->points.push_back(basic_point);
            cout<<"basic_point: "<<endl;
            cout<< basic_point<< endl;
        }
    }
    else{
        cout<< "can not find cornors" <<endl;
    }
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_visual_edge(new pcl::PointCloud<pcl::PointXYZRGBA>);
     
    for(size_t i = 0; i < cloud_ptr->size(); i++)
    {
        pcl::PointXYZRGBA point;
        point.x = cloud_ptr->points[i].x;
        point.y = cloud_ptr->points[i].y;
        point.z = cloud_ptr->points[i].z;
        point.r = 100;
        point.g = 100;
        point.b = 100;
        point.a = 80;
        cloud_visual_edge->push_back(point);
    }

    for(size_t i = 0; i < cloud_pass_ptr->size(); i++)
    {
        pcl::PointXYZRGBA point;
        point.x = cloud_pass_ptr->points[i].x;
        point.y = cloud_pass_ptr->points[i].y;
        point.z = cloud_pass_ptr->points[i].z;
        point.r = 0;
        point.g = 80;
        point.b = 80;
        point.a = 100;
        cloud_visual_edge->push_back(point);
    }

    for(size_t i = 0; i < cloud_filtered->size(); i++)
    {
        pcl::PointXYZRGBA point;
        point.x = cloud_filtered->points[i].x;
        point.y = cloud_filtered->points[i].y;
        point.z = cloud_filtered->points[i].z;
        point.r = 10;
        point.g = 10;
        point.b = 0;
        point.a = 100;
        cloud_visual_edge->push_back(point);
    }

    for(size_t i = 0; i < cloud->size(); i++)
    {
        pcl::PointXYZRGBA point;
        point.x = cloud->points[i].x;
        point.y = cloud->points[i].y;
        point.z = cloud->points[i].z;
        point.r = 150;
        point.g = 150;
        point.b = 150;
        point.a = 180;

        cloud_visual_edge->push_back(point);
    }
  
    for(size_t i = 0; i < chess_board_edge_1->size(); i++)
    {
        pcl::PointXYZRGBA point;
        point.x = chess_board_edge_1->points[i].x;
        point.y = chess_board_edge_1->points[i].y;
        point.z = chess_board_edge_1->points[i].z;
        point.r = 0;
        point.g = 255;
        point.b = 0;
        point.a = 220;

        cloud_visual_edge->push_back(point);
    }

    for(size_t i = 0; i < chess_board_edge_2->size(); i++)
    {
        pcl::PointXYZRGBA point;
        point.x = chess_board_edge_2->points[i].x;
        point.y = chess_board_edge_2->points[i].y;
        point.z = chess_board_edge_2->points[i].z;
        point.r = 255;
        point.g = 0;
        point.b = 0;
        point.a = 220;

        cloud_visual_edge->push_back(point);
    }

    for(size_t i = 0; i < chess_board_edge_3->size(); i++)
    {
        pcl::PointXYZRGBA point;
        point.x = chess_board_edge_3->points[i].x;
        point.y = chess_board_edge_3->points[i].y;
        point.z = chess_board_edge_3->points[i].z;
        point.r = 0;
        point.g = 0;
        point.b = 255;
        point.a = 220;

        cloud_visual_edge->push_back(point);
    }

    for(size_t i = 0; i < chess_board_edge_4->size(); i++)
    {
        pcl::PointXYZRGBA point;
        point.x = chess_board_edge_4->points[i].x;
        point.y = chess_board_edge_4->points[i].y;
        point.z = chess_board_edge_4->points[i].z;
        point.r = 255;
        point.g = 255;
        point.b = 0;
        point.a = 220;

        cloud_visual_edge->push_back(point);
    }

    for(size_t i = 0; i < basic_cloud_ptr->size(); i++)
    {
        pcl::PointXYZRGBA point;
        point.x = basic_cloud_ptr->points[i].x;
        point.y = basic_cloud_ptr->points[i].y;
        point.z = basic_cloud_ptr->points[i].z;
        point.r = 255;
        point.g = 255;
        point.b = 255;
        point.a = 255;
    
        cloud_visual_edge->push_back(point);
    }
   

    double y_max = -99;
    double y_min = 99;
    double z_max = -99;
    double z_min = 99;
    double x_max = -99;
    double x_min = 99;
    
    pcl::PointXYZ board_top_point;
    pcl::PointXYZ board_down_point;
    pcl::PointXYZ board_left_point;
    pcl::PointXYZ board_right_point;

    for(size_t i = 0; i < basic_cloud_ptr->size(); i++)
    {
        if(basic_cloud_ptr->points[i].y > y_max)
        {
            y_max = basic_cloud_ptr->points[i].y;
            board_left_point = basic_cloud_ptr->points[i];
        }

        if(basic_cloud_ptr->points[i].y < y_min)
        {
            y_min = basic_cloud_ptr->points[i].y;
            board_right_point = basic_cloud_ptr->points[i];
        }

        if(basic_cloud_ptr->points[i].z > z_max)
        {
            z_max = basic_cloud_ptr->points[i].z;
            board_top_point = basic_cloud_ptr->points[i];
        }

        if(basic_cloud_ptr->points[i].z < z_min)
        {
            z_min = basic_cloud_ptr->points[i].z;
            board_down_point = basic_cloud_ptr->points[i];
        }
    }

    cout<<"board_left_point"<< board_left_point <<endl;
    cout<<"board_right_point"<< board_right_point <<endl;
    cout<<"board_top_point"<< board_top_point <<endl;
    cout<<"board_down_point"<< board_down_point <<endl;
    	
    config[pair_num]["lidar"]["board_left_point"]["x"] = board_left_point.x;
    config[pair_num]["lidar"]["board_left_point"]["y"] = board_left_point.y;
    config[pair_num]["lidar"]["board_left_point"]["z"] = board_left_point.z;
    
    config[pair_num]["lidar"]["board_right_point"]["x"] = board_right_point.x;
    config[pair_num]["lidar"]["board_right_point"]["y"] = board_right_point.y;
    config[pair_num]["lidar"]["board_right_point"]["z"] = board_right_point.z;
    
    config[pair_num]["lidar"]["board_top_point"]["x"] = board_top_point.x;
    config[pair_num]["lidar"]["board_top_point"]["y"] = board_top_point.y;
    config[pair_num]["lidar"]["board_top_point"]["z"] = board_top_point.z;
    
    config[pair_num]["lidar"]["board_down_point"]["x"] = board_down_point.x;
    config[pair_num]["lidar"]["board_down_point"]["y"] = board_down_point.y;
    config[pair_num]["lidar"]["board_down_point"]["z"] = board_down_point.z;
    
    save_yaml();
    pcl::visualization::CloudViewer viewer_chess_board_edge("view");
    viewer_chess_board_edge.showCloud(cloud_visual_edge);
    pause();
}

bool find4corners(cv::Mat src,std::vector<cv::Point>& corners){
   cv::Mat hsv_img;
   cv::cvtColor(src, hsv_img, cv::COLOR_BGR2HSV);
   cv::Mat mask;
   cv::inRange(hsv_img,cv::Scalar(156,60,80),cv::Scalar(180,255,255),mask);
   std::vector<std::vector<cv::Point>> contours;
   std::vector<cv::Vec4i> hierarchy;
   cv::findContours( mask, contours, hierarchy, 2,1);
   int maxl=4,maxind=0;
   for(int i =0;i<contours.size();i++){
    auto contour = contours[i];
    if (contour.size()>maxl){
        maxind = i;
        maxl = contour.size();
    }
   }
   if(maxl<4) {
    return false;
   }
 
   std::vector<cv::Point> hull;
   cv::convexHull(contours[maxind], hull);
   int xmin_ind,ymin_ind,xmax_ind,ymax_ind;
   int xmin_val=9999,ymin_val=9999,xmax_val=-1,ymax_val=-1;
   for(int i=0;i<hull.size();i++){
      if (hull[i].x<xmin_val){
        xmin_val = hull[i].x;
        xmin_ind = i;
      }
      if (hull[i].y<ymin_val){
        ymin_val = hull[i].y;
        ymin_ind = i;
      }
      if (hull[i].x>xmax_val){
        xmax_val = hull[i].x;
        xmax_ind = i;
      }
      if (hull[i].y>ymax_val){
        ymax_val = hull[i].y;
        ymax_ind = i;
      }
   }
   corners.push_back(hull[xmin_ind]);
   corners.push_back(hull[ymin_ind]);
   corners.push_back(hull[xmax_ind]);
   corners.push_back(hull[ymax_ind]);
   return true;
}

void camera_feature_extracrt(void)
{
    image_ = cv::imread(img_file, IMREAD_UNCHANGED);
   
    if(!image_.data)
    {
        std::cout<< "read fail ...."<<std::endl;
    }

    std::vector<cv::Point3f> boardcorners;
    std::vector<cv::Point2f> image_bc_points;

    cv::Mat gray_;
    cv::Mat rvec_c(3, 1, cv::DataType<double>::type);
    cv::Mat tvec_c(3, 1, cv::DataType<double>::type);

    // cv::cvtColor(image_, gray, COLOR_BGR2GRAY);
    cv::cvtColor(image_, gray_, CV_BGR2GRAY);
    
    // gray_=image_.clone();
    cv::Size2i patternNum(3,3);
    // cv::Size2i patternNum(7,10);

    cv::Size2d patternSize(0.2,0.2);
    std::vector<cv::Point2f> corners;

    cout<<"image_.rows"<< image_.rows <<endl;
    cout<<"image_.cols"<< image_.cols <<endl;

    cv::Mat res;
    int morph_size = 3;
    int morph_elem = 1; //0 rectangle  1 cross  2 ellipse
    int operation = 3;  //0 erode 1 dilate 2 open 3 close
    
    bool patternfound = cv::findChessboardCorners(gray_, patternNum, corners,
                                                          CV_CALIB_CB_ADAPTIVE_THRESH + CV_CALIB_CB_FILTER_QUADS + CV_CALIB_CB_NORMALIZE_IMAGE + CV_CALIB_CB_FAST_CHECK);

    imgsize.height = image_.rows;
    imgsize.width  = image_.cols;

    cout << camera_instrinsics_ << endl;
    cout << distortion_coefficients_ << endl;

    if (!patternfound)
    {
        cout<<"can not found cornor..."<<endl;
    } 
    else
    {
        cout<<"found cornor..."<<endl;
        cornerSubPix(gray_, corners, Size(11, 11), Size(-1, -1),
                             TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, 0.1));
        cv::drawChessboardCorners(image_, patternNum, corners, patternfound);

        
        std::cout<<"imgsize.height: "<<imgsize.height<<std::endl;
        std::cout<<"imgsize.width: "<<imgsize.width<<std::endl;
        
        double tx, ty; // Translation values
        // Location of board frame origin from the bottom left inner corner of the checkerboard
        tx = (patternNum.height - 1) * patternSize.height / 2;
        ty = (patternNum.width - 1) * patternSize.width / 2;

        std::cout<< "tx: "<< tx <<"  ty:  "<<ty<<std::endl;

        std::vector<cv::Point3f> grid3dpoint;
        
        int scale = 1;
        grid3dpoint.push_back(cv::Point3f(0.2*scale,  0.2*scale,  0.0));
        grid3dpoint.push_back(cv::Point3f(0.2*scale,  0.0*scale,  0.0));
        grid3dpoint.push_back(cv::Point3f(0.2*scale,  -0.2*scale,  0.0));
        grid3dpoint.push_back(cv::Point3f(0.0*scale,  0.2*scale,  0.0));
        grid3dpoint.push_back(cv::Point3f(0.0*scale,  0.0*scale,  0.0));
        grid3dpoint.push_back(cv::Point3f(0.0*scale,  -0.2*scale,  0.0));
        grid3dpoint.push_back(cv::Point3f(-0.2*scale,  0.2*scale,  0.0));
        grid3dpoint.push_back(cv::Point3f(-0.2*scale,  0.0*scale,  0.0));
        grid3dpoint.push_back(cv::Point3f(-0.2*scale,  -0.2*scale,  0.0));

        cv::solvePnP(grid3dpoint, corners, camera_instrinsics_, distortion_coefficients_, rvec_c, tvec_c);
        
        scale = 1;
        boardcorners.push_back(cv::Point3f(0.4*scale+0.05,  0.4*scale+0.05,  0.0));
        boardcorners.push_back(cv::Point3f(0.4*scale+0.05, -0.4*scale-0.05,  0.0));
        boardcorners.push_back(cv::Point3f(-0.4*scale-0.05, 0.4*scale+0.05, 0.0));
        boardcorners.push_back(cv::Point3f(-0.4*scale-0.05, -0.4*scale-0.05, 0.0));
        
        cv::projectPoints(boardcorners, rvec_c, tvec_c, camera_instrinsics_, distortion_coefficients_, image_bc_points);
        cout<<"rvec_c"<<rvec_c<<endl;
        cout<<"tvec_c"<<tvec_c<<endl;
        
        for (int i = 0; i < image_bc_points.size(); i++)
        {
            cv::circle(image_, image_bc_points[i], 5+i*2, CV_RGB(255, 0, 0), -1);

        }

        for (int i = 0; i < corners.size(); i++)
        {
            cv::circle(image_, corners[i], i, CV_RGB(0, 255, 0), -1);
        }

    }

    double x_max = -9999;
    double y_max = -9999;
    double x_min =  9999;
    double y_min =  9999;
    
    cv::Point2f im_top_bc;
    cv::Point2f im_down_bc;
    cv::Point2f im_right_bc;
    cv::Point2f im_left_bc;

    for (int i = 0; i < image_bc_points.size(); i++)
    {
        if(image_bc_points[i].x > x_max)
        {
            x_max = image_bc_points[i].x;
            im_right_bc = image_bc_points[i];
        }

        if(image_bc_points[i].x < x_min)
        {
            x_min = image_bc_points[i].x;
            im_left_bc = image_bc_points[i];
        }

        if(image_bc_points[i].y > y_max)
        {
            y_max = image_bc_points[i].y;
            im_down_bc = image_bc_points[i];
        }

        if(image_bc_points[i].y < y_min)
        {
            y_min = image_bc_points[i].y;
            im_top_bc = image_bc_points[i];
        }

    }    
    cout<< "im_right_bc" << im_right_bc<<endl;
    cout<< "im_left_bc" << im_left_bc<<endl;
    cout<< "im_top_bc" << im_top_bc<<endl;
    cout<< "im_down_bc" << im_down_bc<<endl;
    
        
    config[pair_num]["camera"]["board_left_point"]["x"] = im_left_bc.x;
    config[pair_num]["camera"]["board_left_point"]["y"] = im_left_bc.y;

    config[pair_num]["camera"]["board_right_point"]["x"] = im_right_bc.x;
    config[pair_num]["camera"]["board_right_point"]["y"] = im_right_bc.y;

    config[pair_num]["camera"]["board_top_point"]["x"] = im_top_bc.x;
    config[pair_num]["camera"]["board_top_point"]["y"] = im_top_bc.y;

    config[pair_num]["camera"]["board_down_point"]["x"] = im_down_bc.x;
    config[pair_num]["camera"]["board_down_point"]["y"] = im_down_bc.y;
     
    cv::imshow("image result", image_);
    cv::waitKey();
}
