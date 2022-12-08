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
double t[3] = {0., 2., 3.};
double q[4] = {1., 0., .0, 0.};

std::string feature_file;
std::string calibration_file;
std::string calibration_file_tf;
int pairnum;

struct calibrate_cam_lidar_data {
  Eigen::Vector3d lidar_point;
  Eigen::Vector2d pixel_point;
};
std::vector<calibrate_cam_lidar_data> calibrate_data_input;


YAML::Node config;
ofstream fout(feature_file,ios::app);

Eigen::Matrix3d inner;
Eigen::Matrix<double,1,5> distortion;

void param_init(void);
void calcul_r_t(void);
void visual_pcs(void);
void save_yaml(void);
void re_project_error_pnp(void);
void re_project_error_ba(void);

void visual_check(void);
int GetReprojectError(
    const Eigen::Vector3d& t, const Eigen::Quaterniond& q,
    const std::vector<calibrate_cam_lidar_data>& data);
void quat2matrix_and_rotatvec(Eigen::Quaterniond &q,Eigen::Matrix3d &mat3X3,cv::Mat &rvec);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "calculate_r_t");
    ros::Time::init();
	param_init();
    calcul_r_t();
    re_project_error_pnp();
    re_project_error_ba();
}

void param_init(void)
{   
    ros::NodeHandle nh;
    ros::param::get("feature_file", feature_file);
    ros::param::get("calibration_file", calibration_file);
    ros::param::get("calibration_file_tf", calibration_file_tf);
    ros::param::get("pairnum", pairnum);

    cout<<"feature_file: "<<feature_file<<endl;
    cout<<"calibration_file: "<<calibration_file<<endl;
    cout<<"calibration_file_tf: "<<calibration_file_tf<<endl;

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
    // Matrix directly read from yaml config is actually the transpose,
    // So additional transpose should be done and then map it.
    Eigen::Matrix3d inner_transpose; 
    inner_transpose = Eigen::Map<Eigen::MatrixXd>(inner_list.data(), 3, 3);
    inner = inner_transpose.transpose();
    distortion = Eigen::Map<Eigen::Matrix<double,1,5>>(dist_list.data(), 1, 5);

    cv::eigen2cv(inner,camera_instrinsics_);
    cv::eigen2cv(distortion,distortion_coefficients_);

    config.IsNull();
    fout<< endl;
	
}

void save_yaml(void)
{
    fout << config;
    fout.close();
}

Mat rot2euler(const Mat & rotationMatrix)
{
    Mat euler(3, 1, CV_64F);
    double m00 = rotationMatrix.at<double>(0, 0);
    double m02 = rotationMatrix.at<double>(0, 2);
    double m10 = rotationMatrix.at<double>(1, 0);
    double m11 = rotationMatrix.at<double>(1, 1);
    double m12 = rotationMatrix.at<double>(1, 2);
    double m20 = rotationMatrix.at<double>(2, 0);
    double m22 = rotationMatrix.at<double>(2, 2);
    double x, y, z;
    // Assuming the angles are in radians.
    if (m10 > 0.998) { // singularity at north pole
        x = 0;
        y = CV_PI / 2;
        z = atan2(m02, m22);
    }
    else if (m10 < -0.998) { // singularity at south pole
        x = 0;
        y = -CV_PI / 2;
        z = atan2(m02, m22);
    }
    else
    {
        x = atan2(-m12, m11);
        y = asin(m10);
        z = atan2(-m20, m00);
    }
    euler.at<double>(0) = x;
    euler.at<double>(1) = y;
    euler.at<double>(2) = z;
    return euler;
}

void quat2matrix_and_rotatvec(Eigen::Quaterniond &q,Eigen::Matrix3d &mat3X3,cv::Mat &rvec)
{
    Eigen::Matrix3d rot_eigen_mat33 = q.toRotationMatrix();
    mat3X3 = rot_eigen_mat33;

    cv::Mat rot_cv_mat33(3, 3, cv::DataType<double>::type);
    eigen2cv(rot_eigen_mat33,rot_cv_mat33);
    Rodrigues (rot_cv_mat33, rvec);

}

template <class T>
void point_reproject(const T* q, const T* t,
const Eigen::Matrix<T, 3, 1> point_at_world, T reproject_point[2]){    
    Eigen::Quaternion<T> q_last(q[0], q[1], q[2], q[3]);
    q_last.normalize();
    Eigen::Matrix<T, 3, 1> t_last(t[0], t[1], t[2]);

    Eigen::Matrix<T, 3, 1> point_at_camera;
    point_at_camera = q_last * point_at_world + t_last;

    Eigen::Matrix<T, 2, 1> pixel_at_image;

    double intrinsic[4] = {inner(0,0), inner(0,2), inner(1,1), inner(1,2)};
    
    pixel_at_image[0] =
        intrinsic[0] * point_at_camera[0] / point_at_camera[2] +
        intrinsic[1];
    pixel_at_image[1] =
        intrinsic[2] * point_at_camera[1] / point_at_camera[2] +
        intrinsic[3];

    T u = (pixel_at_image[0] - intrinsic[1]) /
          intrinsic[0];
    T v = (pixel_at_image[1] - intrinsic[3]) /
          intrinsic[2];

    T r_2 = u * u + v * v;

    double k1 = distortion[0];
    double k2 = distortion[1];
    double p1 = distortion[2];
    double p2 = distortion[3];
    double k3 = distortion[4];

    v = v * (1. + k1 * r_2 + k2 * r_2 * r_2 + k3*r_2 * r_2*r_2 ) +  p1 * (r_2 + 2. * pow(v,2)) + 2. * p2 * u * v; 
    u = u * (1. + k1 * r_2 + k2 * r_2 * r_2 + k3*r_2 * r_2*r_2) +  2. * p1 * u * v + p2 * (r_2 + 2. * pow(u,2)) ;


    reproject_point[0] =
        u * intrinsic[0] + intrinsic[1];
    reproject_point[1] =
        v * intrinsic[2] + intrinsic[3];
}

template <typename _T>
struct pixel_cost_function {
  // obtained from the point reproject to image
  Eigen::Matrix<_T, 3, 1> point_from_world_;
  // obtained directly from image
  Eigen::Matrix<_T, 2, 1> pixel_from_image_;
 
  // construct function
  pixel_cost_function(const Eigen::Matrix<_T, 3, 1>& point_from_world,const Eigen::Matrix<_T, 2, 1>& pixel_from_image)
      :point_from_world_(point_from_world),pixel_from_image_(pixel_from_image)
        {}

  // operator() function
  template <typename T>
  bool operator()(const T* t, const T* q, T* residual) const {
    Eigen::Matrix<T, 3, 1> point_at_world(static_cast<T>(point_from_world_(0)),
                                          static_cast<T>(point_from_world_(1)),
                                          static_cast<T>(point_from_world_(2)));
    T reproject_point[2];

    point_reproject( q,  t, point_at_world, reproject_point);

    Eigen::Matrix<T, 2, 1> residual_matrix(pixel_from_image_[0] - reproject_point[0],
                                           pixel_from_image_[1] - reproject_point[1]);

    residual[0] = residual_matrix[0];
    residual[1] = residual_matrix[1];

    return true;
  }

  // param[in] weight:
  static ceres::CostFunction* Create(
      const Eigen::Matrix<_T, 3, 1> point_from_world,
      const Eigen::Matrix<_T, 2, 1> point_from_pixel
      ) {
    return (
        new ceres::AutoDiffCostFunction<pixel_cost_function, 2,3,4>(new pixel_cost_function(
            point_from_world, point_from_pixel)));
  }
};


void CalibrateCamera(
    const std::vector<calibrate_cam_lidar_data>& calibrate_data,
    const int& start_index,
    const int& total_num) {

    t[0] = tvec_c_l.at<double>(0);
    t[1] = tvec_c_l.at<double>(1);
    t[2] = tvec_c_l.at<double>(2);

    cv::Mat cv_rotationMatrix(3, 3, CV_64F);
    Eigen::Matrix3d mat3x3;

    Rodrigues (rvec_c_l, cv_rotationMatrix);//rootation_vector -> cv_mat 
    cv2eigen(cv_rotationMatrix,mat3x3);//cv_mat -> eigen_mat
    Eigen::Quaterniond quaternion(mat3x3);//eigen_mat -> quat
    quaternion.normalize();
    q[0] = quaternion.w();
    q[1] = quaternion.x();
    q[2] = quaternion.y();
    q[3] = quaternion.z();

    ceres::LocalParameterization* q_parameterization =
        new ceres::EigenQuaternionParameterization();
   
    ceres::Problem::Options problem_options;
    ceres::Problem problem(problem_options);

    for (int i = start_index; i < total_num + start_index; ++i) {

        problem.AddResidualBlock(
            pixel_cost_function<
                double>::Create(calibrate_data[i].lidar_point.cast<double>(),
                                calibrate_data[i].pixel_point.cast<double>()
                                ),
            new ceres::HuberLoss(0.5), t, q);
    }

    ceres::Solver::Options options;
    options.max_num_iterations = 100;
    options.linear_solver_type = ceres::DENSE_QR;
    options.function_tolerance = 1e-18;
    options.parameter_tolerance = 1e-18;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.FullReport() << "\n";

    Eigen::Quaterniond q_final(q[0], q[1], q[2], q[3]);
    Eigen::Vector3d euler_angle = q_final.toRotationMatrix().eulerAngles(2, 1, 0);

    cout<<"transform:  "<<endl;
    cout<<  t[0]<<" , "<<  t[1]<<" , "<<  t[2]<<endl;
    cout<<"euler_angle:  "<<endl;
    cout<<  euler_angle <<endl;

    vector<double> vec_t  = {t[0],t[1],t[2]};
    cv::Mat Trans(vec_t);

    vector<double> vec_eul  = {euler_angle[0],euler_angle[1],euler_angle[2]};
    cv::Mat euler(vec_eul);

    cv::FileStorage fs_write_tf(calibration_file_tf,cv::FileStorage::WRITE);

    fs_write_tf<< "transform"<<vec_t;
    fs_write_tf<< "euler_angle"<<vec_eul;
    fs_write_tf.release();
  
}

void calcul_r_t(void)
{
    YAML::Node config_r = YAML::LoadFile(feature_file);
    
    for(int pair_cnt=1; pair_cnt<=pairnum; pair_cnt++)
    {    
        cv::Point3f tmp_l;
        std::string pair_num_read="pair"+std::to_string(pair_cnt);

        tmp_l.x=config_r[pair_num_read]["lidar"]["board_left_point"]["x"].as<float>();
        tmp_l.y=config_r[pair_num_read]["lidar"]["board_left_point"]["y"].as<float>();
        tmp_l.z=config_r[pair_num_read]["lidar"]["board_left_point"]["z"].as<float>();
        board_corner_ps.push_back(tmp_l);

        tmp_l.x=config_r[pair_num_read]["lidar"]["board_right_point"]["x"].as<float>();
        tmp_l.y=config_r[pair_num_read]["lidar"]["board_right_point"]["y"].as<float>();
        tmp_l.z=config_r[pair_num_read]["lidar"]["board_right_point"]["z"].as<float>();
        board_corner_ps.push_back(tmp_l);

        tmp_l.x=config_r[pair_num_read]["lidar"]["board_top_point"]["x"].as<float>();
        tmp_l.y=config_r[pair_num_read]["lidar"]["board_top_point"]["y"].as<float>();
        tmp_l.z=config_r[pair_num_read]["lidar"]["board_top_point"]["z"].as<float>();
        board_corner_ps.push_back(tmp_l);

        tmp_l.x=config_r[pair_num_read]["lidar"]["board_down_point"]["x"].as<float>();
        tmp_l.y=config_r[pair_num_read]["lidar"]["board_down_point"]["y"].as<float>();
        tmp_l.z=config_r[pair_num_read]["lidar"]["board_down_point"]["z"].as<float>();
        board_corner_ps.push_back(tmp_l);

        cv::Point2f tmp_c;
        tmp_c.x=config_r[pair_num_read]["camera"]["board_left_point"]["x"].as<float>();
        tmp_c.y=config_r[pair_num_read]["camera"]["board_left_point"]["y"].as<float>();
        img_corners.push_back(tmp_c);

        tmp_c.x=config_r[pair_num_read]["camera"]["board_right_point"]["x"].as<float>();
        tmp_c.y=config_r[pair_num_read]["camera"]["board_right_point"]["y"].as<float>();
        img_corners.push_back(tmp_c);

        tmp_c.x=config_r[pair_num_read]["camera"]["board_top_point"]["x"].as<float>();
        tmp_c.y=config_r[pair_num_read]["camera"]["board_top_point"]["y"].as<float>();
        img_corners.push_back(tmp_c);

        tmp_c.x=config_r[pair_num_read]["camera"]["board_down_point"]["x"].as<float>();
        tmp_c.y=config_r[pair_num_read]["camera"]["board_down_point"]["y"].as<float>();
        img_corners.push_back(tmp_c);
    } 
    cout<< img_corners<<endl;
    cout<< board_corner_ps<<endl;

    // 1st step: rough calculation of extrinsic using pnp
    std::vector<cv::Point3f> board_corner_ps_tough_cali;
    std::vector<cv::Point2f> img_corners_tough_cali;
    for(int i = 0;i<4;i++)
    {
        board_corner_ps_tough_cali.push_back(board_corner_ps[i]);
        img_corners_tough_cali.push_back(img_corners[i]);
    }

    cv::solvePnP(board_corner_ps_tough_cali, img_corners_tough_cali, camera_instrinsics_, distortion_coefficients_, rvec_c_l, tvec_c_l);
    cout<<"rvec_c_l"<<endl;
    cout<<rvec_c_l<<endl;
    cout<<"tvec_c_l"<<endl;
    cout<<tvec_c_l<<endl;
    // 2nd step: send all point-pairs and rough extrinsic to ceres

    for(int i = 0; i<img_corners.size(); i++)
    {
        calibrate_cam_lidar_data calib_tmp;
        cv::Point3f tmp_lidar = board_corner_ps[i];
        cv::Point2f tmp_img   = img_corners[i];
        
        calib_tmp.lidar_point.x() = tmp_lidar.x;
        calib_tmp.lidar_point.y() = tmp_lidar.y;
        calib_tmp.lidar_point.z() = tmp_lidar.z;

        calib_tmp.pixel_point.x() = tmp_img.x;
        calib_tmp.pixel_point.y() = tmp_img.y;
        
        calibrate_data_input.push_back(calib_tmp);    
    }    
    
    int start_index = 0;
    int total_num = calibrate_data_input.size();

    CalibrateCamera(calibrate_data_input,start_index,total_num);
    
}
void re_project_error_pnp(void)
{
    // calculate re-project using pnp
    std::vector<cv::Point2f> pr_board_corner_ps;
    cv::projectPoints(board_corner_ps, rvec_c_l, tvec_c_l, camera_instrinsics_, distortion_coefficients_, pr_board_corner_ps);
    for (int i = 0; i < pr_board_corner_ps.size(); i++)
    {
        cv::circle(image_, pr_board_corner_ps[i], 2+i*1.2, CV_RGB(0, 0, 255), -1);
    }
    double rms = 0;
    if(pr_board_corner_ps.size() == pr_board_corner_ps.size())
    {
        for(int i = 0;i<pr_board_corner_ps.size();i++)
        {
            double error = sqrt(pow((pr_board_corner_ps[i].x-img_corners[i].x),2)+pow((pr_board_corner_ps[i].y-img_corners[i].y),2));
            rms += error;
            cout<<"pnp error: "<< error <<endl;

        }
        cout<<"pnp reproject error: "<< rms/pr_board_corner_ps.size()<<endl;
    }
    else
    {
        cout<< "pnp reproject error..."<<endl;
        return ;
    }
}

void re_project_error_ba(void)
{
    
    // calculate re-project using ba
    int use_ba_param = 1;
    cv::Mat rvec_c_l_tmp(3, 1, cv::DataType<double>::type);
    cv::Mat tvec_c_l_tmp(3, 1, cv::DataType<double>::type);
    rvec_c_l_tmp = rvec_c_l;
    tvec_c_l_tmp = tvec_c_l;

    if(use_ba_param == 1)
    {
        cout<<"re_project_error_ba ... "<<endl;
        Eigen::Quaterniond q_tmp(q[0], q[1], q[2], q[3]);
        q_tmp.normalize();

        Eigen::Matrix3d rot_eigen_mat33 = q_tmp.toRotationMatrix();

        cv::Mat rot_cv_mat33(3, 3, cv::DataType<double>::type);
        eigen2cv(rot_eigen_mat33,rot_cv_mat33);

        Rodrigues (rot_cv_mat33, rvec_c_l_tmp);
        
        tvec_c_l_tmp.at<double>(0) = t[0];
        tvec_c_l_tmp.at<double>(1) = t[1];
        tvec_c_l_tmp.at<double>(2) = t[2];

        cout<<"tvec_c_l_tmp :"<< rvec_c_l_tmp <<endl;
        cout<<"tvec_c_l_tmp :"<< tvec_c_l_tmp <<endl;

        cv::FileStorage fs_write(calibration_file,cv::FileStorage::WRITE);
        fs_write<< "rvec"<<rvec_c_l_tmp;
        fs_write<< "tvec"<<tvec_c_l_tmp;
        fs_write.release();
        
    }

    std::vector<cv::Point2f> pr_board_corner_ps;
    cv::projectPoints(board_corner_ps, rvec_c_l_tmp, tvec_c_l_tmp, camera_instrinsics_, distortion_coefficients_, pr_board_corner_ps);
    for (int i = 0; i < pr_board_corner_ps.size(); i++)
    {
        cv::circle(image_, pr_board_corner_ps[i], 2+i*1.2, CV_RGB(0, 0, 255), -1);
    }
    double rms = 0;
    if(pr_board_corner_ps.size() == pr_board_corner_ps.size())
    {
        for(int i = 0;i<pr_board_corner_ps.size();i++)
        {
            double error = sqrt(pow((pr_board_corner_ps[i].x-img_corners[i].x),2)+pow((pr_board_corner_ps[i].y-img_corners[i].y),2));
            rms += error;
            cout<<"ba error: "<< error <<endl;

        }
        cout<<"ba reproject error: "<< rms/pr_board_corner_ps.size()<<endl;
    }
    else
    {
        cout<< "ba reproject error..."<<endl;
        return ;
    }
    const int width = 1920;
    const int height = 1080;
    cv::Mat blank_img(height,width,CV_8UC3,cv::Scalar(48,48,48));

    for(const auto &pt:pr_board_corner_ps)
    {
        cv::circle(blank_img,pt,3,cv::Scalar(0,0,255),-1);
    }

    for(const auto &pt:img_corners)
    {
        cv::circle(blank_img,pt,3,cv::Scalar(0,255,0),-1);
    }

    for(int i = 0;i<img_corners.size();i++)
    {
        cv::line(blank_img,img_corners.at(i),pr_board_corner_ps.at(i),cv::Scalar(255,255,255));
    }

    for(int i = 0;i<pr_board_corner_ps.size()/4;i++)
    {
        for(int v = 0;v<3;v++)
        {
            cv::line(blank_img,pr_board_corner_ps.at(i*4+v),pr_board_corner_ps.at(i*4+v+1),cv::Scalar(0,128,0));
        }
        cv::line(blank_img,pr_board_corner_ps.at(i*4),pr_board_corner_ps.at(i*4+3),cv::Scalar(128,0,0));
    }

    cv::imshow("check:projected points in cyan, original 3d points in red.",blank_img);

    cv::waitKey(0);

}

int GetReprojectError(
    const Eigen::Vector3d& t, const Eigen::Quaterniond& q,
    const std::vector<calibrate_cam_lidar_data>& data) {
  double error_l2 = 0;
  std::size_t data_size = data.size();

  for (std::size_t i = 0; i < data_size; i++) {
    const auto& lidar_point = data[i].lidar_point;
    const auto& image_point = data[i].pixel_point;
    
    Eigen::Vector3d point_at_camera = q * lidar_point + t;

    double fx = inner(0,0);
    double fy = inner(1,1);
    double cx = inner(0,2);
    double cy = inner(1,2);

    double u = double(fx * point_at_camera[0] / point_at_camera[2] +
                      cx);
    double v = double(fy * point_at_camera[1] / point_at_camera[2] +
                      cy);

    u = (u - cx) /fx;
    v = (v - cy) /fy;

    double r_2 = u * u + v * v;

    double k1 = distortion[0];
    double k2 = distortion[1];
    double p1 = distortion[2];
    double p2 = distortion[3];
    double k3 = distortion[4];

    v = v * (1. + k1 * r_2 + k2 * r_2 * r_2 + k3*r_2 * r_2*r_2 ) + p1 * (r_2 + 2. * pow(v,2)) + 2. * p2 * u * v; 
    u = u * (1. + k1 * r_2 + k2 * r_2 * r_2 + k3*r_2 * r_2*r_2) +  2. * p1 * u * v + p2 * (r_2 + 2. * pow(u,2));

    u = u * fx + cx;
    v = v * fy + cy;

    double pixel_error = sqrt(pow((u - image_point.x()),2)  + pow((v - image_point.y()),2));
    cout<< "pixel reproject error :"<< pixel_error <<endl;

    error_l2 += pixel_error;
  }
  cout<< "all_reproject error :"<< error_l2/data_size <<endl;
  return error_l2;
}

