// Copyright 2022 AutoCore
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// you may obtain a copy of the License at
// 
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <functional>
#include "multi_lidar_calib/multi_lidar_calib.hpp"

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <fast_gicp/gicp/fast_gicp.hpp>
#include <fast_gicp/gicp/fast_gicp_st.hpp>
#include <fast_gicp/gicp/fast_vgicp.hpp>

#include <iostream>

MultiLidarCalibSub::MultiLidarCalibSub()
: Node("MultiLidarCalibSub"),
  cloud_captureed_flag_(false),
  lidar_extrinsic_calculated_(false),
  vehicle_stoped_count_(0),
  pointcloud_stored_count_(0)
{
  std::string main_lidar_topic = this->declare_parameter("main_lidar_topic", "/sensing/lidar/top/rectified/pointcloud");
  std::string sub_lidar_topic = this->declare_parameter("sub_lidar_topic", "/rslidar_points");
  std::string velocity_status_topic = this->declare_parameter("velocity_status_topic", "/vehicle/status/velocity_status");

  lidar_main_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    main_lidar_topic, 10, std::bind(&MultiLidarCalibSub::lidar_main_callback, this, std::placeholders::_1));
  lidar_sub_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    sub_lidar_topic, 10, std::bind(&MultiLidarCalibSub::lidar_sub_callback, this, std::placeholders::_1));
  vehicle_velocity_subscription_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>(
    velocity_status_topic, 50, std::bind(&MultiLidarCalibSub::vehicle_velocity_callback, this, std::placeholders::_1));

  srv_capture = this->create_service<rviz2_capture_plugin_interface::srv::Capture>(
  "/multi_lidar_calib/capture", 
  std::bind(&MultiLidarCalibSub::onCaptureService, this, std::placeholders::_1, std::placeholders::_2));

  final_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/final_map", 10);

  num_wait_period_ = this->declare_parameter("num_wait_period", 50);
  total_count_ = this->declare_parameter("total_count", 5);
  save_path_ = this->declare_parameter("save_path", "./");
  file_res_ = this->declare_parameter("file_res", "calib_res.csv");
  file_pcd_ = this->declare_parameter("file_pcd", "result_with_sub_lidar.pcd");

  if (save_path_.empty() || access(save_path_.c_str(), F_OK)==-1)
  {
    //if the directory to save data is not accessible, data will be stored under current folder.
    save_path_ = "./";
    RCLCPP_INFO(this->get_logger(), "\033[1;31mThe user defined path to save data/pcd is not accessible, data will be stored under current path %s by default.\033[0m", save_path_.c_str());       
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "\033[1;32mThe user defined path to save data/pcd is accessible, data will be stored under %s\033[0m", save_path_.c_str());
  }

    // ptr vector init
  for (int i = 0; i < total_count_; i++){
    cloud_array_main_lidar_[i] = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  }
  cloud_array_sub_lidar_[0] = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

  cloud_main_lidar_current_ = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  cloud_sub_lidar_current_ = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

  // for input form: rotation matrix + transformation
  double ida[] = {1.0, 0.0,  0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  std::vector<double> id(ida, std::end(ida));
  std::vector<double> extRotV;
  declare_parameter("rotation", id);
  get_parameter("rotation", extRotV);
  double zea[] = {0.0, 0.0, 0.0};
  std::vector<double> ze(zea, std::end(zea));
  std::vector<double> extTransV;
  declare_parameter("translation", ze);
  get_parameter("translation", extTransV);
  Eigen::Matrix3d extRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV.data(), 3, 3);
  Eigen::Vector3d extTrans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extTransV.data(), 3, 1);
  relative_pose_ = Eigen::Matrix4f::Identity();
  relative_pose_.block<3,3>(0,0) = extRot.cast<float>();
  relative_pose_.block<3,1>(0,3) = extTrans.cast<float>();

}

template <typename Registration>
void MultiLidarCalibSub::register_frame(
  Registration& reg, 
  const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& target, 
  const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& source, 
  Eigen::Matrix<float, 4, 4>& transformation_return)
{
  double fitness_score = 0.0; 
  pcl::PointCloud<pcl::PointXYZI>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZI>);

  // single run
  auto t1 = std::chrono::high_resolution_clock::now();
  // fast_gicp reuses calculated covariances if an input cloud is the same as the previous one
  // to prevent this for benchmarking, force clear source and target clouds
  reg.clearTarget();
  reg.clearSource();
  reg.setInputTarget(target);
  reg.setInputSource(source);
  reg.align(*aligned);
  auto t2 = std::chrono::high_resolution_clock::now();
  fitness_score = reg.getFitnessScore();
  double single = std::chrono::duration_cast<std::chrono::nanoseconds>(t2-t1).count() / 1e6;

  std::cout << "single:" << single << "[msec] " << "fitness_score:" << fitness_score << std::endl;
  std::cout << "result matrix:" << std::endl;
  transformation_return = reg.getFinalTransformation();

  std::cout << transformation_return << std::endl;
}


pcl::PointCloud<pcl::PointXYZI> MultiLidarCalibSub::draw_pointcloud(pcl::PointCloud<pcl::PointXYZI> pointcloud_original, double colour_value)
{

  pcl::PointCloud<pcl::PointXYZI> pointcloud_drawed;
  pointcloud_drawed = pointcloud_original;

  size_t pointcloud_size = pointcloud_original.points.size();
  for (size_t i = 0; i < pointcloud_size; i++)
  {
    pointcloud_drawed.points[i].intensity = colour_value;
  }

  return pointcloud_drawed;
}


void MultiLidarCalibSub::capture_point_cloud()
{
  if(cloud_main_lidar_current_->points.size() > 0 && cloud_sub_lidar_current_->points.size() > 0 )
  {
    /* store current main lidar pointcloud and sub lidar pointcloud into pointcloud sequence */
    cloud_array_main_lidar_[pointcloud_stored_count_] = cloud_main_lidar_current_;
    if (pointcloud_stored_count_ == 0){
      cloud_array_sub_lidar_[0] = cloud_sub_lidar_current_;
    }
    cloud_captureed_flag_ = true;
    pointcloud_stored_count_ += 1;
    vehicle_stoped_count_ = 0;
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "\033[1;32mAt least one of lidar inputs is unavaliable, stop capture.\033[0m");
  }
}


void MultiLidarCalibSub::vehicle_velocity_callback(const autoware_auto_vehicle_msgs::msg::VelocityReport::SharedPtr msg) 
{
  // RCLCPP_INFO(this->get_logger(), "current vehicle velocity: '%f'", msg->longitudinal_velocity);
  if((msg->longitudinal_velocity == 0) && cloud_captureed_flag_ == false && lidar_extrinsic_calculated_ == false)
  {
    vehicle_stoped_count_ += 1;
    if(vehicle_stoped_count_ == 50)
    {
      capture_point_cloud();
      RCLCPP_INFO(this->get_logger(), "lidar pointcloud No.'%d' captured according to current velocity.", pointcloud_stored_count_);
      if(pointcloud_stored_count_ == total_count_)
      {
        calculate_lidar_extrinsic();
        lidar_extrinsic_calculated_ = true;
      }
    }
  }

  if(msg->longitudinal_velocity > 0)
  {
    cloud_captureed_flag_ = false;
  }
}


void MultiLidarCalibSub::lidar_main_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) 
{
  pcl::fromROSMsg(*msg, *cloud_main_lidar_current_);
}


void MultiLidarCalibSub::lidar_sub_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) 
{
  pcl::fromROSMsg(*msg, *cloud_sub_lidar_current_);
}

void MultiLidarCalibSub::onCaptureService(
  const rviz2_capture_plugin_interface::srv::Capture::Request::SharedPtr request,
  const rviz2_capture_plugin_interface::srv::Capture::Response::SharedPtr response)
{
  if (request)
  {
    response->status = true;
  }
  RCLCPP_INFO(this->get_logger(), "sending back response: %d", response->status);
  if (!lidar_extrinsic_calculated_)
  {
    capture_point_cloud();
    RCLCPP_INFO(this->get_logger(), "lidar pointcloud No.'%d' captured manually.", pointcloud_stored_count_);
    if(pointcloud_stored_count_ == total_count_)
      {
        calculate_lidar_extrinsic();
        lidar_extrinsic_calculated_ = true;
      }
  }
}

void MultiLidarCalibSub::calculate_lidar_extrinsic()
{
  /* calculate lidar extrinsic */
  pcl::PointCloud<pcl::PointXYZI>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  *final_cloud += draw_pointcloud(*cloud_array_main_lidar_[0],40);

  pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  Eigen::Matrix<float, 4, 4> result_matrix;

  // construct a local map using main lidar's pointcloud
  for (int i = 0; i <= total_count_ - 2; i++)
  {  
    // remove invalid points around origin
    source_cloud = cloud_array_main_lidar_[i+1];
    target_cloud = cloud_array_main_lidar_[i];

    source_cloud->erase(
      std::remove_if(source_cloud->begin(), source_cloud->end(), [=](const pcl::PointXYZI& pt) { return pt.getVector3fMap().squaredNorm() < 1e-3; }),
      source_cloud->end());
    target_cloud->erase(
      std::remove_if(target_cloud->begin(), target_cloud->end(), [=](const pcl::PointXYZI& pt) { return pt.getVector3fMap().squaredNorm() < 1e-3; }),
      target_cloud->end());

    // downsampling
    pcl::ApproximateVoxelGrid<pcl::PointXYZI> voxelgrid;
    voxelgrid.setLeafSize(0.1f, 0.1f, 0.1f);

    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>());

    voxelgrid.setInputCloud(target_cloud);
    voxelgrid.filter(*filtered);
    target_cloud = filtered;

    filtered.reset(new pcl::PointCloud<pcl::PointXYZI>());
    voxelgrid.setInputCloud(source_cloud);
    voxelgrid.filter(*filtered);
    source_cloud = filtered;

    fast_gicp::FastGICPSingleThread<pcl::PointXYZI, pcl::PointXYZI> fgicp_st;
    register_frame(fgicp_st, target_cloud, source_cloud, transformation_array_[i]);

    pcl::PointCloud<pcl::PointXYZI> trans_cloud_temp_1,trans_cloud_temp_2;
    pcl::transformPointCloud(*source_cloud, trans_cloud_temp_1, transformation_array_[0]);
    for (int j = 1; j < i+1; j++)
    {
      pcl::transformPointCloud(trans_cloud_temp_1, trans_cloud_temp_2, transformation_array_[j]);
      trans_cloud_temp_1 = trans_cloud_temp_2;
    }
    
    // std::stringstream ss;
    // ss << "result"<< (i+1)*10<< ".pcd";
    // pcl::io::savePCDFileASCII(ss.str(), draw_pointcloud(trans_cloud_temp_1, 40*(i+2)));
    // std::cerr << "Data saved to " << ss.str () << std::endl;

    *final_cloud += draw_pointcloud(trans_cloud_temp_1, 40*(i+2));
  }
  std::cout << "--- map construction finished ---" << std::endl;

  std::cout << "initial extrinsic matrix:" << std::endl;
  std::cout << relative_pose_ << std::endl;

  Eigen::Matrix<float, 4, 4> transformation_array_sub[1];

  // downsampling sub lidar pointcloud
  cloud_array_sub_lidar_[0]->erase(
    std::remove_if(cloud_array_sub_lidar_[0]->begin(), cloud_array_sub_lidar_[0]->end(), [=](const pcl::PointXYZI& pt) { return pt.getVector3fMap().squaredNorm() < 1e-3; }),
    cloud_array_sub_lidar_[0]->end());

  pcl::ApproximateVoxelGrid<pcl::PointXYZI> voxelgrid;
  voxelgrid.setLeafSize(0.1f, 0.1f, 0.1f);

  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>());
  voxelgrid.setInputCloud(cloud_array_sub_lidar_[0]);
  voxelgrid.filter(*filtered);
  std::vector<int> mapping;
  pcl::removeNaNFromPointCloud(*filtered, *filtered, mapping);
  cloud_array_sub_lidar_[0] = filtered;

  pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_sub_calib_initial(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::transformPointCloud(*cloud_array_sub_lidar_[0], *pointcloud_sub_calib_initial, relative_pose_);

  source_cloud = pointcloud_sub_calib_initial;
  target_cloud = final_cloud;
  fast_gicp::FastGICPSingleThread<pcl::PointXYZI, pcl::PointXYZI> fgicp_st_sub;
  register_frame(fgicp_st_sub, target_cloud, source_cloud, transformation_array_sub[0]);
  
  std::cout << "--- clibration process finish ---" << std::endl;

  pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_sub_calib_final(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::transformPointCloud(*pointcloud_sub_calib_initial, *pointcloud_sub_calib_final, transformation_array_sub[0]);

  *final_cloud += draw_pointcloud(*pointcloud_sub_calib_final, 240);

  std::stringstream ss2;
  ss2 << save_path_ + file_pcd_;
  pcl::io::savePCDFileASCII(ss2.str(), *final_cloud);
  std::cerr << "PCL saved to " << ss2.str () << std::endl;

  Eigen::Matrix<float, 4, 4> transformation_final = transformation_array_sub[0] * relative_pose_;
  std::cout << "--- final extrinsic matrix: ---" << std::endl;
  std::cout << transformation_final << std::endl;

  Eigen::Matrix3d Rotation_matrix = transformation_final.block<3,3>(0,0).cast<double>();
  std::cout << "--- final rotation matrix: ---" << std::endl;
  std::cout << Rotation_matrix << std::endl;

  Eigen::Quaterniond q(Rotation_matrix);
  std::cout << "--- final quaterniond: ---" << std::endl;
  std::cout << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w() << std::endl;

  Eigen::Vector3d eulerAngle = Rotation_matrix.eulerAngles(2,1,0);
  std::cout << "--- final yaw pitch roll: ---" << std::endl;
  std::cout << eulerAngle << std::endl;


  std::cout << "--- final extrinsic matrix (from main to sub): ---" << std::endl;
  Eigen::Matrix<float, 4, 4> transformation_main_2_sub = transformation_final.inverse();
  std::cout << transformation_main_2_sub  << std::endl;

  Eigen::Matrix3d Rotation_matrix_main_2_sub = transformation_main_2_sub.block<3,3>(0,0).cast<double>();
  std::cout << "--- final rotation matrix (from main to sub): ---" << std::endl;
  std::cout << Rotation_matrix_main_2_sub << std::endl;

  Eigen::Quaterniond q_main_2_sub(Rotation_matrix_main_2_sub);
  std::cout << "--- final quaterniond (from main to sub): ---" << std::endl;
  std::cout << q_main_2_sub.x() << ", " << q_main_2_sub.y() << ", " << q_main_2_sub.z() << ", " << q_main_2_sub.w() << std::endl;

  Eigen::Vector3d eulerAngle_main_2_sub = Rotation_matrix_main_2_sub.eulerAngles(2,1,0);
  std::cout << "--- final yaw pitch roll (from main to sub): ---" << std::endl;
  std::cout << eulerAngle_main_2_sub << std::endl;

  std::stringstream ss3; 
  ss3 << save_path_ + file_res_;
  std::fstream file;
  file.open(ss3.str(),std::ios::out | std::ios::ate | std::ios::in | std::ios::app);
  if(file.is_open())
  {
      file<<"final_quaterniond (from sub to main)"<<"\n";
      file<<std::setfill('0')<<std::setiosflags(std::ios::fixed)<<std::setprecision(6)<<
      q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w() <<"\n";

      // eigen::Rotation_matrix.eulerAngles(2,1,0) --> yaw pitch roll;
      file<<"final_eulerangle_rpy [rad] (from sub to main)"<<"\n";
      file<<std::setfill('0')<<std::setiosflags(std::ios::fixed)<<std::setprecision(6)<<
      eulerAngle(2) << ", " << eulerAngle(1) << ", " << eulerAngle(0) <<"\n";

      file<<"final_translation [m] (from sub to main)"<<"\n";
      file<<std::setfill('0')<<std::setiosflags(std::ios::fixed)<<std::setprecision(6)<<
      transformation_final.row(0).col(3) << ", " 
      << transformation_final.row(1).col(3)  << ", " 
      << transformation_final.row(2).col(3)  << ", " <<"\n";
  }
  file.close();

  sensor_msgs::msg::PointCloud2 ros_msg;
  pcl::toROSMsg(*final_cloud, ros_msg);
  ros_msg.header.frame_id = "map";
  ros_msg.header.stamp = this->now();
  final_cloud_publisher_->publish(ros_msg);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MultiLidarCalibSub>());
  rclcpp::shutdown();
  return 0;
}
