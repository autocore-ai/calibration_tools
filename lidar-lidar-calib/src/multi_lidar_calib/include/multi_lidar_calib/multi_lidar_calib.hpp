#ifndef MULTI_LIDAR_CALIB__MULTI_LIDAR_CALIB_HPP_
#define MULTI_LIDAR_CALIB__MULTI_LIDAR_CALIB_HPP_

#include <functional>
#include <memory>
#include <iomanip>
#include <fstream>
#include <iostream>
#include <boost/filesystem.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "autoware_auto_vehicle_msgs/msg/velocity_report.hpp"
#include <rviz2_capture_plugin_interface/srv/capture.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/approximate_voxel_grid.h>


class MultiLidarCalibSub : public rclcpp::Node
{
public:
  MultiLidarCalibSub();

private:
  bool cloud_captureed_flag_;
  bool lidar_extrinsic_calculated_;

  int vehicle_stoped_count_;
  int pointcloud_stored_count_;
  int total_count_;

  std::string save_path_;
  std::string file_res_;
  std::string file_pcd_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_main_lidar_current_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_sub_lidar_current_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_array_main_lidar_[5];
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_array_sub_lidar_[1];
  Eigen::Matrix<float, 4, 4> transformation_array_[4];

  int num_wait_period_;
  Eigen::Matrix4f relative_pose_;
  // std::array<double, 16> trans_matrix_;

  template <typename Registration>
  void register_frame(
      Registration& reg, 
      const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& target, 
      const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& source,
      Eigen::Matrix<float, 4, 4>& transformation_return);

  pcl::PointCloud<pcl::PointXYZI> draw_pointcloud(
      pcl::PointCloud<pcl::PointXYZI> pointcloud_original, double colour_value);

  void capture_point_cloud();
  void calculate_lidar_extrinsic();

  void vehicle_velocity_callback(const autoware_auto_vehicle_msgs::msg::VelocityReport::SharedPtr msg);
  void lidar_main_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void lidar_sub_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void onCaptureService(
  const rviz2_capture_plugin_interface::srv::Capture::Request::SharedPtr request,
  const rviz2_capture_plugin_interface::srv::Capture::Response::SharedPtr response);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_main_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_subscription_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr vehicle_velocity_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr final_cloud_publisher_;

  rclcpp::Service<rviz2_capture_plugin_interface::srv::Capture>::SharedPtr srv_capture;
};

#endif  // MULTI_LIDAR_CALIB__MULTI_LIDAR_CALIB_HPP_
