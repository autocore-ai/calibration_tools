#include <chrono>
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/registration/ndt.h>
#include <pcl/registration/gicp.h>
#include <fast_gicp/gicp/fast_gicp.hpp>
#include <fast_gicp/gicp/fast_gicp_st.hpp>
#include <fast_gicp/gicp/fast_vgicp.hpp>

#ifdef USE_VGICP_CUDA
#include <fast_gicp/ndt/ndt_cuda.hpp>
#include <fast_gicp/gicp/fast_vgicp_cuda.hpp>
#endif

// benchmark for fast_gicp registration methods
template <typename Registration>
void register_frame(Registration& reg, 
                      const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& target, 
                      const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& source,
                      Eigen::Matrix<float, 4, 4>& transformation_return) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZI>);

  double fitness_score = 0.0;

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
  double single = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count() / 1e6;

  std::cout << "single:" << single << "[msec] " << "fitness_score:" << fitness_score << std::endl;
  std::cout << "result matrix:" << std::endl;
  transformation_return = reg.getFinalTransformation();

  std::cout << transformation_return << std::endl;


  // pcl::PointCloud<pcl::PointXYZI>::Ptr trans_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  // pcl::transformPointCloud(*source, *trans_cloud_ptr, reg.getFinalTransformation());

  // std::stringstream ss;
  // ss << "./result.pcd";
  // pcl::io::savePCDFileASCII(ss.str(), *trans_cloud_ptr);
  // std::cerr << "Data saved to " << ss.str () << std::endl;
}

pcl::PointCloud<pcl::PointXYZI> draw_pointcloud(pcl::PointCloud<pcl::PointXYZI> pointcloud_original, double colour_value)
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


/**
 * @brief main
 */
int main(int argc, char** argv) {
  // if (argc < 3) {
  //   std::cout << "usage: gicp_align target_pcd source_pcd" << std::endl;
  //   return 0;
  // }

  std::cout << "[Multi lidar calibration]: collecting main-lidar data start." << std::endl;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_array[5];
  // pcl::PointCloud<pcl::PointXYZI>::Ptr result_array[4];

  Eigen::Matrix<float, 4, 4> transformation_array[4];

  for (size_t i = 0; i < 5; i++)
  {
    cloud_array[i] = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    // result_array[i] = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  } 

  for (size_t i = 1; i <= 5; i++)
  {
    std::string ss;
    ss = std::to_string(i) + ".pcd";
    if (pcl::io::loadPCDFile(ss, *cloud_array[i-1])) 
    {
      std::cerr << "failed to open " << ss << std::endl;
      return 1;
    }
    cloud_array[i-1]->erase(
      std::remove_if(cloud_array[i-1]->begin(), cloud_array[i-1]->end(), [=](const pcl::PointXYZI& pt) { return pt.getVector3fMap().squaredNorm() < 1e-3; }),
      cloud_array[i-1]->end());


    // downsampling
    pcl::ApproximateVoxelGrid<pcl::PointXYZI> voxelgrid;
    voxelgrid.setLeafSize(0.1f, 0.1f, 0.1f);

    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>());
    voxelgrid.setInputCloud(cloud_array[i-1]);
    voxelgrid.filter(*filtered);
    cloud_array[i-1] = filtered;
  }
  
  std::cout << "[Multi lidar calibration]: collecting main-lidar data finish." << std::endl;

  std::cout << "[Multi lidar calibration]: mapping process start." << std::endl;

  pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZI>());

  pcl::PointCloud<pcl::PointXYZI>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZI>());

  *final_cloud += draw_pointcloud(*cloud_array[0],40);

  // if (pcl::io::loadPCDFile(argv[1], *target_cloud)) {
  //   std::cerr << "failed to open " << argv[1] << std::endl;
  //   return 1;
  // }
  // if (pcl::io::loadPCDFile(argv[2], *source_cloud)) {
  //   std::cerr << "failed to open " << argv[2] << std::endl;
  //   return 1;
  // }


  for (size_t i = 0; i <= 3; i++)
  {  
    // remove invalid points around origin
    source_cloud = cloud_array[i+1];
    target_cloud = cloud_array[i];

    // source_cloud->erase(
    //   std::remove_if(source_cloud->begin(), source_cloud->end(), [=](const pcl::PointXYZI& pt) { return pt.getVector3fMap().squaredNorm() < 1e-3; }),
    //   source_cloud->end());
    // target_cloud->erase(
    //   std::remove_if(target_cloud->begin(), target_cloud->end(), [=](const pcl::PointXYZI& pt) { return pt.getVector3fMap().squaredNorm() < 1e-3; }),
    //   target_cloud->end());

    // // downsampling
    // pcl::ApproximateVoxelGrid<pcl::PointXYZI> voxelgrid;
    // voxelgrid.setLeafSize(0.1f, 0.1f, 0.1f);

    // pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>());
    // voxelgrid.setInputCloud(target_cloud);
    // voxelgrid.filter(*filtered);
    // target_cloud = filtered;

    // filtered.reset(new pcl::PointCloud<pcl::PointXYZI>());
    // voxelgrid.setInputCloud(source_cloud);
    // voxelgrid.filter(*filtered);
    // source_cloud = filtered;

    std::cout << "target:" << target_cloud->size() << "[pts] source:" << source_cloud->size() << "[pts]" << std::endl;



    std::cout << "--- fgicp_st ---" << std::endl;
    fast_gicp::FastGICPSingleThread<pcl::PointXYZI, pcl::PointXYZI> fgicp_st;

    // Eigen::Matrix<Scalar, 4, 4> transformation_current;

    register_frame(fgicp_st, target_cloud, source_cloud, transformation_array[i]);


    pcl::PointCloud<pcl::PointXYZI> trans_cloud_temp_1,trans_cloud_temp_2;
    pcl::transformPointCloud(*source_cloud, trans_cloud_temp_1, transformation_array[0]);
    for (size_t j = 1; j < i+1; j++)
    {
      pcl::transformPointCloud(trans_cloud_temp_1, trans_cloud_temp_2, transformation_array[j]);
      trans_cloud_temp_1 = trans_cloud_temp_2;
    }
    
    std::stringstream ss;
    ss << "result"<< (i+1)*10<< ".pcd";
    pcl::io::savePCDFileASCII(ss.str(), draw_pointcloud(trans_cloud_temp_1,40*(i+2)));
    std::cerr << "Data saved to " << ss.str () << std::endl;

    *final_cloud += draw_pointcloud(trans_cloud_temp_1,40*(i+2));
  }

  // pcl::PointCloud<pcl::PointXYZI>::Ptr trans_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  // pcl::transformPointCloud(*source, *trans_cloud_ptr, reg.getFinalTransformation());

  std::stringstream ss;
  ss << "./result.pcd";
  pcl::io::savePCDFileASCII(ss.str(), *final_cloud);
  std::cerr << "Data saved to " << ss.str () << std::endl;

  std::cout << "[Multi lidar calibration]: mapping process finish." << std::endl;

  std::cout << "[Multi lidar calibration]: reading initial extrinsic parameter start." << std::endl;

  Eigen::Matrix4f relative_pose;
  relative_pose.setIdentity();
  std::ifstream ifs("relative.txt");
  if (!ifs) {
    return false;
  }

  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      ifs >> relative_pose(i, j);
    }
  }

  std::cout << "[Multi lidar calibration]: reading initial extrinsic parameter finish." << std::endl;

  std::cout << "initial extrinsic matrix:" << std::endl;
  std::cout << relative_pose << std::endl;

  std::cout << "[Multi lidar calibration]: collecting sub-lidar data start." << std::endl;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_array_sub[5];
  // pcl::PointCloud<pcl::PointXYZI>::Ptr result_array[4];

  Eigen::Matrix<float, 4, 4> transformation_array_sub[5];

  for (size_t i = 0; i < 5; i++)
  {
    cloud_array_sub[i] = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    // result_array[i] = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  } 

  for (size_t i = 6; i <= 10; i++)
  {
    std::string ss;
    ss = std::to_string(i) + ".pcd";
    if (pcl::io::loadPCDFile(ss, *cloud_array_sub[i-6])) 
    {
      std::cerr << "failed to open " << ss << std::endl;
      return 1;
    }
    cloud_array_sub[i-6]->erase(
      std::remove_if(cloud_array_sub[i-6]->begin(), cloud_array_sub[i-6]->end(), [=](const pcl::PointXYZI& pt) { return pt.getVector3fMap().squaredNorm() < 1e-3; }),
      cloud_array_sub[i-6]->end());


    // downsampling
    pcl::ApproximateVoxelGrid<pcl::PointXYZI> voxelgrid;
    voxelgrid.setLeafSize(0.1f, 0.1f, 0.1f);

    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>());
    voxelgrid.setInputCloud(cloud_array_sub[i-6]);
    voxelgrid.filter(*filtered);
    std::vector<int> mapping;
    pcl::removeNaNFromPointCloud(*filtered, *filtered, mapping);
    cloud_array_sub[i-6] = filtered;
  }
  
  std::cout << "[Multi lidar calibration]: collecting sub-lidar data finish." << std::endl;

  std::cout << "[Multi lidar calibration]: clibration process start." << std::endl;

  pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_sub_calib_initial(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::transformPointCloud(*cloud_array_sub[0], *pointcloud_sub_calib_initial, relative_pose);

  source_cloud = pointcloud_sub_calib_initial;
  target_cloud = final_cloud;
  std::cout << "target:" << target_cloud->size() << "[pts] source:" << source_cloud->size() << "[pts]" << std::endl;

  std::cout << "--- fgicp_st ---" << std::endl;
  fast_gicp::FastGICPSingleThread<pcl::PointXYZI, pcl::PointXYZI> fgicp_st_sub;
  register_frame(fgicp_st_sub, target_cloud, source_cloud, transformation_array_sub[0]);
  
  std::cout << "[Multi lidar calibration]: clibration process finish." << std::endl;

  pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_sub_calib_final(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::transformPointCloud(*pointcloud_sub_calib_initial, *pointcloud_sub_calib_final, transformation_array_sub[0]);
    
  *final_cloud += draw_pointcloud(*pointcloud_sub_calib_final,240);

  std::stringstream ss2;
  ss2 << "./result_with_sub_lidar.pcd";
  pcl::io::savePCDFileASCII(ss2.str(), *final_cloud);
  std::cerr << "Data saved to " << ss2.str () << std::endl;

  pcl::PointCloud<pcl::PointXYZI>::Ptr final_cloud_with_main_frame_0(new pcl::PointCloud<pcl::PointXYZI>());
  *final_cloud_with_main_frame_0 += draw_pointcloud(*cloud_array[0],40);
  *final_cloud_with_main_frame_0 += draw_pointcloud(*pointcloud_sub_calib_final,240);
  std::stringstream ss3;
  ss3 << "./main_lidar_frame_0_with_sub_lidar.pcd";
  pcl::io::savePCDFileASCII(ss3.str(), *final_cloud_with_main_frame_0);
  std::cerr << "Data saved to " << ss3.str () << std::endl;

  Eigen::Matrix<float, 4, 4> transformation_final = transformation_array_sub[0] * relative_pose;

  std::cout << "---final extrinsic matrix:---" << std::endl;
  std::cout << transformation_final << std::endl;

  Eigen::Matrix3d Rotation_matrix = transformation_final.block<3,3>(0,0).cast<double>();

  std::cout << "---final rotation matrix:---" << std::endl;
  std::cout << Rotation_matrix << std::endl;

  // 旋转向量转换为四元数
  Eigen::Quaterniond q(Rotation_matrix);
  //    Eigen::Quaterniond q;
  //    q = Rotation_vector;

  std::cout << "---final quaterniond:---" << std::endl;
  std::cout << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w() << std::endl;

  // 旋转向量转为欧拉角，顺序为z-y-x,也即yaw pitch roll
  Eigen::Vector3d eulerAngle = Rotation_matrix.eulerAngles(2,1,0);

  std::cout << "---final yaw pitch roll:---" << std::endl;
  std::cout << eulerAngle << std::endl;

  /*just for test*/
  // Eigen::Matrix<float, 4, 4> transformation_main_2_sub;
  // Eigen::Matrix<float, 4, 4> transformation_temp;
  // pcl::PointCloud<pcl::PointXYZI>::Ptr final_cloud_temp(new pcl::PointCloud<pcl::PointXYZI>());

  // Eigen::Matrix4f relative_pos;
  // relative_pos.setIdentity();
  // relative_pos(0, 3) = -2.5;
  // relative_pos(1, 3) = 0;
  // relative_pos(2, 3) = 1.45;

  // pcl::transformPointCloud(*final_cloud, *final_cloud_temp, relative_pos);



  // std::cout << "[just for test]" << std::endl;
  // source_cloud = final_cloud_temp;
  // target_cloud = cloud_array_sub[0];
  // std::cout << "target:" << target_cloud->size() << "[pts] source:" << source_cloud->size() << "[pts]" << std::endl;
  // register_frame(fgicp_st_sub, target_cloud, source_cloud, transformation_temp);
  // transformation_main_2_sub = transformation_temp * relative_pos;

  std::cout << "---final extrinsic matrix (from main to sub):---" << std::endl;
  Eigen::Matrix<float, 4, 4> transformation_main_2_sub = transformation_final.inverse();
  std::cout << transformation_main_2_sub  << std::endl;

  Eigen::Matrix3d Rotation_matrix_main_2_sub = transformation_main_2_sub.block<3,3>(0,0).cast<double>();

  std::cout << "---final rotation matrix (from main to sub):---" << std::endl;
  std::cout << Rotation_matrix_main_2_sub << std::endl;

  // 旋转向量转换为四元数
  Eigen::Quaterniond q_main_2_sub(Rotation_matrix_main_2_sub);
  //    Eigen::Quaterniond q;
  //    q = Rotation_vector;

  std::cout << "---final quaterniond (from main to sub):---" << std::endl;
  std::cout << q_main_2_sub.x() << ", " << q_main_2_sub.y() << ", " << q_main_2_sub.z() << ", " << q_main_2_sub.w() << std::endl;

  // 旋转向量转为欧拉角，顺序为z-y-x,也即yaw pitch roll
  Eigen::Vector3d eulerAngle_main_2_sub = Rotation_matrix_main_2_sub.eulerAngles(2,1,0);

  std::cout << "---final yaw pitch roll (from main to sub):---" << std::endl;
  std::cout << eulerAngle_main_2_sub << std::endl;

  return 0;
}
