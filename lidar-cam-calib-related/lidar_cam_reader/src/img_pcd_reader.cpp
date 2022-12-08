#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"

#include "rcpputils/filesystem_helper.hpp"
#include "rcutils/time.h"

#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"

#include "std_msgs/msg/string.hpp"

#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "pcl_conversions.h"

#include <iostream>
#include <filesystem>	


namespace fs = std::filesystem;

std::string spilt_last(const std::string& path, std::string delimiter) {
    int index(0);
    int last_index(0);
    std::string str = path;
    while (str.find(delimiter) != std::string::npos) {
        index = str.find(delimiter);
        if(index==str.size()-1) break;
        str = str.substr(index + 1);
        last_index+=index+1;
    }
    return path.substr(last_index);
}

void get_all_bag_folder(std::string path, std::vector<std::string> &folder){
    fs::directory_iterator di(path);
    for (auto &item : di) {
       auto current_path = std::string(di->path());
       if(current_path.find("rosbag2")!=std::string::npos){
        std::cout << current_path<< std::endl;
        folder.push_back(current_path+"/");
       }
    }
}  

class BagReader : public rclcpp::Node{
  public:

    BagReader(): Node("bag_reader"){

      this->declare_parameter<std::string>("bag_folder","");
      this->get_parameter("bag_folder", bag_folder);
      if(bag_folder!=""){
        get_all_bag_folder(bag_folder, bag_paths);
        if(bag_paths.size()==0)   RCLCPP_INFO(this->get_logger(), "No ros2bag!");
      }
      else{
        bag_paths.push_back(bag_path);
      }
      this->declare_parameter<std::string>("datasave_folder");
      this->get_parameter("datasave_folder", saving_folder);
      try{
        fs::create_directory(saving_folder);
      }
      catch(char* s){
        std::cout<<"can't create saving_folder! "<<std::endl;
      }

      this->declare_parameter<std::string>("pair_camera_topic","");
      this->get_parameter("pair_camera_topic", pair_camera_topic);
      this->declare_parameter<std::string>("pair_lidar_topic","");
      this->get_parameter("pair_lidar_topic", pair_lidar_topic);
      this->declare_parameter<float>("lag_time",0.0);
      this->get_parameter("lag_time", lag_time);


    }

    void save_bag_aspair(std::string bag_path, const std::string& datasave_folder, int pair_idx, const std::string& camera_topic,const std::string& lidar_topic,
                         double camera_start_thres = 2.0,double lidar_start_thres = 2.0){


      storage_options.uri = bag_path;
      converter_options.output_serialization_format = "cdr";

      auto tmp = spilt_last(bag_path,"/");
      if(tmp.find("rosbag2")!=std::string::npos){
        auto i = tmp.find("rosbag2");
        tmp = tmp.substr(i+8);
      }
      else{
          RCLCPP_INFO(this->get_logger(), "Wrong ros2bag path!");
          return ;
      }

      std::cout<<"save folder: "<<datasave_folder<<std::endl;
      try{
        fs::create_directory(datasave_folder);
      }
      catch(char* s){
        std::cout<<"can't create folder! "<<std::endl;
      }
      int camera_count = 0;
      int lidar_count = 0;
      double lidar_start_time;
      double camera_start_time;

      pcl::PointCloud<pcl::PointXYZI> total_pcl_pointcloud;
      rosbag2_cpp::Reader reader(std::make_unique<rosbag2_cpp::readers::SequentialReader>());
      reader.open(storage_options, converter_options);
      while (reader.has_next()) {
        auto bag_message = reader.read_next();
        // std::cout<<"Found topic name " << bag_message->topic_name << std::endl;
        if (bag_message->topic_name==camera_topic){
          auto index = std::find(camera_topics.begin(),camera_topics.end(),bag_message->topic_name)-camera_topics.begin();
          sensor_msgs::msg::Image msg;
          rclcpp::Serialization<sensor_msgs::msg::Image> serialization;
          rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
          serialization.deserialize_message(&extracted_serialized_msg, &msg);
          auto stamp = msg.header.stamp;
          if(camera_count==0) {
            camera_start_time = stamp.sec + stamp.nanosec/1000000000.0;
            camera_count++;
            continue;
            }
          if (camera_count>1){
            continue;
          }
          double delta_time =  stamp.sec + stamp.nanosec/1000000000.0 - camera_start_time;
          if(delta_time>=camera_start_thres){
            auto img = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
            cv::Mat img_cv = img->image;
            cv::flip(img_cv,img_cv,-1);
            std::string filename = datasave_folder + std::to_string(pair_idx+1) + ".png";
            RCLCPP_INFO(rclcpp::get_logger("camera"), "Writing png file");
            cv::imwrite(filename, img_cv);
            camera_count++;
          }
        }
        if (bag_message->topic_name==lidar_topic){
          
          sensor_msgs::msg::PointCloud2 msg;
          rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization;
          rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
          serialization.deserialize_message(&extracted_serialized_msg, &msg);
          pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pointcloud(new pcl::PointCloud<pcl::PointXYZI>);
          auto stamp = msg.header.stamp;
          if(lidar_count==0) {
            lidar_start_time = stamp.sec + stamp.nanosec/1000000000.0;
            lidar_count++;
            continue;
            }
          if (lidar_count>1&&!sum_pcd){
            continue;
          }
          double delta_time =  stamp.sec + stamp.nanosec/1000000000.0 - lidar_start_time;
          if(delta_time>=lidar_start_thres){
             pcl::fromROSMsg(msg, *pcl_pointcloud);
             total_pcl_pointcloud+=*pcl_pointcloud;
             lidar_count++;
          }
          if (lidar_count>1&&!sum_pcd&&camera_count>1) break;
        }
      }

      std::string pcd_filename = datasave_folder + std::to_string(pair_idx+1) + ".pcd";
      RCLCPP_INFO(rclcpp::get_logger("lidar"), "Writing sum pcd file");
      pcl::io::savePCDFileASCII(pcd_filename, total_pcl_pointcloud);     
    }


    void save_bags_pairdata(){
      for(int i=0; i<bag_paths.size();i++){
        auto path = bag_paths.at(i);
        std::cout<<"make pair "<<path<<std::endl;
        save_bag_aspair(path, saving_folder, i, pair_camera_topic, pair_lidar_topic,lag_time,lag_time);
        RCLCPP_INFO(this->get_logger(), "%s pair data has being saved!",path.c_str());
      }
    }

  private:

      rosbag2_storage::StorageOptions storage_options;
      rosbag2_cpp::ConverterOptions converter_options;
      std::vector<std::string> camera_topics;
      std::vector<std::string> lidar_topics;
      std::string bag_path;
      std::string bag_folder;
      std::string saving_folder;
      int sum_pcd = false;
      std::vector<std::string> bag_paths;
      std::string pair_camera_topic;
      std::string pair_lidar_topic;
      std::string pair_save_folder;
      float lag_time;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<BagReader>();

  std::cout<<"start reading!"<<std::endl;

  node->save_bags_pairdata();
  
  std::cout<<"finish reading!"<<std::endl;
  rclcpp::shutdown();
  return 0;
}