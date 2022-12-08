from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lidar_cam_reader',
            executable='img_pcd_reader',
            output='screen',
            parameters=[
                {   
                    "bag_folder":"/home/alan/Desktop/1205/",
                    "datasave_folder":"/home/alan/Desktop/1205/calib_pairs/",
                    "pair_camera_topic":"/gmsl_driver/front0",
                    "pair_lidar_topic":"/sensing/lidar/top/rectified/pointcloud",
                    "lag_time":0.5
                }
            ]
        ),])
        