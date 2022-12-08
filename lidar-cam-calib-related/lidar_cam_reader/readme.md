# This is package for reading image or pcl data from rosbag2 

**Enviroment**
1. ros2 galactic +
2. c++17 +
3. opencv pcl

**Compile**
```bash
source {ros2_path}/setup.bash
colcon build --symlink-install
```
**Usage**
```bash
cd lidar_cam_reader/
source install/setup.bash
change parameters in launch img_pcd_reader.launch.py
ros2 launch lidar_cam_reader img_pcd_reader.py
```
**Parameters**
1. pair_lidar_topic: image topic of your bag.
2. pair_camera_topic: pcl topic of your bag.
3. bag_folder: multiple bags folder (if you want read data from multiple rosbag2 in a folder, set this, and bag_path will be ignored)
4. datasave_folder: absolute path for your data to save.
5. lag_time: extract time from bag start

**Format**
The final data folder looks like datasave_folder/pair_save_folder/image0 ,pcd0, image1,pcd1 ...