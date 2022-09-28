# 安装环境
python version>=3.6
```bash
   pip install -r requirements.txt
```

# 制作标定板
1. 标定板生成网页[1] https://calib.io/pages/camera-calibration-pattern-generator
2. 目前支持标定板为: 棋盘格, 对称圆形，非对称圆形（参考marker_board文件夹）
3. 根据标定场景选择合适的图形和尺寸，标定板的硬度尽量大不易被弯折

# 标定数据集
1. 通过摄像头采集标定的数据集
2. 在摄像头的不同角度，距离拍摄标定板图片,尽可能保证标定板分布的多样性，保证图片不发生弯曲
3. 将采集的图片保存在一个文件夹下
4. 默认不需要人为挑选图片，但是挑选拍摄清晰，不同分布的图片有助于提高标定精度

# 运行标定程序
## run (以maker_board中的io_circlesA_1200x800_16x29_50_20为例)
```bash
   python intrinsic_calib.py -R 8 -C 29 -d 0.07 --save_result 1 --save_folder "your save path" --calib_folder "your sample path" -M "Aellipse" -i 2 -n 10
```
   运行完程序，会在保存文件夹下生成intrinisic.yaml的内参文件，及标定矫正畸变后的图片
   
## 参数含义
   -R 是标定点的行，-C 是标定板的列， -d 是标定点之间的间隔 -M是标定方法 
   
   -i 是读取图片的间隔（采集图片会有重复，加快标定速度）-n（使用有效标定图片数）
   
   -M 包含 circle，ellipse，Acircle,Aellipse,chess_board（圆形，椭圆非对称圆形，非对此椭圆，棋盘格），圆形和椭圆形都使用圆形标定板，不过采用的标定方法不同
   
   -R,-C,-d 的含义参考[2] https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
   
   --brightness_thr 标定板亮度差值，--theta_thr 标定板角度差值， --centerdist_thr 标定板中心位置差值
   
   当被选图片与标定数据集中的图片的差异满足其一时被加入到标定数据集
 
## 标定优化
   一般默认采用Aellipse方法效果最好，当标定效果不太好时:
   1. 提高拍摄质量，减少图片模糊，单一的状况
   2. 如果畸变矫正不好，换成Acircle方法 
   3. 增大有效标定图片参数（增大-n）
   4. 增大数据集中标定板的分布 (拍摄不同位置角度的标定板)
   5. 调整--brightness_thr，--theta_thr，--centerdist_thr参数，根据拍摄场景，在默认参数基础上调整大小
