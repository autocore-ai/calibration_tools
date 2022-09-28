# Dependencies Installation
python version>=3.6
```bash
   pip install -r requirements.txt
```

# Calibration Board Generation
1. Refer to this free [link](https://calib.io/pages/camera-calibration-pattern-generator) to get your customerized board.
2. Supported board type: checkerboard, symmetric circle，asymmetric circle. See folder "marker_board".

# Dataset Collection
1. Launch your camera driver.
2. Collect several clear images at different angles and distances to enrich the diversity of dataset.
3. Save all the images in one folder.

# How to run
## Sample Dataset
See this [dataset](https://drive.google.com/drive/folders/1nM08yXUDFryCs7eShVZ35Z7AwhDIhZ2r) collected by developers as reference.

## Example (io_circlesA_1200x800_16x29_50_20 in maker_board)
```bash
   python3 intrinsic_calib.py -R 8 -C 29 -d 0.07 --save_result 1 --save_folder "your save path" --calib_folder "your sample path" -M "Aellipse" -i 2 -n 10
```
   After that，the calibrated result and modified image will be saved under above defined folder. Or you can see this shared link for [results](https://drive.google.com/drive/folders/1enVisQlP5RM6B48CfU72x6_6WSWDeOsA)
   
## Parameter Configuration
- R: row of caliration board
- C: column of caliration board
- d: distance between points
- See definition of R, C, d in the [link](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html)
- M: Calibration method, options: circle, ellips, Acircle, Aellipse, chess_board. Here, the method "ellipse" or "Aellipse" is applied for circle calibration board as well
- i: interval to load images, to skip duplicated images
- n: total account of image to be used for calibration
 
## Hints 
    The default method "Aellipse" performs well in most cases, if not, try the following tips:
   1. Improve diversity and clarity of image
   2. Apply "Acircle" method if bad distortion correction performance 
   3. Increase parameter n to use more images
   4. Tune parameter: --brightness_thr，--theta_thr，--centerdist_thr in "intrinsic_calib.py" according to actual situation based on the default values
