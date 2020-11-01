# Dependency Setup
+ OpenCV
Following https://docs.opencv.org/master/d7/d9f/tutorial_linux_install.html (we need `opencv_contrib` for some of the feature detection functions)
```
# Install minimal prerequisites (Ubuntu 18.04 as reference)
sudo apt update && sudo apt install -y cmake g++ wget unzip
# Download and unpack sources
wget -O opencv.zip https://github.com/opencv/opencv/archive/master.zip
wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/master.zip
unzip opencv.zip
unzip opencv_contrib.zip
# Create build directory and switch into it
mkdir -p build && cd build
# Configure
cmake -DOPENCV_ENABLE_NONFREE:BOOL=ON -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib-master/modules ../opencv-master
# Build
cmake --build .
```

## Usage
+ `rosrun computer_vision feature_detection` (will only use 2 hardcoded frames)

For later:
+ `roslaunch openni_launch openni.launch` (or play bagfile)
+ `rosrun computer_vision computer_vision`

## Camera calibration
+ Create a checkerboard ([reference](https://markhedleyjones.com/projects/calibration-checkerboard-collection))
    + Checkerboard used: A4 - 25mm squares - 8x6 verticies, 9x7 squares
+ Set up kinect drivers: Clone this repo & catkin build https://github.com/ros-drivers/openni_camera
+ Install package to calibrate camera: Run `rosdep install camera_calibration`
+ Start the camera node: Run `roslaunch openni_launch openni.launch`
+ Start the camera calibration node: Run `rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.025 image:=/camera/rgb/image_raw camera:=/camera/rgb` ([reference](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration))


#### Recording Bagfiles
+ `rosbag record -j /camera/rgb/camera_info /camera/rgb/image_raw -O raw-img-bagfile1`

