## Environment

- Ubuntu 20.04
- OpenCV 4.2.0
- LibTorch 1.12.1+cu113
- CUDA 11.3

## Install Dependencies
1. Install CUDA 11.3

2. Install CUDNN 8.x for CUDA 11.x

3. Install in terminal
   ```
   sudo apt install ros-noetic-darknet-ros-msgs
   pip3 install pyfirmata
   pip3 install joblib
   pip3 install torch
   pip3 install scipy
   pip3 install tqdm
   pip3 install torchvision
   pip3 install pandas
   pip3 install seaborn
   pip3 install thop
   ```

## Getting Started

1. Install Package.
   ```shell
   git clone https://github.com/geonwoo-kim-2002/TVCar.git
   ```
   
2. Install LibTorch.
   ```shell
   cd ~/Downloads
   wget https://download.pytorch.org/libtorch/cu113/libtorch-cxx11-abi-shared-with-deps-1.12.1%2Bcu113.zip
   unzip libtorch-cxx11-abi-shared-with-deps-1.12.1+cu113.zip
   ```

3. Modify "`set(Torch_DIR)`" in `src/libtorch_yolov5/CMakeLists.txt` to configure LibTorch correctly.

4. Compile and run.
   ```shell
   cd ~ workspace/
   catkin build or catkin_make
   source devel/setup.bash
   ```

5. Run launch files
   ```
   roslaunch sick_scan sick_tim_7xxS.launch 
   roslaunch usb_cam usb_cam-test.launch
   roslaunch usb_cam usb_cam-test1.launch

   rosrun tracer_bringup bringup_can2usb.bash
   roslaunch tracer_bringup tracer_robot_base.launch

   roslaunch calibration calibration.launch
   roslaunch libtorch_yolov5 Libtorch_yolov5.launch
   rosrun detect_face recognition.py
   roslaunch my_cmd_vel test.launch
   ```

### Bug fix
   - `The CUDA compiler "/usr/bin/nvcc" is not able to compile a simple test program.`<br>add `set(CMAKE_CUDA_COMPILER "/usr/local/cuda-11.3/bin/nvcc")` in libtorch_yolov5/CMakeLists.txt


   - `AttributeError: module 'em' has no attribute 'RAW_OPT'`<br>
   ```shell
   pip3 install empy==3.3.4
   ```
