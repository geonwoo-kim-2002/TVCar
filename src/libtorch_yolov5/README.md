# YOLOv5-LibTorch
Real time object detection with deployment of YOLOv5 through LibTorch C++ API

### Environment

- Ubuntu 20.04
- OpenCV 4.2.0
- LibTorch 1.12.1+cu113
- CUDA 11.3

### Getting Started

1. Install Package.

   ```shell
   git clone https://github.com/Parkeunseok/Libtorch_yolov5.git
   ```

2. Install LibTorch.

   ```shell
   wget https://download.pytorch.org/libtorch/cu113/libtorch-cxx11-abi-shared-with-deps-1.12.1%2Bcu113.zip
   unzip libtorch-cxx11-abi-shared-with-deps-1.12.1+cu113.zip
   ```

3. Edit "CMakeLists.txt" to configure OpenCV and LibTorch correctly.



4. Compile and run.

   ```shell
   cd ~ workspace/
   catkin build or catkin_make
   source devel/setup.bash
   roslaunch Libtorch_yolov5 Libtorch_yolov5.launch 
   ```
   
5. Weights file share
    ```shell
    https://drive.google.com/drive/folders/0AMhURE7Yf6NwUk9PVA
   ``` 
