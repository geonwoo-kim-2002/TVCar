## Environment

- Ubuntu 20.04
- OpenCV 4.2.0
- LibTorch 1.12.1+cu113
- CUDA 11.3

## Getting Started YOLO

1. Install Package.
   ```shell
   git clone https://github.com/geonwoo-kim-2002/TVCar.git
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

### Bug fix
   - `The CUDA compiler "/usr/bin/nvcc" is not able to compile a simple test program.`
   add `set(CMAKE_CUDA_COMPILER "/usr/local/cuda-11.3/bin/nvcc")` in libtorch_yolov5/CMakeLists.txt


   - `AttributeError: module 'em' has no attribute 'RAW_OPT'`
  
   ```shell
   pip3 install empy==3.3.4
   ```
