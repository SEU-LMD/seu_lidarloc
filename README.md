# SEU_LIDAR_LOC

For the old king

## 0.Develop Dairy

8-20 

compile bug : eigen3.4.0 comflict with gtsam, move eigen3.4.0 into the gtsam

compile bug: geolib miss the geolib.hpp, download it from github

compile warning: pcl version may cause bug.


## 1. envLib compile and requirements

We need these libraries in our code: **Eigen3.4.0, GTSAM 4.0.3, Opencv3.4,yaml-cpp**

Firstly, you should enter into `env` folder and compile those libs in order, especially for eigen and gtsam:

```
sh compile_eigen.sh
sh compile_gtsam.sh
sh compile_opencv.sh
sh compile_yaml_cpp.sh
```

Then, enter into the `src` folder to compile the code: ./build.sh


## 2. How to run

Firstly, correct **bag's path** and **ROS Node topics, if necessary** ,in `run.launch`. Then enter into `src/build` folder, please run those commands:

```shell
source devel/setup.bash
roslaunch lio_sam_6axis run.launch

rosservice call /lio_sam_6axis/save_map #Please run these codes to save the map.

```
