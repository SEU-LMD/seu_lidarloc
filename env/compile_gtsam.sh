dir_abs_path=$(pwd)
sudo rm -rf ${dir_abs_path}/gtsam-4.0.3-install/
mkdir -p ${dir_abs_path}/gtsam-4.0.3-install/
cd gtsam-4.0.3-install/
sudo rm -rf ./build/
mkdir build
cd build
cmake ../../gtsam-4.0.3/ -DCMAKE_BUILD_TYPE=Release \
                           -DCMAKE_INSTALL_PREFIX=${dir_abs_path}/gtsam-4.0.3-install \
                           -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
                           -DGTSAM_USE_SYSTEM_EIGEN=OFF \
                           -DBUILD_SHARED_LIBS=OFF \
                           -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
                           -DGTSAM_BUILD_UNSTABLE=OFF \
                           -DEigen3_DIR=${dir_abs_path}/eigen-3.4.0-install-x86/share/eigen3/cmake
make -j10                         
make install
