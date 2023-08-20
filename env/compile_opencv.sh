dir_abs_path=$(pwd)
sudo rm -rf ${dir_abs_path}/opencv-3.4.16-install/
mkdir -p ${dir_abs_path}/opencv-3.4.16-install/
cd opencv-3.4.16-install/
sudo rm -rf ./build/
mkdir build
cd build
cmake ../../opencv-3.4.16/ -DCMAKE_BUILD_TYPE=Release \
                           -DCMAKE_INSTALL_PREFIX=${dir_abs_path}/opencv-3.4.16-install \
                           -DOPENCV_EXTRA_MODULES=${dir_abs_path}/opencv-3.4.16/opencv_contrib-3.4.16/modules \
                           -DWITH_QT=false \
                           -DBUILD_EXAMPLES=false \
                           -DBUILD_PREF_TESTS=false \
                           -DBUILD_TESTS=false  \
                           -DBUILD_SHARED_LIBS=OFF \
                           -DWITH_EIGEN=OFF \
                           -DBUILD_SHARED_LIBS=OFF
make -j8
sudo make install
