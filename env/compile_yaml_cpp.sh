dir_abs_path=$(pwd)
sudo rm -rf ${dir_abs_path}/yaml-cpp-install/
mkdir -p ${dir_abs_path}/yaml-cpp-install/
cd yaml-cpp-install/
sudo rm -rf ./build/
mkdir build
cd build
cmake ../../yaml-cpp/ -DYAML_BUILD_SHARED_LIBS=OFF \
                      -DCMAKE_INSTALL_PREFIX=${dir_abs_path}/yaml-cpp-install
make -j10
sudo make install
