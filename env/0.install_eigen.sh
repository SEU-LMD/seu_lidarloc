dir_abs_path=$(pwd)
sudo rm -rf ${dir_abs_path}/eigen-3.4.0-install-x86/
mkdir -p ${dir_abs_path}/eigen-3.4.0-install-x86/
cd eigen-3.4.0/
sudo rm -rf ./build/
mkdir build
cd build && cmake .. -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_INSTALL_PREFIX=${dir_abs_path}/eigen-3.4.0-install-x86
make install

