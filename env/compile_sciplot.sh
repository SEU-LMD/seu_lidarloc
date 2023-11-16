dir_abs_path=$(pwd)
rm -rf ${dir_abs_path}/sciplot-install
mkdir -p ${dir_abs_path}/sciplot-install
cd sciplot/
rm -rf ./build/
mkdir build
cd build
cmake      	   -DCMAKE_BUILD_TYPE=Release \
		   -DCMAKE_PREFIX_PATH=${dir_abs_path}\
		   -DCMAKE_INSTALL_PREFIX=${dir_abs_path}/sciplot-install \
		   ..
make -j 8
make install


