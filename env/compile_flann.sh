dir_abs_path=$(pwd)
rm -rf ${dir_abs_path}/flann-1.9.2-install/
mkdir -p ${dir_abs_path}/flann-1.9.2-install/
cd flann-1.9.2/
rm -rf ./build/
mkdir build
cd build
cmake      	   -DCMAKE_BUILD_TYPE=Release \
		   -DCMAKE_PREFIX_PATH=${dir_abs_path}\
		   -DCMAKE_INSTALL_PREFIX=${dir_abs_path}/flann-1.9.2-install \
		   -DBUILD_PYTHON_BINDINGS=OFF\
		   -DBUILD_MATLAB_BINDINGS=OFF\
		   -DBUILD_EXAMPLES=OFF\
		   -DBUILD_TESTS=OFF\
		   -DBUILD_DOC=OFF\
		   -DUSE_OPENMP=OFF\
		   ..
make -j 8
make install
cp -r ${dir_abs_path}/flann-1.9.2/mycmake/* ${dir_abs_path}/flann-1.9.2-install/lib/cmake/flann/

