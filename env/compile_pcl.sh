dir_abs_path=$(pwd)
rm -rf ${dir_abs_path}/pcl-1.10.0-install/
mkdir -p ${dir_abs_path}/pcl-1.10.0-install/
cd pcl-1.10.0/
rm -rf ./build/
mkdir build
cd build
cmake				-DCMAKE_BUILD_TYPE=Release \
				-DCMAKE_INSTALL_PREFIX=${dir_abs_path}/pcl-1.10.0-install \
				-DWITH_QT=FALSE \
				-DWITH_LIBUSB=false \
				-DWITH_PNG=FALSE \
				-DWITH_QHULL=FALSE \
				-DWITH_CUDA=FALSE \
				-DWITH_VTK=FALSE \
				-DWITH_OPENGL=FALSE \
				-DWITH_OPENNI=FALSE \
				-DWITH_OPENNI2=FALSE \
				-DWITH_PCAP=FALSE\
				-DWITH_ENSENSO=FALSE\
				-DWITH_VTK=FALSE\
				..
make -j 8
make install                           

