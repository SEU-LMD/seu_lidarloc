cd data_check_plot
rm -rf ./build
mkdir build
cd build
cmake ..
make -j10
cd ..
cd ..
