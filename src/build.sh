reset
mkdir build
if [ "$1" = "clear" ]; then
    sudo rm -rf ./build
    mkdir build
fi
cd build
cmake ..
make -j
cd ..

