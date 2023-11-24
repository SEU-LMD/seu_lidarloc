reset
mkdir build
if [ "$1" = "clear" ]; then
     rm -rf ./build
    mkdir build
fi
cd build
cmake -DCOMPILE_MODE=MDC ..
make -j10
cd ..

