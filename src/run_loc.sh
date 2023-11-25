mkdir flag_gnss
rm -rf ./flag_rmAlldata
time=$(date "+%Y-%m-%d %H:%M:%S")
./build/loc | tee "./log/${time}_loc.log"
