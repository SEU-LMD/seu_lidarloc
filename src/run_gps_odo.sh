time=$(date "+%Y-%m-%d %H:%M:%S")
./build/gpsOdometry | tee "./log/${time}_gps_odo.log"
