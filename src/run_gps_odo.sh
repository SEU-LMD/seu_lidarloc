time=$(date "+%Y-%m-%d %H:%M:%S")
./build/devel/lib/lio_sam_6axis/lio_sam_6axis_gpsOdometry | tee "./log/${time}_gps_odo.log"
